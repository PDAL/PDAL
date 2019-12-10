/******************************************************************************
* Copyright (c) 2017, Connor Manning (connor@hobu.co)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include "EptReader.hpp"

#include <limits>

#include "private/EptSupport.hpp"

#include "LasReader.hpp"

#include <arbiter/arbiter.hpp>
#include <nlohmann/json.hpp>

#include <pdal/GDALUtils.hpp>
#include <pdal/SrsBounds.hpp>
#include <pdal/compression/ZstdCompression.hpp>
#include <pdal/util/Algorithm.hpp>
#include "../filters/CropFilter.hpp"

namespace pdal
{

namespace
{
    const StaticPluginInfo s_info
    {
        "readers.ept",
        "EPT Reader",
        "http://pdal.io/stages/reader.ept.html",
        { "ept" }
    };

    const std::string addonFilename { "ept-addon.json" };

    Dimension::Type getRemoteType(const NL::json& dim)
    {
        try {
            const std::string typestring(dim.at("type").get<std::string>());
            const uint64_t size(dim.at("size").get<uint64_t>());
            return Dimension::type(typestring, size);
        }
        catch (const NL::json::exception&)
        {}
        return Dimension::Type::None;
    }

    Dimension::Type getCoercedType(const NL::json& dim)
    {
        if (dim.contains("scale") && dim["scale"].is_number())
            return Dimension::Type::Double;
        else if (dim.contains("type") && dim.contains("size"))
        {
            try
            {
                const std::string typestring(dim["type"].get<std::string>());
                const uint64_t size(dim["size"].get<uint64_t>());
                return Dimension::type(typestring, size);
            }
            catch (const NL::json::type_error&)
            {}
        }
        return Dimension::Type::None;
    }
}

CREATE_STATIC_STAGE(EptReader, s_info);

class EptBounds : public SrsBounds
{
public:
    static constexpr double LOWEST = (std::numeric_limits<double>::lowest)();
    static constexpr double HIGHEST = (std::numeric_limits<double>::max)();

    EptBounds() : SrsBounds(BOX3D(LOWEST, LOWEST, LOWEST,
        HIGHEST, HIGHEST, HIGHEST))
    {}
    EptBounds(const SrsBounds& b) : SrsBounds(b)
    {}
};

namespace Utils
{
    template<>
    bool fromString<EptBounds>(const std::string& s, EptBounds& bounds)
    {
        if (!fromString(s, (SrsBounds&)bounds))
            return false;

        // If we're setting 2D bounds, grow to 3D by explicitly setting
        // Z dimensions.
        if (!bounds.is3d())
        {
            BOX2D box = bounds.to2d();
            bounds.grow(box.minx, box.miny, EptBounds::LOWEST);
            bounds.grow(box.maxx, box.maxy, EptBounds::HIGHEST);
        }
        return true;
    }
}

struct EptReader::Args
{
public:
    EptBounds m_bounds;
    std::string m_origin;
    std::size_t m_threads = 0;
    double m_resolution = 0;
    std::vector<Polygon> m_polys;
    NL::json m_addons;

    NL::json m_query;
    NL::json m_headers;
    NL::json m_ogr;
};

EptReader::EptReader() : m_args(new EptReader::Args)
{}

EptReader::~EptReader()
{}

std::string EptReader::getName() const { return s_info.name; }

void EptReader::addArgs(ProgramArgs& args)
{
    args.add("bounds", "Bounds to fetch", m_args->m_bounds);
    args.add("origin", "Origin of source file to fetch", m_args->m_origin);
    args.add("threads", "Number of worker threads", m_args->m_threads);
    args.add("resolution", "Resolution limit", m_args->m_resolution);
    args.add("addons", "Mapping of addon dimensions to their output directory",
        m_args->m_addons);
    args.add("polygon", "Bounding polygon(s) to crop requests",
        m_args->m_polys).setErrorText("Invalid polygon specification. "
            "Must be valid GeoJSON/WKT");
    args.add("header", "Header fields to forward with HTTP requests",
        m_args->m_headers);
    args.add("query", "Query parameters to forward with HTTP requests",
        m_args->m_query);
    args.add("ogr", "OGR filter geometries",
        m_args->m_ogr);
}


std::string EptReader::get(const std::string path) const
{
    if (m_ep->isLocal())
        return m_ep->get(path);
    else
        return m_ep->get(path, m_headers, m_query);
}


std::vector<char> EptReader::getBinary(const std::string path) const
{
    if (m_ep->isLocal())
        return m_ep->getBinary(path);
    else
        return m_ep->getBinary(path, m_headers, m_query);
}


std::unique_ptr<arbiter::LocalHandle> EptReader::getLocalHandle(
    const std::string path) const
{
    if (m_ep->isLocal())
        return m_ep->getLocalHandle(path);
    else
        return m_ep->getLocalHandle(path, m_headers, m_query);
}


void EptReader::initialize()
{
    m_root = m_filename;

    auto& debug(log()->get(LogLevel::Debug));

    initializeHttpForwards();

    const std::string prefix("ept://");
    const std::string postfix("ept.json");
    if (Utils::startsWith(m_root, prefix))
    {
        m_root = m_root.substr(prefix.size());
    }
    if (Utils::endsWith(m_root, postfix))
    {
        m_root = m_root.substr(0, m_root.size() - postfix.size());
    }

    if (m_root.empty())
        throwError("Missing input filename");

    m_arbiter.reset(new arbiter::Arbiter());
    m_ep.reset(new arbiter::Endpoint(m_arbiter->getEndpoint(m_root)));

    const std::size_t threads((std::max)(m_args->m_threads, size_t(4)));
    if (threads > 100)
    {
        log()->get(LogLevel::Warning) << "Using a large thread count: " <<
            threads << " threads" << std::endl;
    }
    m_pool.reset(new Pool(threads));

    debug << "Endpoint: " << m_ep->prefixedRoot() << std::endl;
    try
    {
        m_info.reset(new EptInfo(parse(get("ept.json"))));
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }
    debug << "Got EPT info" << std::endl;
    debug << "SRS: " << m_info->srs() << std::endl;

    setSpatialReference(m_info->srs());

    if (!m_args->m_ogr.is_null())
    {
        auto& plist = m_args->m_polys;
        std::vector<Polygon> ogrPolys = gdal::getPolygons(m_args->m_ogr);
        plist.insert(plist.end(), ogrPolys.begin(), ogrPolys.end());
    }

    // Transform query bounds to match point source SRS.
    const SpatialReference& boundsSrs = m_args->m_bounds.spatialReference();
    if (!m_info->srs().valid() && boundsSrs.valid())
        throwError("Can't use bounds with SRS with data source that has "
            "no SRS.");
    m_queryBounds = m_args->m_bounds.to3d();
    if (boundsSrs.valid())
        gdal::reprojectBounds(m_queryBounds,
            boundsSrs.getWKT(), getSpatialReference().getWKT());

    // Transform polygons and bounds to point source SRS.
    std::vector <Polygon> exploded;
    for (Polygon& poly : m_args->m_polys)
    {
        if (!poly.valid())
            throwError("Geometrically invalid polyon in option 'polygon'.");
        poly.transform(getSpatialReference());

        std::vector<Polygon> polys = poly.polygons();
        exploded.insert(exploded.end(),
            std::make_move_iterator(polys.begin()),
            std::make_move_iterator(polys.end()));
    }
    m_args->m_polys = std::move(exploded);

    try
    {
        handleOriginQuery();
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }

    // Figure out our max depth.
    const double queryResolution(m_args->m_resolution);
    if (queryResolution)
    {
        double currentResolution =
            (m_info->bounds().maxx - m_info->bounds().minx) / m_info->span();

        debug << "Root resolution: " << currentResolution << std::endl;

        // To select the current resolution level, we need depthEnd to be one
        // beyond it - this is a non-inclusive parameter.
        ++m_depthEnd;

        while (currentResolution > queryResolution)
        {
            currentResolution /= 2;
            ++m_depthEnd;
        }

        debug << "Query resolution:  " << queryResolution << "\n";
        debug << "Actual resolution: " << currentResolution << "\n";
        debug << "Depth end: " << m_depthEnd << "\n";
    }

    debug << "Query bounds: " << m_queryBounds << "\n";
    debug << "Threads: " << m_pool->size() << std::endl;
}


void EptReader::initializeHttpForwards()
{
    const auto remap([&](StringMap& map, NL::json obj, std::string type)
    {
        if (obj.is_null())
            return;

        if (!obj.is_object())
            throwError("Invalid " + type + " parameters: expected object");

        for (const auto& entry : obj.items())
        {
            if (!entry.value().is_string())
                throwError("Invalid " + type + " parameters: "
                    "expected string->string mapping");
            map[entry.key()] = entry.value().get<std::string>();
        }
    });

    remap(m_headers, m_args->m_headers, "header");
    remap(m_query, m_args->m_query, "query");

    auto& debug(log()->get(LogLevel::Debug));
    if (!m_headers.empty())
        debug << "Using header parameters" << std::endl;
    if (!m_query.empty())
        debug << "Using query parameters" << std::endl;
}


void EptReader::handleOriginQuery()
{
    const std::string search(m_args->m_origin);

    if (search.empty())
        return;

    log()->get(LogLevel::Debug) << "Searching sources for " << search <<
        std::endl;

    const NL::json sources(parse(get("ept-sources/list.json")));
    log()->get(LogLevel::Debug) << "Fetched sources list" << std::endl;

    if (!sources.is_array())
    {
        throwError("Unexpected sources list: " + sources.dump());
    }

    if (search.find_first_not_of("0123456789") == std::string::npos)
    {
        // If the origin search is integral, then the OriginId value has been
        // specified directly.
        m_queryOriginId = std::stoll(search);
    }
    else
    {
        // Otherwise it's a file path (or part of one - for example selecting
        // by a basename or a tile ID rather than a full path is convenient).
        // Find it within the sources list, and make sure it's specified
        // uniquely enough to select only one file.
        for (size_t i = 0; i < sources.size(); ++i)
        {
            const NL::json& el = sources.at(i);
            if (el["id"].get<std::string>().find(search) != std::string::npos)
            {
                if (m_queryOriginId != -1)
                    throwError("Origin search ID is not unique.");
                m_queryOriginId = static_cast<int64_t>(i);
            }
        }
    }

    if (m_queryOriginId == -1)
    {
        throwError("Failed lookup of origin: " + search);
    }

    if (m_queryOriginId >= (int64_t)sources.size())
    {
        throwError("Invalid origin ID");
    }

    // Now that we have our OriginId value, clamp the bounds to select only the
    // data sources that overlap the selected origin.

    const NL::json found(sources.at(m_queryOriginId));

    try
    {
        BOX3D q(toBox3d(found["bounds"]));

        // Clip the bounds to the queried origin bounds.  Don't just overwrite
        // it - it's possible that both a bounds and an origin are specified.
        m_queryBounds.clip(q);

        log()->get(LogLevel::Debug) << "Query origin " << m_queryOriginId <<
            ": " << found["id"].get<std::string>() << std::endl;
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }
}

QuickInfo EptReader::inspect()
{
    QuickInfo qi;

    try
    {
        initialize();

        const BOX3D fullBounds(toBox3d(m_info->json()["boundsConforming"]));

        qi.m_bounds = fullBounds;
        qi.m_srs = m_info->srs();
        qi.m_pointCount = m_info->points();

        for (auto& el : m_info->schema())
            qi.m_dimNames.push_back(el["name"].get<std::string>());

        // If we've passed a spatial query, determine an upper bound on the
        // point count.
        if (!m_queryBounds.contains(fullBounds) || m_args->m_polys.size())
        {
            log()->get(LogLevel::Debug) <<
                "Determining overlapping point count" << std::endl;

            overlaps();

            qi.m_pointCount = 0;
            for (const auto& p : m_overlaps)
                qi.m_pointCount += p.second;
        }
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }

    qi.m_valid = true;

    return qi;
}

void EptReader::addDimensions(PointLayoutPtr layout)
{
    const NL::json& schema(m_info->schema());
    m_remoteLayout.reset(new FixedPointLayout());

    for (auto& el : schema)
    {
        const std::string name(el["name"].get<std::string>());

        // If the dimension has a scale, make sure we register it as a double
        // rather than its serialized type.  However for our local PointTables
        // which we will extract into our public-facing table, we'll need to
        // make sure we match the remote dimension schema exactly.
        const Dimension::Type remoteType = getRemoteType(el);
        const Dimension::Type coercedType = getCoercedType(el);

        log()->get(LogLevel::Debug) << "Registering dim " << name << ": " <<
            Dimension::interpretationName(coercedType) << std::endl;

        layout->registerOrAssignDim(name, coercedType);
        m_remoteLayout->registerOrAssignDim(name, remoteType);
    }

    m_remoteLayout->finalize();

    using D = Dimension::Id;

    m_dimTypes = m_remoteLayout->dimTypes();
    for (DimType& dt : m_dimTypes)
    {
        const NL::json dim(m_info->dim(m_remoteLayout->dimName(dt.m_id)));

        dt.m_xform.m_scale.m_val = dim.value("scale", 1.0);
        dt.m_xform.m_offset.m_val = dim.value("offset", 0);

        // For LAS, we allow the LAS reader to transform, so here
        // we leave the transforms default-initialized (scale 1/offset 0)
        if (m_info->dataType() != EptInfo::DataType::Laszip)
        {
            if (dt.m_id == D::X)
                m_xyzTransforms[0] = dt.m_xform;
            else if (dt.m_id == D::Y)
                m_xyzTransforms[1] = dt.m_xform;
            else if (dt.m_id == D::Z)
                m_xyzTransforms[2] = dt.m_xform;
        }
    }

    try
    {
        for (auto it : m_args->m_addons.items())
        {
            std::string dimName = it.key();
            const NL::json& val = it.value();
            std::string root(val.get<std::string>());
            if (Utils::endsWith(root, addonFilename))
            {
                root = root.substr(0, root.size() - addonFilename.size());
            }
            root = arbiter::expandTilde(root);

            const arbiter::Endpoint ep(m_arbiter->getEndpoint(root));
            try
            {
                const NL::json addonInfo(
                    NL::json::parse(ep.get(addonFilename)));
                const Dimension::Type type(getRemoteType(addonInfo));
                const Dimension::Id id(
                    layout->registerOrAssignDim(dimName, type));
                m_addons.emplace_back(new Addon(*layout, ep, id));
            }
            catch (NL::json::parse_error&)
            {
                throwError("Unable to parse EPT addon file '" +
                    addonFilename + "'.");
            }

            log()->get(LogLevel::Debug) << "Registering addon dim " <<
                dimName << ": " <<
                Dimension::interpretationName(m_addons.back()->type()) <<
                ", from " << root << std::endl;
        }
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }
}


void EptReader::ready(PointTableRef table)
{
    m_userLayout = table.layout();

    // These may not exist, in general they are only needed to track point
    // origins and ordering for an EPT writer.
    m_nodeIdDim = table.layout()->findDim("EptNodeId");
    m_pointIdDim = table.layout()->findDim("EptPointId");

    m_overlaps.clear();

    // Determine all overlapping data files we'll need to fetch.
    try
    {
        overlaps();
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }

    point_count_t overlapPoints(0);

    // Convert the key/overlap map to JSON for output as metadata.
    NL::json j;
    for (const auto& p : m_overlaps)
    {
        overlapPoints += p.second;
        j[p.first.toString()] = p.second;
    }

    log()->get(LogLevel::Debug) << "Overlap nodes: " << m_overlaps.size() <<
        std::endl;
    log()->get(LogLevel::Debug) << "Overlap points: " << overlapPoints <<
        std::endl;

    if (overlapPoints > 1e8)
    {
        log()->get(LogLevel::Warning) << overlapPoints <<
            " will be downloaded" << std::endl;
    }

    if (m_nodeIdDim != Dimension::Id::Unknown)
    {
        // If we have an EPT writer in this pipeline, serialize some metadata
        // which will be needed to create addon dimensions.
        MetadataNode meta(table.privateMetadata("ept"));
        meta.add("info", m_info->json().dump());
        meta.add("keys", j.dump());
        meta.add("step", m_hierarchyStep);
    }
}

void EptReader::overlaps()
{
    auto parseEndpoint = [this](const arbiter::Endpoint& ep,
        const std::string file)
    {
        NL::json j;
        try
        {
            j = NL::json::parse(get(file));
        }
        catch (NL::json::parse_error&)
        {
            throwError("Error parsing EPT hierarchy file '" + file + "'.");
        }
        return j;
    };

    // Determine all the keys that overlap the queried area by traversing the
    // EPT hierarchy:
    //      https://entwine.io/entwine-point-tile.html#ept-hierarchy)
    //
    // Because this may require fetching lots of JSON files, it'll run in our
    // thread pool.
    Key key;
    key.b = m_info->bounds();
    const std::string file("ept-hierarchy/" + key.toString() + ".json");

    {
        const NL::json root = parseEndpoint(*m_ep, file);
        // First, determine the overlapping nodes from the EPT resource.
        overlaps(*m_ep, m_overlaps, root, key);
        m_pool->await();
    }

    for (auto& addon : m_addons)
    {
        // Next, determine the overlapping nodes from each addon dimension.
        const NL::json root = parseEndpoint(addon->ep(), file);
        overlaps(addon->ep(), addon->hierarchy(), root, key);
        m_pool->await();
    }

    m_overlapIt = m_overlaps.begin();
}

void EptReader::overlaps(const arbiter::Endpoint& ep,
        std::map<Key, uint64_t>& target, const NL::json& hier,
        const Key& key)
{
    // If this key doesn't overlap our query
    // we can skip
    if (!key.b.overlaps(m_queryBounds))
        return;

    // Check the box of the key against our
    // query polygon(s). If it doesn't overlap,
    // we can skip
    for (auto& p: m_args->m_polys)
        if (p.disjoint(key.b))
            return;

    if (m_depthEnd && key.d >= m_depthEnd) return;

    auto it = hier.find(key.toString());
    if (it == hier.end())
        return;

    int64_t numPoints = it->get<int64_t>();

    if (numPoints == -1)
    {
        if (!m_hierarchyStep)
            m_hierarchyStep = key.d;

        // If the hierarchy points value here is -1, then we need to fetch the
        // hierarchy subtree corresponding to this root.
        m_pool->add([this, &ep, &target, key]()
        {
            const auto subRoot(parse(ep.get(
                            "ept-hierarchy/" + key.toString() + ".json")));
            overlaps(ep, target, subRoot, key);
        });
    }
    else
    {
        {
            //ABELL we could probably use a local mutex to lock the target map.
            std::lock_guard<std::mutex> lock(m_mutex);
            target[key] = static_cast<uint64_t>(numPoints);
        }

        for (uint64_t dir(0); dir < 8; ++dir)
            overlaps(ep, target, hier, key.bisect(dir));
    }
}

PointViewSet EptReader::run(PointViewPtr view)
{
#ifndef PDAL_HAVE_ZSTD
    if (m_info->dataType() == EptInfo::DataType::Zstandard)
        throwError("Cannot read Zstandard dataType: "
            "PDAL must be configured with WITH_ZSTD=On");
#endif

    // Start these at 1 to differentiate from points added by other stages,
    // which will be ignored by the EPT writer.
    uint64_t nodeId(1);

    for (const auto& entry : m_overlaps)
    {
        const Key& key(entry.first);

        log()->get(LogLevel::Debug) << "Data " << nodeId << "/" <<
            m_overlaps.size() << ": " << key.toString() << std::endl;

        m_pool->add([this, &view, &key, nodeId]()
        {
            PointId startId(0);

            if (m_info->dataType() == EptInfo::DataType::Laszip)
                startId = readLaszip(*view, key, nodeId);
            else if (m_info->dataType() == EptInfo::DataType::Binary)
                startId = readBinary(*view, key, nodeId);
#ifdef PDAL_HAVE_ZSTD
            else if (m_info->dataType() == EptInfo::DataType::Zstandard)
                startId = readZstandard(*view, key, nodeId);
#endif
            else
                throw ept_error("Unrecognized EPT dataType");

            // Read addon information after the native data, we'll possibly
            // overwrite attributes.
            for (const auto& addon : m_addons)
                readAddon(*view, key, *addon, startId);
        });

        ++nodeId;
    }

    m_pool->await();
    log()->get(LogLevel::Debug) << "Done reading!" << std::endl;

    PointViewSet views;
    views.insert(view);
    return views;
}


PointId EptReader::readLaszip(PointView& dst, const Key& key,
        const uint64_t nodeId) const
{
    // If the file is remote (HTTP, S3, Dropbox, etc.), getLocalHandle will
    // download the file and `localPath` will return the location of the
    // downloaded file in a temporary directory.  Otherwise it's a no-op.
    auto handle(getLocalHandle("ept-data/" + key.toString() + ".laz"));

    PointTable table;

    Options options;
    options.add("filename", handle->localPath());
    options.add("use_eb_vlr", true);

    LasReader reader;
    reader.setOptions(options);

    std::unique_lock<std::mutex> lock(m_mutex);
    reader.prepare(table);  // Geotiff SRS initialization is not thread-safe.
    lock.unlock();

    const auto views(reader.execute(table));

    PointId pointId(0);

    lock.lock();
    const PointId startId(dst.size());
    for (auto& src : views)
    {
        PointRef pr(*src);
        for (uint64_t i(0); i < src->size(); ++i)
        {
            pr.setPointId(i);
            process(dst, pr, nodeId, pointId);
            ++pointId;
        }
    }

    return startId;
}

PointId EptReader::processPackedData(PointView& dst, const uint64_t nodeId,
    char* data, const uint64_t size) const
{
    ShallowPointTable table(*m_remoteLayout, data, size);
    PointRef pr(table);

    std::lock_guard<std::mutex> lock(m_mutex);

    const PointId startId(dst.size());
    for (PointId pointId(0); pointId < table.numPoints(); ++pointId)
    {
        pr.setPointId(pointId);
        process(dst, pr, nodeId, pointId);
    }

    return startId;
}

PointId EptReader::readBinary(PointView& dst, const Key& key,
        const uint64_t nodeId) const
{
    auto data(getBinary("ept-data/" + key.toString() + ".bin"));
    return processPackedData(dst, nodeId, data.data(), data.size());
}

#ifdef PDAL_HAVE_ZSTD
uint64_t EptReader::readZstandard(PointView& dst, const Key& key,
        const uint64_t nodeId) const
{
    auto compressed(m_ep->getBinary("ept-data/" + key.toString() + ".zst"));
    std::vector<char> data;
    pdal::ZstdDecompressor dec([&data](char* pos, std::size_t size)
    {
        data.insert(data.end(), pos, pos + size);
    });

    dec.decompress(compressed.data(), compressed.size());
    return processPackedData(dst, nodeId, data.data(), data.size());
}
#endif

void EptReader::process(PointView& dst, PointRef& pr, const uint64_t nodeId,
        const PointId pointId) const
{
    using D = Dimension::Id;

    const point_count_t dstId(dst.size());

    const double x = pr.getFieldAs<double>(D::X) *
        m_xyzTransforms[0].m_scale.m_val + m_xyzTransforms[0].m_offset.m_val;
    const double y = pr.getFieldAs<double>(D::Y) *
        m_xyzTransforms[1].m_scale.m_val + m_xyzTransforms[1].m_offset.m_val;
    const double z = pr.getFieldAs<double>(D::Z) *
        m_xyzTransforms[2].m_scale.m_val + m_xyzTransforms[2].m_offset.m_val;

    const bool selected = m_queryOriginId == -1 ||
        pr.getFieldAs<int64_t>(D::OriginId) == m_queryOriginId;

    auto passesPolyFilter = [this](double x, double y)
    {
        if (m_args->m_polys.empty())
            return true;

        for (Polygon& poly : m_args->m_polys)
            if (poly.contains(x, y))
                return true;
        return false;
    };

    if (selected && m_queryBounds.contains(x, y, z) && passesPolyFilter(x, y))
    {
        dst.setField(Dimension::Id::X, dstId, x);
        dst.setField(Dimension::Id::Y, dstId, y);
        dst.setField(Dimension::Id::Z, dstId, z);

        for (const DimType& dt : m_dimTypes)
        {
            if (dt.m_id != D::X && dt.m_id != D::Y && dt.m_id != D::Z)
            {
                const double d = pr.getFieldAs<double>(dt.m_id) *
                    dt.m_xform.m_scale.m_val + dt.m_xform.m_offset.m_val;

                dst.setField(dt.m_id, dstId, d);
            }
        }

        dst.setField(m_nodeIdDim, dstId, nodeId);
        dst.setField(m_pointIdDim, dstId, pointId);
    }
}


void EptReader::readAddon(PointView& dst, const Key& key, const Addon& addon,
    const PointId pointId) const
{
    PointId np(addon.points(key));
    if (!np)
    {
        // If our addon has no points, then we are reading a superset of this
        // addon, in which case we need to zero-fill this dimension.
        //
        // For example, an EPT reader/writer loop may be performed with a
        // spatially-bounded or resolution-limited query.  Then, this partial
        // addon may be mapped to a well-known dimension, like Classification,
        // for an EPT-read of the full dataset.  If the native EPT set already
        // contains Classification, then we should overwrite it with zeroes
        // where the addon leaves off.

        std::lock_guard<std::mutex> lock(m_mutex);

        np = m_overlaps.at(key);
        for (PointId id(pointId); id < pointId + np; ++id)
        {
            dst.setField(addon.id(), id, 0);
        }

        return;
    }

    // If the addon hierarchy exists, it must match the EPT data.
    if (np != m_overlaps.at(key))
        throwError("Invalid addon hierarchy");

    const auto data(addon.ep().getBinary(
                "ept-data/" + key.toString() + ".bin"));
    const size_t dimSize(Dimension::size(addon.type()));

    if (np * dimSize != data.size())
    {
        throwError("Invalid addon content length");
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    const char* pos(data.data());
    for (PointId id(pointId); id < pointId + np; ++id)
    {
        dst.setField(addon.id(), addon.type(), id, pos);
        pos += dimSize;
    }
}


Dimension::Type EptReader::getRemoteTypeTest(const NL::json& j)
{
    return getRemoteType(j);
}

Dimension::Type EptReader::getCoercedTypeTest(const NL::json& j)
{
    return getCoercedType(j);
}

struct EptReader::NodeBuffer
{
    NodeBuffer(PointLayout& layout) : table(layout), view(table) { }
    VectorPointTable table;
    PointView view;
};

void EptReader::load()
{
    // Asynchronously trigger the fetching and point-view execution of
    // a lookahead buffer of nodes.
    while (
        m_upcomingNodeBuffers.size() < m_pool->size() &&
        m_overlapIt != m_overlaps.end())
    {
        const auto nodeId(m_nodeId++);
        const auto key(m_overlapIt->first);
        ++m_overlapIt;

        log()->get(LogLevel::Debug) << nodeId << "/" << m_overlaps.size() <<
            std::endl;

        // Insert an empty placeholder node to keep track of the outstanding
        // nodes that are currently being fetched/executed.
        std::unique_lock<std::mutex> lock(m_mutex);
        m_upcomingNodeBuffers.emplace_front();
        auto& loadingBuffer = m_upcomingNodeBuffers.front();
        lock.unlock();

        m_pool->add([this, &loadingBuffer, nodeId, key]()
        {
            std::unique_ptr<NodeBuffer> nodeBuffer(
                new NodeBuffer(*m_userLayout));

            if (m_info->dataType() == EptInfo::DataType::Laszip)
                readLaszip(nodeBuffer->view, key, nodeId);
#ifdef PDAL_HAVE_ZSTD
            else if (m_info->dataType() == EptInfo::DataType::Zstandard)
                readZstandard(nodeBuffer->view, key, nodeId);
#endif
            else if (m_info->dataType() == EptInfo::DataType::Binary)
                readBinary(nodeBuffer->view, key, nodeId);
            else
                throw ept_error("Unrecognized EPT dataType");

            for (const auto& addon : m_addons)
                readAddon(nodeBuffer->view, key, *addon);

            std::unique_lock<std::mutex> lock(m_mutex);
            loadingBuffer = std::move(nodeBuffer);
            lock.unlock();

            // A node has been populated - notify our consumer thread.
            m_cv.notify_one();
        });
    }
}

EptReader::NodeBufferIt EptReader::findBuffer()
{
    return std::find_if(
        m_upcomingNodeBuffers.begin(),
        m_upcomingNodeBuffers.end(),
        [](const std::unique_ptr<NodeBuffer>& n)
            { return static_cast<bool>(n); });
}

bool EptReader::next()
{
    // Asynchronously trigger the loading of some nodes.
    load();

    // Now wait for a node to be populated, or for there to be no outstanding
    // nodes left, in which case we're all done.
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [this]()
    {
        return m_upcomingNodeBuffers.empty() ||
            findBuffer() != m_upcomingNodeBuffers.end();
    });

    // Grab the first completed buffer from our list and transfer it out of our
    // `upcoming` pool to make it the current active node.  If there are no
    // completed buffers in the list, then we're done with processing.
    const auto it = findBuffer();
    if (it == m_upcomingNodeBuffers.end()) return false;

    m_pointId = 0;
    m_currentNodeBuffer = std::move(*it);
    m_upcomingNodeBuffers.erase(it);

    return true;
}

bool EptReader::processOne(PointRef& point)
{
    while (!m_currentNodeBuffer)
    {
        if (!next()) return false;

        // If we have a query bounds or query polygon, it's possible that the
        // octree bounds of a node overlaps the query, but this node contains
        // no matching points for the query.  So we may have a zero point
        // buffer which we have to handle.
        if (m_currentNodeBuffer->view.empty()) m_currentNodeBuffer.reset();
    }

    auto& sourceView(m_currentNodeBuffer->view);
    const auto& layout(*m_currentNodeBuffer->table.layout());

    for (const auto& id : layout.dims())
    {
        point.setField(
            id,
            layout.dimType(id),
            sourceView.getPoint(m_pointId) + layout.dimOffset(id));
    }

    if (++m_pointId == sourceView.size()) m_currentNodeBuffer.reset();

    return true;
}

} // namespace pdal
