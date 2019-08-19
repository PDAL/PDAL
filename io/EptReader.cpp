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
#include <pdal/util/Algorithm.hpp>
#include "../filters/CropFilter.hpp"
#include "../filters/private/pnp/GridPnp.hpp"

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
};


EptReader::EptReader() : m_args(new EptReader::Args), m_currentIndex(0)
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
}


void EptReader::initialize()
{
    m_root = m_filename;

    auto& debug(log()->get(LogLevel::Debug));

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
        m_info.reset(new EptInfo(parse(m_ep->get("ept.json"))));
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }
    debug << "Got EPT info" << std::endl;
    debug << "SRS: " << m_info->srs() << std::endl;

    setSpatialReference(m_info->srs());

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
    for (Polygon& poly : m_args->m_polys)
    {
        if (!poly.valid())
            throwError("Geometrically invalid polyon in option 'polygon'.");
        poly.transform(getSpatialReference());
    }

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


void EptReader::handleOriginQuery()
{
    const std::string search(m_args->m_origin);

    if (search.empty())
        return;

    log()->get(LogLevel::Debug) << "Searching sources for " << search <<
        std::endl;

    const NL::json sources(parse(m_ep->get("ept-sources/list.json")));
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

        qi.m_bounds = toBox3d(m_info->json()["boundsConforming"]);
        qi.m_srs = m_info->srs();
        qi.m_pointCount = m_info->points();

        for (auto& el : m_info->schema())
            qi.m_dimNames.push_back(el["name"].get<std::string>());
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

    // Backup the layout for streamable pipeline.
    // Will be used to restore m_bufferPointTable layout after flushing
    // points from previous tile.
    if (pipelineStreamable())
    {
    	m_bufferLayout = layout;
        m_temp_buffer.reserve(layout->pointSize());
	}
}


void EptReader::ready(PointTableRef table)
{
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
            j = NL::json::parse(ep.get(file));
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
    // Start these at 1 to differentiate from points added by other stages,
    // which will be ignored by the EPT writer.
    uint64_t nodeId(1);

    for (auto& p : m_args->m_polys)
    {
        std::unique_ptr<GridPnp> gridPnp(new GridPnp(
            p.exteriorRing(), p.interiorRings()));
        m_queryGrids.push_back(std::move(gridPnp));
    }

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
            else
                startId = readBinary(*view, key, nodeId);

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
    auto handle(m_ep->getLocalHandle("ept-data/" + key.toString() + ".laz"));

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

PointId EptReader::readBinary(PointView& dst, const Key& key,
        const uint64_t nodeId) const
{
    auto data(m_ep->getBinary("ept-data/" + key.toString() + ".bin"));
    ShallowPointTable table(*m_remoteLayout, data.data(), data.size());
    PointRef pr(table);

    std::lock_guard<std::mutex> lock(m_mutex);

    const PointId startId(dst.size());

    PointId pointId(0);
    for (PointId pointId(0); pointId < table.numPoints(); ++pointId)
    {
        pr.setPointId(pointId);
        process(dst, pr, nodeId, pointId);
    }

    return startId;
}

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
        if (m_queryGrids.empty())
            return true;

        for (auto& grid: m_queryGrids)
            if (grid->inside(x,y))
                return true;
        return false;
    };

    if (selected && m_queryBounds.contains(x, y, z) && passesPolyFilter(x, y))
    {
        // If we were given polys, check that our point is inside those
        // polygons too.

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

void EptReader::loadNextOverlap()
{
    std::map<Key, uint64_t>::iterator itr = m_overlaps.begin();
    std::advance(itr, m_nodeId-1);
    Key key(itr->first);
    log()->get(LogLevel::Debug)
        << "Streaming Data " << m_nodeId << "/" << m_overlaps.size() << ": "
        << key.toString() << std::endl;

    uint64_t startId(0);

    // Reset PointTable to make sure all points from previous tile are flushed.
    m_bufferPointTable.reset(new PointTable());

    // We did reset PointTable,
    // So it will not have the dimensions registered to it.
    // Register/Restore Dimensions for PointTable layout.
    PointLayoutPtr layout = m_bufferPointTable->layout();
    for (auto id : m_bufferLayout->dims())
        layout->registerOrAssignDim(m_bufferLayout->dimName(id),
                                    m_bufferLayout->dimType(id));

    // Reset PointView to have new point table.
    m_bufferPointView.reset(new PointView(*m_bufferPointTable));

    if (m_info->dataType() == EptInfo::DataType::Laszip)
        startId = readLaszip(*m_bufferPointView, key, m_nodeId);
    else
        startId = readBinary(*m_bufferPointView, key, m_nodeId);

    log()->get(LogLevel::Debug) << "Points : "<< m_bufferPointView->size() <<
        std::endl;
    m_currentIndex = 0;

    // Read addon information after the native data, we'll possibly
    // overwrite attributes.
    for (const auto& addon : m_addons)
        readAddon(*m_bufferPointView, key, *addon, startId);

    m_nodeId++;
}


void EptReader::fillPoint(PointRef& point)
{
    DimTypeList dims = m_bufferPointView->dimTypes();
    m_bufferPointView->getPackedPoint(dims, m_currentIndex,
        m_temp_buffer.data());
    point.setPackedData(dims, m_temp_buffer.data());
    m_currentIndex++;
}


bool EptReader::processOne(PointRef& point)
{
    // bufferView is not set if no tile has been read and
    // currentIndex will be greater or equal to the view size when the
    // whole tile has been read
    bool finishedCurrentOverlap = !(m_bufferPointView &&
        m_currentIndex < m_bufferPointView->size());

    // We're done with all overlaps, Its time to finish reading.
    if (m_nodeId > m_overlaps.size() && finishedCurrentOverlap)
        return false;

    // Either this is a first overlap or we've streamed all points
    // from current overlap. So its time to load new overlap.
    if (finishedCurrentOverlap)
        loadNextOverlap();

    // In some rare cases there are 0 points in the overlap.
    // If this happen, fillPoint() will crash while retriving point information.
    // In that case proceed to load next overlap.
    if (m_bufferPointView->size())
        fillPoint(point);
    else
        return processOne(point);

    return true;
}

} // namespace pdal

