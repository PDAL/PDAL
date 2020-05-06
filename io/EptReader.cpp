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

#include "private/ept/EptSupport.hpp"
#include "private/ept/TileContents.hpp"

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

EptReader::EptReader() : m_args(new EptReader::Args), m_currentTile(nullptr)
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


void EptReader::initialize()
{
    auto& debug(log()->get(LogLevel::Debug));

    const std::size_t threads((std::max)(m_args->m_threads, size_t(4)));
    if (threads > 100)
    {
        log()->get(LogLevel::Warning) << "Using a large thread count: " <<
            threads << " threads" << std::endl;
    }
    m_pool.reset(new Pool(threads));

    m_info.reset(new EptInfo(m_filename, m_args->m_headers, m_args->m_query));
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
        auto ok = poly.transform(getSpatialReference());
        if (!ok)
            throwError(ok.what());
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

    m_info->loadAddonInfo(m_args->m_addons);
}

void EptReader::handleOriginQuery()
{
    const std::string search(m_args->m_origin);

    if (search.empty())
        return;

    log()->get(LogLevel::Debug) << "Searching sources for " << search <<
        std::endl;

    const NL::json sources(m_info->endpoint().getJson("ept-sources/list.json"));
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

    initialize();

    qi.m_bounds = m_info->boundsConforming();
    qi.m_srs = m_info->srs();
    qi.m_pointCount = m_info->points();

    for (auto& el : m_info->dims())
        qi.m_dimNames.push_back(el.first);

    // If we've passed a spatial query, determine an upper bound on the
    // point count.
    if (!m_queryBounds.contains(qi.m_bounds) || m_args->m_polys.size())
    {
        log()->get(LogLevel::Debug) <<
            "Determining overlapping point count" << std::endl;

        overlaps();

        qi.m_pointCount = 0;
        for (const auto& p : m_overlaps)
            qi.m_pointCount += p.m_count;
    }
    qi.m_valid = true;

    return qi;
}


void EptReader::addDimensions(PointLayoutPtr layout)
{
    for (auto& el : m_info->dims())
    {
        const std::string& name = el.first;
        DimType& dt = el.second;

        if (dt.m_xform.nonstandard())
            layout->registerOrAssignDim(name, Dimension::Type::Double);
        else
            layout->registerOrAssignDim(name, dt.m_type);
    }

    for (Addon& addon : m_info->addons())
        addon.setDstId(layout->registerOrAssignDim(addon.name(), addon.type()));
}


// Start a thread to read an overlap.  When the data has been read,
// stick the tile on the queue and notify the main thread.
void EptReader::load(const Overlap& overlap)
{
    m_pool->add([this, overlap]()
        {
            // Read the tile.
            TileContents tile(overlap, m_info.get());
            tile.read();

            // Put the tile on the output queue.
            std::unique_lock<std::mutex> l(m_mutex);
            m_contents.push(std::move(tile));
            l.unlock();
            m_contentsCv.notify_one();
        }
    );
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
    for (const Overlap& overlap : m_overlaps)
    {
        overlapPoints += overlap.m_count;
        j[overlap.m_key.toString()] = overlap.m_count;
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

    // A million is a silly-large number for the number of tiles.
    m_pool.reset(new Pool(m_pool->numThreads(), 1000000));
    // Start node ID at 1 to differentiate from points added by other stages,
    // which will be ignored by the EPT writer.
    m_nodeId = 1;
    m_pointId = 0;
    m_tileCount = m_overlaps.size();

    // If we're running in standard mode, queue up all the requests for data.
    // In streaming mode, queue up at most 4 to avoid having a ton of data
    // show up at once. Others requests will be queued as the results
    // are handled.
    if (table.supportsView())
    {
        for (const Overlap& overlap : m_overlaps)
            load(overlap);
        m_overlaps.clear();
    }
    else
    {
        int count = 4;
        while (m_overlaps.size() && count)
        {
            load(m_overlaps.front());
            m_overlaps.pop_front();
            count--;
        }
    }
}


void EptReader::overlaps()
{
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
        const NL::json root = m_info->endpoint().getJson(file);
        // First, determine the overlapping nodes from the EPT resource.
        overlaps(m_info->endpoint(), m_overlaps, root, key);
        m_pool->await();
    }

    // Determine the addons that exist to correspond to tiles.
    for (auto& addon : m_info->addons())
    {
        // Next, determine the overlapping nodes from each addon dimension.
        const NL::json root = addon.endpoint().getJson(file);
        overlaps(addon.endpoint(), addon.overlaps(), root, key);
    }
    m_pool->await();
}


void EptReader::overlaps(const Endpoint& ep, std::list<Overlap>& target,
    const NL::json& hier, const Key& key)
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
            const auto subRoot(ep.getJson(
                "ept-hierarchy/" + key.toString() + ".json"));
            overlaps(ep, target, subRoot, key);
        });
    }
    else if (numPoints < 0)
    {
        throwError("Invalid point count for key '" + key.toString() + "'.");
    }
    else
    {
        {
            //ABELL we could probably use a local mutex to lock the target map.
            std::lock_guard<std::mutex> lock(m_mutex);
            target.push_back({key, (point_count_t)numPoints});
        }

        for (uint64_t dir(0); dir < 8; ++dir)
            overlaps(ep, target, hier, key.bisect(dir));
    }
}

bool EptReader::processPoint(PointRef& dst, const TileContents& tile)
{
    using namespace Dimension;

    const PointViewPtr v = tile.view();

    // Save current point ID and increment so that we can return without
    // worrying about m_pointId.
    PointId pointId = m_pointId++;

    int64_t originId = v->getFieldAs<int64_t>(Id::OriginId, pointId);
    if (m_queryOriginId != -1 && originId != m_queryOriginId)
        return false;

    auto passesPolyFilter = [this](double x, double y)
    {
        if (m_args->m_polys.empty())
            return true;

        for (Polygon& poly : m_args->m_polys)
            if (poly.contains(x, y))
                return true;
        return false;
    };

    double x = v->getFieldAs<double>(Id::X, pointId);
    double y = v->getFieldAs<double>(Id::Y, pointId);
    double z = v->getFieldAs<double>(Id::Z, pointId);

    if (!m_queryBounds.contains(x, y, z) || !passesPolyFilter(x, y))
        return false;

    for (auto& el : m_info->dims())
    {
        DimType& dt = el.second;
        if (dt.m_id != Dimension::Id::X &&
                dt.m_id != Dimension::Id::Y &&
                dt.m_id != Dimension::Id::Z)
        {
            const double val = v->getFieldAs<double>(dt.m_id, pointId) *
                dt.m_xform.m_scale.m_val + dt.m_xform.m_offset.m_val;

            dst.setField(dt.m_id, val);
        }
    }
    dst.setField(Id::X, x);
    dst.setField(Id::Y, y);
    dst.setField(Id::Z, z);
    dst.setField(m_nodeIdDim, m_nodeId);
    dst.setField(m_pointIdDim, pointId);
    for (Addon& addon : m_info->addons())
    {
        Dimension::Id srcId = addon.srcId();
        BasePointTable *t = tile.addonTable(srcId);
        PointRef addonPoint(*t, pointId);
        double val = addonPoint.getFieldAs<double>(srcId);
        dst.setField(addon.dstId(), val);
    }
    return true;
}


point_count_t EptReader::read(PointViewPtr view, point_count_t count)
{
#ifndef PDAL_HAVE_ZSTD
    if (m_info->dataType() == EptInfo::DataType::Zstandard)
        throwError("Cannot read Zstandard dataType: "
            "PDAL must be configured with WITH_ZSTD=On");
#endif

    point_count_t numRead = 0;

    // Pop tiles until there are no more, or wait for them to appear.
    // Exit when we've handled all the tiles or we've read enough points.
    do
    {
        std::unique_lock<std::mutex> l(m_mutex);
        if (m_contents.size())
        {
            TileContents tile = std::move(m_contents.front());
            m_contents.pop();
            l.unlock();
            process(view, tile, count - numRead);
            numRead += tile.size();
        }
        else
            m_contentsCv.wait(l);
    } while (m_nodeId <= m_tileCount && numRead <= count);
    return numRead;
}


// Put the contents of a tile into the destination point view.
void EptReader::process(PointViewPtr dstView, const TileContents& tile,
    point_count_t count)
{
    m_pointId = 0;
    PointRef dstPoint(*dstView);
    for (PointId idx = 0; idx < tile.size(); ++idx)
    {
        dstPoint.setPointId(dstView->size());
        processPoint(dstPoint, tile);
        if (--count == 0)
            return;
    }
    m_nodeId++;
}


bool EptReader::processOne(PointRef& point)
{
top:
    // If there is no active tile, grab one off the queue and ask for
    // another if there are more.  If none are available, wait.
    if (!m_currentTile)
    {
        do
        {
            std::unique_lock<std::mutex> l(m_mutex);
            if (m_contents.size())
            {
                m_currentTile = &m_contents.front();
                l.unlock();
                if (m_overlaps.size())
                {
                    load(m_overlaps.front());
                    m_overlaps.pop_front();
                }
                break;
            }
            else
                m_contentsCv.wait(l);
        } while (true);
    }

    bool ok = processPoint(point, *m_currentTile);

    // If we've processed all the points in the current tile, pop it.
    // If we've processed all the tiles, return false to indicate that
    // we're done.
    if (m_pointId == m_currentTile->size())
    {
        m_pointId = 0;
        m_currentTile = nullptr;
        m_contents.pop();
        m_nodeId++;
        if (m_nodeId > m_tileCount)
            return false;
    }

    // If we didn't pass a point, try again.
    if (!ok)
        goto top;

    return true;
}

/**
Dimension::Type EptReader::getRemoteTypeTest(const NL::json& j)
{
    return getRemoteType(j);
}

Dimension::Type EptReader::getCoercedTypeTest(const NL::json& j)
{
    return getCoercedType(j);
}
**/

} // namespace pdal
