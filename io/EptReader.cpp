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

#include <nlohmann/json.hpp>

#include <pdal/ArtifactManager.hpp>
#include <pdal/Polygon.hpp>
#include <pdal/SrsBounds.hpp>
#include <pdal/pdal_features.hpp>
#include <pdal/util/ThreadPool.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>

#include "private/ept/Connector.hpp"
#include "private/ept/EptArtifact.hpp"
#include "private/ept/EptSupport.hpp"
#include "private/ept/TileContents.hpp"

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
    StatusWithReason fromString(const std::string& s, EptBounds& bounds)
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

struct EptReader::Private
{
public:
    std::unique_ptr<Connector> connector;
    std::unique_ptr<EptInfo> info;
    std::unique_ptr<ThreadPool> pool;
    std::unique_ptr<TileContents> currentTile;
    std::unique_ptr<Hierarchy> hierarchy;
    std::queue<TileContents> contents;
    Hierarchy::const_iterator hierarchyIter;
    AddonList addons;
    std::mutex mutex;
    std::condition_variable contentsCv;
};

EptReader::EptReader() : m_args(new EptReader::Args), m_p(new EptReader::Private),
    m_artifactMgr(nullptr)
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


void EptReader::setForwards(StringMap& headers, StringMap& query)
{
    try
    {
        if (!m_args->m_headers.is_null())
            headers = m_args->m_headers.get<StringMap>();
    }
    catch (const std::exception& err)
    {
        throwError(std::string("Error parsing 'headers': ") + err.what());
    }

    try
    {
        if (!m_args->m_query.is_null())
            query = m_args->m_query.get<StringMap>();
    }
    catch (const std::exception& err)
    {
        throwError(std::string("Error parsing 'query': ") + err.what());
    }
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
    m_p->pool.reset(new ThreadPool(threads));

    StringMap headers;
    StringMap query;
    setForwards(headers, query);
    m_p->connector.reset(new Connector(headers, query));

    try
    {
        m_p->info.reset(new EptInfo(m_filename, *m_p->connector));
        setSpatialReference(m_p->info->srs());
        m_p->addons = Addon::load(*m_p->connector, m_args->m_addons);
    }
    catch (const arbiter::ArbiterError& err)
    {
        throwError(err.what());
    }

    if (!m_args->m_ogr.is_null())
    {
        auto& plist = m_args->m_polys;
        std::vector<Polygon> ogrPolys = gdal::getPolygons(m_args->m_ogr);
        plist.insert(plist.end(), ogrPolys.begin(), ogrPolys.end());
    }

    // Transform query bounds to match point source SRS.
    const SpatialReference& boundsSrs = m_args->m_bounds.spatialReference();
    if (!m_p->info->srs().valid() && boundsSrs.valid())
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
    //reseting depthEnd if initialize() has been called before
    m_depthEnd = 0;
    if (queryResolution)
    {
        double currentResolution =
            (m_p->info->bounds().maxx - m_p->info->bounds().minx) / m_p->info->span();

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
    debug << "Threads: " << m_p->pool->size() << std::endl;
}

void EptReader::handleOriginQuery()
{
    const std::string search(m_args->m_origin);

    if (search.empty())
        return;

    log()->get(LogLevel::Debug) << "Searching sources for " << search <<
        std::endl;

    std::string filename = m_p->info->sourcesDir() + "list.json";
    NL::json sources;
    try
    {
        sources = m_p->connector->getJson(filename);
    }
    catch (const arbiter::ArbiterError& err)
    {
        throwError(err.what());
    }
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

    qi.m_bounds = m_p->info->boundsConforming();
    qi.m_srs = m_p->info->srs();
    qi.m_pointCount = m_p->info->points();

    for (auto& el : m_p->info->dims())
        qi.m_dimNames.push_back(el.first);

    // If there is a spatial query from an explicit --bounds, an origin query,
    // or polygons, then we'll limit our number of points to be an upper bound,
    // and clip our bounds to the selected region.
    if (!m_queryBounds.contains(qi.m_bounds) || m_args->m_polys.size())
    {
        log()->get(LogLevel::Debug) <<
            "Determining overlapping point count" << std::endl;

        m_p->hierarchy.reset(new Hierarchy);
        overlaps();

        // If we've passed a spatial query, determine an upper bound on the
        // point count based on the hierarchy.
        qi.m_pointCount = 0;
        for (const Overlap& overlap : *m_p->hierarchy)
            qi.m_pointCount += overlap.m_count;

        // Also clip the resulting bounds to the intersection of:
        //  - the query bounds (from an explicit bounds or an origin query)
        //  - the extents of the polygon selection
        qi.m_bounds.clip(m_queryBounds);
        if (m_args->m_polys.size())
        {
            BOX3D b;
            for (const auto& poly : m_args->m_polys)
            {
                b.grow(poly.bounds());
            }
            qi.m_bounds.clip(b);
        }
    }
    qi.m_valid = true;

    return qi;
}


void EptReader::addDimensions(PointLayoutPtr layout)
{
    for (auto& el : m_p->info->dims())
    {
        const std::string& name = el.first;
        DimType& dt = el.second;

        if (dt.m_xform.nonstandard())
            layout->registerOrAssignDim(name, Dimension::Type::Double);
        else
            layout->registerOrAssignDim(name, dt.m_type);
    }

    for (Addon& addon : m_p->addons)
        addon.setExternalId(
            layout->registerOrAssignDim(addon.name(), addon.type()));
}


// Start a thread to read an overlap.  When the data has been read,
// stick the tile on the queue and notify the main thread.
void EptReader::load(const Overlap& overlap)
{
    m_p->pool->add([this, overlap]()
        {
            // Read the tile.
            TileContents tile(overlap, *m_p->info, *m_p->connector, m_p->addons);
            tile.read();

            // Put the tile on the output queue.
            std::unique_lock<std::mutex> l(m_p->mutex);
            m_p->contents.push(std::move(tile));
            l.unlock();
            m_p->contentsCv.notify_one();
        }
    );
}


void EptReader::ready(PointTableRef table)
{
    // These may not exist, in general they are only needed to track point
    // origins and ordering for an EPT writer.
    m_nodeIdDim = table.layout()->findDim("EptNodeId");
    m_pointIdDim = table.layout()->findDim("EptPointId");

    m_p->hierarchy.reset(new Hierarchy);

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
    for (const Overlap& overlap : *m_p->hierarchy)
        overlapPoints += overlap.m_count;

    if (overlapPoints > 1e8)
    {
        log()->get(LogLevel::Warning) << overlapPoints <<
            " will be downloaded" << std::endl;
    }

    // Ten million is a silly-large number for the number of tiles.
    m_p->pool.reset(new ThreadPool(m_p->pool->numThreads()));
    m_pointId = 0;
    m_tileCount = m_p->hierarchy->size();

    // If we're running in standard mode, queue up all the requests for data.
    // In streaming mode, queue up at most 4 to avoid having a ton of data
    // show up at once. Others requests will be queued as the results
    // are handled.
    if (table.supportsView())
    {
        m_artifactMgr = &table.artifactManager();
        for (const Overlap& overlap : *m_p->hierarchy)
            load(overlap);
    }
    else
    {
        int count = 4;
        m_p->hierarchyIter = m_p->hierarchy->cbegin();
        while (m_p->hierarchyIter != m_p->hierarchy->cend() && count)
        {
            load(*m_p->hierarchyIter++);
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
    key.b = m_p->info->bounds();

    {
        m_nodeId = 1;
        std::string filename = m_p->info->hierarchyDir() + key.toString() + ".json";

        // First, determine the overlapping nodes from the EPT resource.
        overlaps(*m_p->hierarchy, m_p->connector->getJson(filename), key);
    }
    m_p->pool->await();

    // Determine the addons that exist to correspond to tiles.
    for (auto& addon : m_p->addons)
    {
        m_nodeId = 1;
        std::string filename = addon.hierarchyDir() + key.toString() + ".json";
        overlaps(addon.hierarchy(), m_p->connector->getJson(filename), key);
        m_p->pool->await();
    }
}


void EptReader::overlaps(Hierarchy& target, const NL::json& hier, const Key& key)
{
    // If this key doesn't overlap our query
    // we can skip
    if (!key.b.overlaps(m_queryBounds))
        return;

    // Check the box of the key against our
    // query polygon(s). If it doesn't overlap,
    // we can skip
    auto polysOverlap = [this, &key]()
    {
        if (m_args->m_polys.empty())
            return true;
        for (auto& p: m_args->m_polys)
            if (!p.disjoint(key.b))
                return true;
        return false;
    };

    if (!polysOverlap())
        return;

    if (m_depthEnd && key.d >= m_depthEnd)
        return;

    // If our key isn't in the hierarchy, we've totally traversed this tree
    // branch (there are no lower nodes).
    auto it = hier.find(key.toString());
    if (it == hier.end())
        return;

    int64_t numPoints(-2);  // -2 will trigger an error below
    try
    {
        numPoints = it->get<int64_t>();
    }
    catch (...)
    {}

    if (numPoints == -1)
    {
        if (!m_hierarchyStep)
            m_hierarchyStep = key.d;

        // If the hierarchy points value here is -1, then we need to fetch the
        // hierarchy subtree corresponding to this root.
        m_p->pool->add([this, &target, key]()
        {
            try
            {
                std::string filename = m_p->info->hierarchyDir() + key.toString() + ".json";
                const auto subRoot(m_p->connector->getJson(filename));
                overlaps(target, subRoot, key);
            }
            catch (const arbiter::ArbiterError& err)
            {
                throwError(err.what());
            }
        });
    }
    else if (numPoints < 0)
    {
        throwError("Invalid point count for key '" + key.toString() + "'.");
    }
    else
    {
        // Note that when processing addons, we set node IDs which may
        // not match the base hierarchy, but it doesn't matter since
        // they are never used.
        {
            std::lock_guard<std::mutex> lock(m_p->mutex);
            target.emplace(key, (point_count_t)numPoints, m_nodeId++);
        }

        for (uint64_t dir(0); dir < 8; ++dir)
            overlaps(target, hier, key.bisect(dir));
    }
}

void EptReader::checkTile(const TileContents& tile)
{
    if (tile.error().size())
    {
        m_p->pool->stop();
        throwError("Error reading tile: " + tile.error());
    }
}


bool EptReader::processPoint(PointRef& dst, const TileContents& tile)
{
    using namespace Dimension;

    BasePointTable& t = tile.table();

    // Save current point ID and increment so that we can return without
    // worrying about m_pointId being correct on exit.
    PointId pointId = m_pointId++;

    PointRef p(t, pointId);
    int64_t originId = p.getFieldAs<int64_t>(Id::OriginId);
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

    double x = p.getFieldAs<double>(Id::X);
    double y = p.getFieldAs<double>(Id::Y);
    double z = p.getFieldAs<double>(Id::Z);

    if (!m_queryBounds.contains(x, y, z) || !passesPolyFilter(x, y))
        return false;

    for (auto& el : m_p->info->dims())
    {
        DimType& dt = el.second;
        if (dt.m_id != Dimension::Id::X &&
                dt.m_id != Dimension::Id::Y &&
                dt.m_id != Dimension::Id::Z)
        {
            const double val = p.getFieldAs<double>(dt.m_id) *
                dt.m_xform.m_scale.m_val + dt.m_xform.m_offset.m_val;

            dst.setField(dt.m_id, val);
        }
    }
    dst.setField(Id::X, x);
    dst.setField(Id::Y, y);
    dst.setField(Id::Z, z);
    dst.setField(m_nodeIdDim, tile.nodeId());
    dst.setField(m_pointIdDim, pointId);
    for (Addon& addon : m_p->addons)
    {
        Dimension::Id srcId = addon.localId();
        BasePointTable *t = tile.addonTable(srcId);
        if (t)
        {
            PointRef addonPoint(*t, pointId);
            double val = addonPoint.getFieldAs<double>(srcId);
            dst.setField(addon.externalId(), val);
        }
    }
    return true;
}


point_count_t EptReader::read(PointViewPtr view, point_count_t count)
{
#ifndef PDAL_HAVE_ZSTD
    if (m_p->info->dataType() == EptInfo::DataType::Zstandard)
        throwError("Cannot read Zstandard dataType: "
            "PDAL must be configured with WITH_ZSTD=On");
#endif

    point_count_t numRead = 0;

    // Pop tiles until there are no more, or wait for them to appear.
    // Exit when we've handled all the tiles or we've read enough points.
    do
    {
        std::unique_lock<std::mutex> l(m_p->mutex);
        if (m_p->contents.size())
        {
            TileContents tile = std::move(m_p->contents.front());
            m_p->contents.pop();
            l.unlock();
            checkTile(tile);
            process(view, tile, count - numRead);
            numRead += tile.size();
            m_tileCount--;
        }
        else
            m_p->contentsCv.wait(l);
    } while (m_tileCount && numRead <= count);

    // Wait for any running threads to finish and don't start any others.
    // Only relevant if we hit the count limit before reading all the tiles.
    m_p->pool->stop();

    // If we're using the addon writer, transfer the info and hierarchy
    // to that stage.
    if (m_nodeIdDim != Dimension::Id::Unknown)
    {
        EptArtifactPtr artifact
            (new EptArtifact(std::move(m_p->info), std::move(m_p->hierarchy),
                std::move(m_p->connector), m_hierarchyStep));
        m_artifactMgr->put("ept", artifact);
    }

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
        if (count-- == 0)
            return;
        dstPoint.setPointId(dstView->size());
        processPoint(dstPoint, tile);
    }
}


bool EptReader::processOne(PointRef& point)
{
top:
    if (m_tileCount == 0)
        return false;

    // If there is no active tile, grab one off the queue and ask for
    // another if there are more.  If none are available, wait.
    if (!m_p->currentTile)
    {
        do
        {
            std::unique_lock<std::mutex> l(m_p->mutex);
            if (m_p->contents.size())
            {
                m_p->currentTile.reset(new TileContents(std::move(m_p->contents.front())));
                m_p->contents.pop();
                l.unlock();
                if (m_p->hierarchyIter != m_p->hierarchy->cend())
                    load(*m_p->hierarchyIter++);
                break;
            }
            else
                m_p->contentsCv.wait(l);
        } while (true);
        checkTile(*m_p->currentTile);
    }

    bool ok = processPoint(point, *m_p->currentTile);

    // If we've processed all the points in the current tile, pop it.
    // If we've processed all the tiles, return false to indicate that
    // we're done.
    if (m_pointId == m_p->currentTile->size())
    {
        m_pointId = 0;
        m_p->currentTile.reset();
        --m_tileCount;
    }

    // If we didn't pass a point, try again.
    if (!ok)
        goto top;

    return true;
}

} // namespace pdal
