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
#include <pdal/private/OGRSpec.hpp>
#include <pdal/SrsBounds.hpp>
#include <pdal/pdal_features.hpp>
#include <pdal/util/ThreadPool.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/SrsTransform.hpp>

#include "private/connector/Connector.hpp"
#include "private/ept/Artifact.hpp"
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

void reprogrow(BOX3D& b, const SrsTransform& xform, double x, double y, double z)
{
    xform.transform(x, y, z);
    b.grow(x, y, z);
}

BOX3D reprojectBoundsViaCorner(BOX3D src, const SrsTransform& xform)
{
    if (!xform.valid())
        return src;

    BOX3D b;

    reprogrow(b, xform, src.minx, src.miny, src.minz);
    reprogrow(b, xform, src.maxx, src.miny, src.minz);
    reprogrow(b, xform, src.minx, src.maxy, src.minz);
    reprogrow(b, xform, src.maxx, src.maxy, src.minz);
    reprogrow(b, xform, src.minx, src.miny, src.maxz);
    reprogrow(b, xform, src.maxx, src.miny, src.maxz);
    reprogrow(b, xform, src.minx, src.maxy, src.maxz);
    reprogrow(b, xform, src.maxx, src.maxy, src.maxz);

    return b;
}

BOX3D reprojectBoundsBcbfToLonLat(BOX3D src, const SrsTransform& xform)
{
    if (!xform.valid())
        return src;

    BOX3D b = reprojectBoundsViaCorner(src, xform);

    // If the Y-values cross the equator, make sure to include the equator.
    if (src.miny < 0 && src.maxy > 0)
    {
        reprogrow(b, xform, src.minx, 0, src.minz);
        reprogrow(b, xform, src.maxx, 0, src.minz);
        reprogrow(b, xform, src.minx, 0, src.maxz);
        reprogrow(b, xform, src.maxx, 0, src.maxz);
    }

    // Round the minimum longitude up to the nearest multiple of 90 degrees.
    int x = (int) std::ceil(src.minx);
    const int remainder = std::abs(x) % 90;
    if (x < 0)
        x = -(std::abs(x) - remainder);
    else if (x > 0)
        x = x + 90 - remainder;

    // And include the reprojected bounds at every 90 degrees within the query.
    for ( ; x <= src.maxx; x += 90)
    {
        reprogrow(b, xform, x, src.miny, src.minz);
        reprogrow(b, xform, x, src.maxy, src.minz);
        reprogrow(b, xform, x, src.miny, src.maxz);
        reprogrow(b, xform, x, src.maxy, src.maxz);

        if (src.miny < 0 && src.maxy > 0)
        {
            reprogrow(b, xform, x, 0, src.minz);
            reprogrow(b, xform, x, 0, src.maxz);
        }
    }

    return b;
}

}

CREATE_STATIC_STAGE(EptReader, s_info);

struct PolyXform
{
    Polygon poly;
    SrsTransform xform;
};

struct BoxXform
{
    BOX3D box;
    SrsTransform xform;
};

struct EptReader::Args
{
public:
    SrsBounds m_bounds;
    std::string m_origin;
    std::size_t m_threads = 0;
    double m_resolution = 0;
    std::vector<Polygon> m_polys;
    NL::json m_addons;
    OGRSpec m_ogr;
    bool m_ignoreUnreadable = false;
};

struct EptReader::Private
{
public:
    std::unique_ptr<connector::Connector> connector;
    std::unique_ptr<ept::EptInfo> info;
    std::unique_ptr<ThreadPool> pool;
    std::unique_ptr<ept::TileContents> currentTile;
    std::unique_ptr<ept::Hierarchy> hierarchy;
    std::queue<ept::TileContents> contents;
    ept::AddonList addons;
    mutable std::mutex mutex;
    std::condition_variable contentsCv;
    std::vector<PolyXform> polys;
    BoxXform bounds;
    SrsTransform llToBcbfTransform;
    uint64_t depthEnd {0};    // Zero indicates selection of all depths.
    uint64_t hierarchyStep {0};
    uint64_t nodeId;
    std::atomic<bool> done;

    void overlaps(ept::Hierarchy& target, const NL::json& hier, const ept::Key& key);
    bool passesSpatialFilter(const BOX3D& tileBounds) const;
    bool hasSpatialFilter() const
    {
        return !polys.empty() || bounds.box.valid();
    }
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
    args.add("requests", "Number of worker threads", m_args->m_threads, (size_t)15);
    args.addSynonym("requests", "threads");
    args.add("resolution", "Resolution limit", m_args->m_resolution);
    args.add("addons", "Mapping of addon dimensions to their output directory", m_args->m_addons);
    args.add("polygon", "Bounding polygon(s) to crop requests",
        m_args->m_polys).setErrorText("Invalid polygon specification. "
            "Must be valid GeoJSON/WKT");
    args.add("ogr", "OGR filter geometries", m_args->m_ogr);
    args.add("ignore_unreadable", "Ignore errors for missing point data nodes",
        m_args->m_ignoreUnreadable);
}


void EptReader::initialize()
{
    auto& debug(log()->get(LogLevel::Debug));

    const std::size_t threads((std::max)(m_args->m_threads, size_t(4)));
    if (threads > 100)
        log()->get(LogLevel::Warning) << "Using a large thread count: " <<
            threads << " threads" << std::endl;
    m_p->pool.reset(new ThreadPool(threads));

    m_p->connector.reset(new connector::Connector(m_filespec));

    try
    {
        m_p->info.reset(new ept::EptInfo(m_filename, *m_p->connector));
        setSpatialReference(m_p->info->srs());
        m_p->addons = ept::Addon::load(*m_p->connector, m_args->m_addons);
    }
    catch (const arbiter::ArbiterError& err)
    {
        throwError(err.what());
    }

    std::vector<Polygon> ogrPolys = m_args->m_ogr.getPolygons();
    m_args->m_polys.insert(m_args->m_polys.end(), ogrPolys.begin(), ogrPolys.end());

    // Create transformations from our source data to the bounds SRS.
    if (m_args->m_bounds.valid())
    {
        const SpatialReference& boundsSrs = m_args->m_bounds.spatialReference();
        if (m_args->m_bounds.is2d())
        {
            if (boundsSrs.isGeographic() && !getSpatialReference().isGeographic())
                throwError("For lon/lat 'bounds', bounds must be 3D");

            m_p->bounds.box = BOX3D(m_args->m_bounds.to2d());
            m_p->bounds.box.minz = (std::numeric_limits<double>::lowest)();
            m_p->bounds.box.maxz = (std::numeric_limits<double>::max)();
        }
        else
            m_p->bounds.box = m_args->m_bounds.to3d();
        if (boundsSrs.valid() && m_p->info->srs().valid())
            m_p->bounds.xform = SrsTransform(m_p->info->srs(), boundsSrs);

        const bool sourceIsBcbf = getSpatialReference().isGeocentric();
        const bool targetIsLonLat = boundsSrs.isGeographic();

        if (sourceIsBcbf && targetIsLonLat)
        {
            const SpatialReference& llsrs = m_args->m_bounds.spatialReference();
            m_p->llToBcbfTransform.set(llsrs, getSpatialReference());
        }
    }

    // Create transform from the point source SRS to the poly SRS.
    for (Polygon& poly : m_args->m_polys)
    {
        if (!poly.valid())
            throwError("Geometrically invalid polygon in option 'polygon'.");

        // Get the sub-polygons from a multi-polygon.
        std::vector<Polygon> exploded = poly.polygons();
        SrsTransform xform;
        if (poly.srsValid() && poly.getSpatialReference().valid())
            xform.set(m_p->info->srs(), poly.getSpatialReference());
        for (Polygon& p : exploded)
        {
            PolyXform ps { std::move(p), xform };
            m_p->polys.push_back(ps);
        }
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
    //reseting depthEnd if initialize() has been called before
    m_p->depthEnd = 0;
    if (queryResolution)
    {
        double currentResolution =
            (m_p->info->bounds().maxx - m_p->info->bounds().minx) / m_p->info->span();

        debug << "Root resolution: " << currentResolution << std::endl;

        // To select the current resolution level, we need depthEnd to be one
        // beyond it - this is a non-inclusive parameter.
        ++m_p->depthEnd;

        while (currentResolution > queryResolution)
        {
            currentResolution /= 2;
            ++m_p->depthEnd;
        }

        debug << "Query resolution:  " << queryResolution << "\n";
        debug << "Actual resolution: " << currentResolution << "\n";
        debug << "Depth end: " << m_p->depthEnd << "\n";
    }

    debug << "Query bounds: " << m_p->bounds.box << "\n";
    debug << "Threads: " << m_p->pool->size() << std::endl;
}


void EptReader::handleOriginQuery()
{
    const std::string search(m_args->m_origin);

    if (search.empty())
        return;

    log()->get(LogLevel::Debug) << "Searching sources for " << search <<
        std::endl;


    // In the initial EPT version 1.0.0, a source-file summary was stored in
    // "list.json", with detailed metadata for each file being stored together
    // in chunks.  So, the list.json array entries contained both a URL (to the
    // proper chunk) and an ID (for the key into that chunk object) in order to
    // look up the detailed metadata for an entry.
    //
    // In version 1.1.0, this chunking indirection was removed, with each
    // summary entry containing a "metadataPath" key pointing to detailed
    // metadata for a single source file only.  This summary file is called
    // "manifest.json", so the older version can coexist with the new version
    // non-destructively.
    //
    // At the moment, we only care about the summary information, which contains
    // both the original file path and the bounds, and in each of these summary
    // formats, those specific entries are exactly the same.  So we just need to
    // grab either file and can use the same logic thereafter.  Prefer manifest,
    // if it exists, since it's the newer one.

    NL::json sources;
    try
    {
        sources = m_p->connector->getJson(
            m_p->info->sourcesDir() + "manifest.json");
    }
    catch (...) {}

    if (sources.is_null())
    {
        try
        {
            sources = m_p->connector->getJson(
                m_p->info->sourcesDir() + "list.json");
        }
        catch (...) {}
    }

    if (sources.is_null())
    {
        throwError("Failed to fetch input sources metadata");
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
            if (
                el.count("path") &&
                el.at("path").get<std::string>().find(search) !=
                    std::string::npos)
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
        BOX3D q(toBox3d(found.at("bounds")));
        // Bloat the query bounds slightly (we'll choose one tick of the EPT's
        // scale) to make sure we don't miss any points due to them being
        // precisely on the bounds edge.
        q.grow(
            (std::max)(
                m_p->info->dims().at("X").m_xform.m_scale.m_val,
                m_p->info->dims().at("Y").m_xform.m_scale.m_val
            ));

        if (m_p->bounds.box.valid())
            m_p->bounds.box.clip(q);
        else
            m_p->bounds.box = q;

        log()->get(LogLevel::Debug) << "Query origin " << m_queryOriginId <<
            ": " << found.at("path").get<std::string>() << std::endl;
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

    // If there is a spatial filter from an explicit --bounds, an origin query,
    // or polygons, then we'll limit our number of points to be an upper bound,
    // and clip our bounds to the selected region.
    if (m_p->hasSpatialFilter())
    {
        log()->get(LogLevel::Debug) <<
            "Determining overlapping point count" << std::endl;

        m_p->hierarchy.reset(new ept::Hierarchy);
        overlaps();

        // If we've passed a spatial filter, determine an upper bound on the
        // point count based on the hierarchy.
        qi.m_pointCount = 0;
        for (const ept::Overlap& overlap : *m_p->hierarchy)
            qi.m_pointCount += overlap.m_count;

        //ABELL - This is wrong since we're not transforming the tile bounds to the
        //  SRS of each clip region, but that seems like a lot of mess for
        //  little value. Wait until someone complains. (Note that's it's a bit
        //  different from queryOverlaps or we'd just call that.)
        // Clip the resulting bounds to the intersection of:
        //  - the query bounds (from an explicit bounds or an origin query)
        //  - the extents of the polygon selection
        BOX3D b;
        b.grow(m_p->bounds.box);
        for (const auto& poly : m_args->m_polys)
            b.grow(poly.bounds());

        if (b.valid())
            qi.m_bounds.clip(b);
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

    for (ept::Addon& addon : m_p->addons)
        addon.setExternalId(
            layout->registerOrAssignDim(addon.name(), addon.type()));
}


// Start a thread to read an overlap.  When the data has been read,
// stick the tile on the queue and notify the main thread.
void EptReader::load(const ept::Overlap& overlap)
{
    using namespace std::chrono_literals;

    m_p->pool->add([this, overlap]()
        {
            // Read the tile.
            ept::TileContents tile(overlap, *m_p->info, *m_p->connector, m_p->addons);

            tile.read();

            if (tile.error().size())
            {
                log()->get(LogLevel::Warning) << "Failed to read " <<
                    tile.key().toString() << ": " << tile.error() << std::endl;
            }

            if (tile.error().empty() || !m_args->m_ignoreUnreadable)
            {
                // Put the tile on the output queue.  Note that if the tile has
                // an error and ignoreUnreadable isn't set, this will be fatal
                // but that will occur downstream outside of this pool thread.

                while (!m_p->done)
                {
                    {
                        std::lock_guard<std::mutex> l(m_p->mutex);
                        if (m_p->contents.size() < m_p->pool->numThreads())
                        {
                            m_p->contents.push(std::move(tile));
                            break;
                        }
                    }
                    // No room on queue, sleep. Could do a condition variable but that's
                    // more complex and probably makes no difference in most cases where
                    // this would come up.
                    std::this_thread::sleep_for(50ms);
                }
            }
            else
            {
                // If the tile has an error, and the ignoreUnreadable option is
                // set, then we just skip this tile.
                std::lock_guard<std::mutex> l(m_p->mutex);
                --m_tileCount;
            }

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

    if (
        m_queryOriginId != -1 &&
        !table.layout()->hasDim(Dimension::Id::OriginId))
    {
        // In this case we can't compare the OriginId for each point since the
        // EPT data does not have that attribute saved.  We will keep the
        // spatial query to limit the data to the extents of the requested
        // origin, but if other origins overlap these extents, then their points
        // will also be included.
        m_queryOriginId = -1;

        log()->get(LogLevel::Warning) <<
            "An origin query was given but no OriginId dimension exists - " <<
            "points from other origins may be included" << std::endl;
    }

    m_p->hierarchy.reset(new ept::Hierarchy);

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
    for (const ept::Overlap& overlap : *m_p->hierarchy)
        overlapPoints += overlap.m_count;

    if (overlapPoints > 1e8)
    {
        log()->get(LogLevel::Warning) << overlapPoints <<
            " points will be downloaded" << std::endl;
    }

    m_pointId = 0;
    m_tileCount = m_p->hierarchy->size();

    // If we're running in standard mode, queue up all the requests for data.
    // In streaming mode, queue up at most 4 to avoid having a ton of data
    // show up at once. Others requests will be queued as the results
    // are handled.
    m_p->pool.reset(new ThreadPool(m_p->pool->numThreads()));
    m_p->done = false;
    for (const ept::Overlap& overlap : *m_p->hierarchy)
        load(overlap);
    if (table.supportsView())
        m_artifactMgr = &table.artifactManager();
}


void EptReader::overlaps()
{
    // Determine all the keys that overlap the queried area by traversing the
    // EPT hierarchy:
    //      https://entwine.io/entwine-point-tile.html#ept-hierarchy)
    //
    // Because this may require fetching lots of JSON files, it'll run in our
    // thread pool.
    ept::Key key;
    key.b = m_p->info->bounds();

    {
        m_p->nodeId = 1;
        std::string filename = m_p->info->hierarchyDir() + key.toString() + ".json";

        // First, determine the overlapping nodes from the EPT resource.
        m_p->overlaps(*m_p->hierarchy, m_p->connector->getJson(filename), key);
    }
    m_p->pool->await();

    // Determine the addons that exist to correspond to tiles.
    for (auto& addon : m_p->addons)
    {
        m_p->nodeId = 1;
        std::string filename = addon.hierarchyDir() + key.toString() + ".json";
        m_p->overlaps(addon.hierarchy(), m_p->connector->getJson(filename), key);
        m_p->pool->await();
    }
}


// Determine if an EPT tile overlaps our query boundary
bool EptReader::Private::passesSpatialFilter(const BOX3D& tileBounds) const
{
    auto boxOverlaps = [this, &tileBounds]() -> bool
    {
        if (!bounds.box.valid())
            return true;

        if (llToBcbfTransform.valid())
        {
            return reprojectBoundsBcbfToLonLat(bounds.box, llToBcbfTransform).overlaps(tileBounds);
        }

        // If the reprojected source bounds doesn't overlap our query bounds, we're done.
        return reprojectBoundsViaCorner(tileBounds, bounds.xform).overlaps(bounds.box);
    };

    // Check the box of the key against our query polygon(s). If it doesn't overlap,
    // we can skip
    auto polysOverlap = [this, &tileBounds]() -> bool
    {
        if (polys.empty())
            return true;

        for (auto& ps : polys)
            if (!ps.poly.disjoint(reprojectBoundsViaCorner(tileBounds, ps.xform)))
                return true;
        return false;
    };

    // If there's no spatial filter, we always overlap.
    if (!hasSpatialFilter())
        return true;

    // This lock is here because if a bunch of threads are using the transform
    // at the same time, it seems to get corrupted. There may be other instances
    // that need to be locked.
    std::lock_guard<std::mutex> lock(mutex);
    return boxOverlaps() && polysOverlap();
}


void EptReader::Private::overlaps(ept::Hierarchy& target, const NL::json& hier, const ept::Key& key)
{
    // If our key isn't in the hierarchy, we've totally traversed this tree
    // branch (there are no lower nodes).
    auto it = hier.find(key.toString());
    if (it == hier.end())
        return;

    // If our query geometry doesn't overlap the tile or we're past the end of the requested
    // depth, return.
    if (!passesSpatialFilter(key.b) || (depthEnd && key.d >= depthEnd))
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
        if (!hierarchyStep)
            hierarchyStep = key.d;

        // If the hierarchy points value here is -1, then we need to fetch the
        // hierarchy subtree corresponding to this root.
        pool->add([this, &target, key]()
        {
            try
            {
                std::string filename = info->hierarchyDir() + key.toString() + ".json";
                const auto subRoot(connector->getJson(filename));
                overlaps(target, subRoot, key);
            }
            catch (const arbiter::ArbiterError& err)
            {
                throw pdal_error(err.what());
            }
        });
    }
    else if (numPoints < 0)
    {
        throw pdal_error("Invalid point count for key '" + key.toString() + "'.");
    }
    else
    {
        // Note that when processing addons, we set node IDs which may
        // not match the base hierarchy, but it doesn't matter since
        // they are never used.
        {
            std::lock_guard<std::mutex> lock(mutex);
            target.emplace(key, (point_count_t)numPoints, nodeId++);
        }

        for (uint64_t dir(0); dir < 8; ++dir)
            overlaps(target, hier, key.bisect(dir));
    }
}

void EptReader::checkTile(const ept::TileContents& tile)
{
    if (tile.error().size())
    {
        m_p->done = true;
        // Since tasks for all tiles were added to the queue, clear them
        // before stopping the pool.
        m_p->pool->clearTasks();
        m_p->pool->stop();
        log()->get(LogLevel::Warning) <<
            "Use readers.ept.ignore_unreadable to ignore this error" <<
            std::endl;
        throwError("Error reading tile " + tile.key().toString() + ": " +
            tile.error());
    }
}


// This code runs in a single thread, so doesn't need locking.
bool EptReader::processPoint(PointRef& dst, const ept::TileContents& tile)
{
    using namespace Dimension;

    BasePointTable& t = tile.table();

    // Save current point ID and increment so that we can return without
    // worrying about m_pointId being correct on exit.
    PointId pointId = m_pointId++;

    PointRef p(t, pointId);
    if (m_queryOriginId != -1 &&
        p.getFieldAs<int64_t>(Id::OriginId)!= m_queryOriginId)
    {
        return false;
    }

    auto passesBoundsFilter = [this](double x, double y, double z)
    {
        if (!m_p->bounds.box.valid())
            return true;
        m_p->bounds.xform.transform(x, y, z);
        return m_p->bounds.box.contains(x, y, z);
    };

    auto passesPolyFilter = [this](double xo, double yo, double zo)
    {
        if (m_p->polys.empty())
            return true;

        for (PolyXform& ps : m_p->polys)
        {
            double x = xo;
            double y = yo;
            double z = zo;

            ps.xform.transform(x, y, z);
            if (ps.poly.contains(x, y))
                return true;
        }
        return false;
    };

    double x = p.getFieldAs<double>(Id::X);
    double y = p.getFieldAs<double>(Id::Y);
    double z = p.getFieldAs<double>(Id::Z);

    // If there is a spatial filter, make sure it passes.
    if (m_p->hasSpatialFilter())
        if (!passesBoundsFilter(x, y, z) || !passesPolyFilter(x, y, z))
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
    for (ept::Addon& addon : m_p->addons)
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
    if (m_p->info->dataType() == ept::EptInfo::DataType::Zstandard)
        throwError("Cannot read Zstandard dataType: "
            "PDAL must be configured with WITH_ZSTD=On");
#endif

    point_count_t numRead = 0;

    if (m_p->hierarchy->size())
    {
        // Pop tiles until there are no more, or wait for them to appear.
        // Exit when we've handled all the tiles or we've read enough points.
        do
        {
            std::unique_lock<std::mutex> l(m_p->mutex);
            if (m_p->contents.size())
            {
                ept::TileContents tile = std::move(m_p->contents.front());
                m_p->contents.pop();
                l.unlock();
                checkTile(tile);
                process(view, tile, count - numRead);
                numRead += tile.size();
                m_tileCount--;
            }
            else if (m_tileCount)
                m_p->contentsCv.wait(l);
        } while (m_tileCount && numRead <= count);
    }

    // Wait for any running threads to finish and don't start any others.
    // Only relevant if we hit the count limit before reading all the tiles.
    m_p->pool->stop();

    // If we're using the addon writer, transfer the info and hierarchy
    // to that stage.
    if (m_nodeIdDim != Dimension::Id::Unknown)
    {
        ept::ArtifactPtr artifact
            (new ept::Artifact(std::move(m_p->info), std::move(m_p->hierarchy),
                std::move(m_p->connector), m_p->hierarchyStep));
        m_artifactMgr->put("ept", artifact);
    }

    return numRead;
}


// Put the contents of a tile into the destination point view.
void EptReader::process(PointViewPtr dstView, const ept::TileContents& tile,
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

void EptReader::done(PointTableRef)
{
    m_p->pool->await();
    m_p->connector.reset();
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
                m_p->currentTile.reset(new ept::TileContents(std::move(m_p->contents.front())));
                m_p->contents.pop();
                break;
            }
            else if (!m_tileCount)
                return false;
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
