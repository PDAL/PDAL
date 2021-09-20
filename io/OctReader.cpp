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

#include "OctReader.hpp"

#include <limits>

#include <pdal/Polygon.hpp>
#include <pdal/SrsBounds.hpp>
#include <pdal/pdal_features.hpp>
#include <pdal/util/Algorithm.hpp>
#include <pdal/util/ThreadPool.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/SrsTransform.hpp>

#include "private/octree/Connector.hpp"
#include "private/octree/Types.hpp"

namespace pdal
{

OctReader::OctReader() : m_args(new OctArgs), m_p(new OctPrivate)
{}


OctReader::~OctReader()
{}

void OctReader::addBaseArgs(ProgramArgs& args)
{
    args.add("bounds", "Bounds to fetch", m_args->clip);
    args.add("requests", "Number of worker threads", m_args->threads, (size_t)15);
    args.addSynonym("requests", "threads");
    args.add("resolution", "Resolution limit", m_args->resolution);
    args.add("polygon", "Bounding polygon(s) to crop requests",
        m_args->polys).setErrorText("Invalid polygon specification. Must be valid GeoJSON/WKT.");
    args.add("ogr", "OGR filter geometries", m_args->ogr);
}


void OctReader::initializeBase()
{
    // Create the thread pool.
    const std::size_t threads((std::max)(m_args->threads, size_t(4)));
    if (threads > 100)
        log()->get(LogLevel::Warning) << "Using a large thread count: " <<
            threads << " threads" << std::endl;
    m_p->pool.reset(new ThreadPool(threads));

    createSpatialFilters();

    // Calculate how many levels we need to descend to meet the resolution requirement.
    if (m_args->resolution < 0)
        throwError("Can't set `resolution` to a value less than 0.");
    m_p->depthEnd = m_args->resolution ?
        (std::min)(1, (int)ceil(log2(rootNodeSpacing() / m_args->resolution)) + 1) :
        0;

    log()->get(LogLevel::Debug) << "Query bounds: " << m_p->clip.box << "\n";
    log()->get(LogLevel::Debug) << "Threads: " << m_p->pool->size() << std::endl;
}


// Create boxes/polygons and associated transforms for spatial filters.
void OctReader::createSpatialFilters()
{
    // Create transformations from our source data to the bounds SRS.
    if (m_args->clip.valid())
    {
        if (m_args->clip.is2d())
        {
            m_p->clip.box = BOX3D(m_args->clip.to2d());
            m_p->clip.box.minz = (std::numeric_limits<double>::lowest)();
            m_p->clip.box.maxz = (std::numeric_limits<double>::max)();
        }
        else
            m_p->clip.box = m_args->clip.to3d();
        const SpatialReference& boundsSrs = m_args->clip.spatialReference();
        if (!getSpatialReference().valid() && boundsSrs.valid())
            throwError("Can't use bounds with SRS with data source that has no SRS.");
        if (boundsSrs.valid())
            m_p->clip.xform = SrsTransform(getSpatialReference(), boundsSrs);
    }

    // Read polygons from OGR and add to the polygon list.
    if (!m_args->ogr.is_null())
    {
        auto& plist = m_args->polys;
        std::vector<Polygon> ogrPolys = gdal::getPolygons(m_args->ogr);
        plist.insert(plist.end(), m_args->polys.begin(), m_args->polys.end());
    }

    // Create transform from the point source SRS to the poly SRS.
    for (Polygon& poly : m_args->polys)
    {
        if (!poly.valid())
            throwError("Geometrically invalid polygon in option 'polygon'.");

        // Get the sub-polygons from a multi-polygon.
        std::vector<Polygon> exploded = poly.polygons();
        SrsTransform xform;
        if (poly.srsValid())
            xform.set(getSpatialReference(), poly.getSpatialReference());
        for (Polygon& p : exploded)
        {
            PolyXform ps { std::move(p), xform };
            m_p->polys.push_back(ps);
        }
    }
}


QuickInfo OctReader::inspectBase()
{
    QuickInfo qi;

    qi.m_bounds = pointBounds();
    qi.m_srs = getSpatialReference();
    qi.m_pointCount = pointCount();
    qi.m_dimNames = dimNames();

    // If there is a spatial filter from an explicit --bounds, an origin query,
    // or polygons, then we'll limit our number of points to be an upper bound,
    // and clip our bounds to the selected region.
    if (hasSpatialFilter())
    {
        log()->get(LogLevel::Debug) <<
            "Determining overlapping point count" << std::endl;

        calcOverlaps();

        // If we've passed a spatial filter, determine an upper bound on the
        // point count based on the hierarchy.
        qi.m_pointCount = m_p->hierarchy->pointCount();

        //ABELL - This is wrong since we're not transforming the tile bounds to the
        //  SRS of each clip region, but that seems like a lot of mess for
        //  little value. Wait until someone complains. (Note that's it's a bit
        //  different from queryOverlaps or we'd just call that.)
        // Clip the resulting bounds to the intersection of:
        //  - the query bounds (from an explicit bounds or an origin query)
        //  - the extents of the polygon selection
        BOX3D b;
        b.grow(m_p->clip.box);
        for (const auto& poly : m_args->polys)
            b.grow(poly.bounds());

        if (b.valid())
            qi.m_bounds.clip(b);
    }
    qi.m_valid = true;

    return qi;
}


// Start a thread to read a tile.  When the data has been read,
// stick the tile on the queue and notify the main thread.
void OctReader::readyBase(PointTableRef table)
{
    // Determine all overlapping data files we'll need to fetch.
    try
    {
        calcOverlaps();
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }

    point_count_t downloadPoints = m_p->hierarchy->pointCount();
    if (downloadPoints > 1e8)
        log()->get(LogLevel::Warning) << downloadPoints << " will be downloaded" << std::endl;

    m_p->tileCount = m_p->hierarchy->size();
    m_p->pool.reset(new ThreadPool(m_p->pool->numThreads()));
    for (const Accessor& acc : *m_p->hierarchy)
        load(acc);
}


void OctReader::load(const Accessor& accessor)
{
    TilePtr tilep = makeTile(accessor);
    m_p->pool->add([this, &tilep]()
        {
            tilep->read();

            // Put the tile on the output queue.
            std::unique_lock<std::mutex> l(m_p->mutex);
            m_p->contents.push(std::move(tilep));
            l.unlock();
            m_p->contentsCv.notify_one();
        }
    );
}


bool OctReader::hasSpatialFilter() const
{
    return !m_p->polys.empty() || m_p->clip.box.valid();
}


bool OctReader::passesHierarchyFilter(const Key& k) const
{
    return passesSpatialFilter(k) && (m_p->depthEnd && k.d >= m_p->depthEnd);
}


// Determine if an EPT tile overlaps our query boundary
bool OctReader::passesSpatialFilter(const Key& key) const
{
    const BOX3D& tileBounds = key.bounds();

    // Reproject the tile bounds to the largest rect. solid that contains all the corners.
    auto reproject = [](BOX3D src, SrsTransform& xform) -> BOX3D
    {
        if (!xform.valid())
            return src;

        BOX3D b;
        auto reprogrow = [&b, &xform](double x, double y, double z)
        {
            xform.transform(x, y, z);
            b.grow(x, y, z);
        };

        reprogrow(src.minx, src.miny, src.minz);
        reprogrow(src.maxx, src.miny, src.minz);
        reprogrow(src.minx, src.maxy, src.minz);
        reprogrow(src.maxx, src.maxy, src.minz);
        reprogrow(src.minx, src.miny, src.maxz);
        reprogrow(src.maxx, src.miny, src.maxz);
        reprogrow(src.minx, src.maxy, src.maxz);
        reprogrow(src.maxx, src.maxy, src.maxz);
        return b;
    };

    auto boxOverlaps = [this, &reproject, &tileBounds]() -> bool
    {
        if (!m_p->clip.box.valid())
            return true;

        // If the reprojected source bounds doesn't overlap our query bounds, we're done.
        return reproject(tileBounds, m_p->clip.xform).overlaps(m_p->clip.box);
    };

    // Check the box of the key against our query polygon(s). If it doesn't overlap,
    // we can skip
    auto polysOverlap = [this, &reproject, &tileBounds]() -> bool
    {
        if (m_p->polys.empty())
            return true;

        for (auto& ps : m_p->polys)
            if (!ps.poly.disjoint(reproject(tileBounds, ps.xform)))
                return true;
        return false;
    };

    // If there's no spatial filter, we always overlap.
    if (!hasSpatialFilter())
        return true;

    // This lock is here because if a bunch of threads are using the transform
    // at the same time, it seems to get corrupted. There may be other instances
    // that need to be locked.
    std::lock_guard<std::mutex> lock(m_p->mutex);
    return boxOverlaps() && polysOverlap();
}


// Entry point for calculating overlaps given a hierarchy to fill and a root accessor.
void OctReader::baseCalcOverlaps(Hierarchy& hierarchy, const Accessor& rootAccessor)
{
    if (!passesHierarchyFilter(rootAccessor.key()))
        return;
    const HierarchyPage& page = fetchHierarchyPage(hierarchy, rootAccessor);
    if (!page.find(rootAccessor.key()).valid())
        throwError("Root hierarchy page missing root entry.");
    calcOverlaps(hierarchy, page, rootAccessor);
    m_p->pool->await();
}


void OctReader::calcOverlaps(Hierarchy& hierarchy, const HierarchyPage& page, const Accessor& acc)
{
    if (acc.isDataAccessor())
    {
        for (int i = 0; i < 8; ++i)
        {
            Key k = acc.key().bisect(i);
            const Accessor& e = page.find(k);
            if (e.valid() && passesHierarchyFilter(k))
                calcOverlaps(hierarchy, page, e);
        }
        // Unfortunately you can't move something out of a set/unordered_set in C++11.
        // C++17 has merge.
        std::lock_guard<std::mutex> lock(m_p->mutex);
        hierarchy.insert(acc.clone());
    }
    else // Sub-hierarchy accessor.
    {
        m_p->pool->add([this, &hierarchy, &acc]()
        {
            try
            {
                const HierarchyPage& page = fetchHierarchyPage(hierarchy, acc);
                if (!page.find(acc).valid())
                    throwError(std::string("Hierarchy page ") + acc.key().toString() +
                        " missing root entry.");
                calcOverlaps(hierarchy, page, acc);
            }
            catch (const arbiter::ArbiterError& err)
            {
                //ABELL - Throwing across threads is bad. Set an error that's
                //  reaped when we join the threads.
                throwError(err.what());
            }
        });
    }
}


void OctReader::checkTile(const Tile& tile)
{
    if (tile.error().size())
    {
        m_p->pool->stop();
        throwError("Error reading tile: " + tile.error());
    }
}


bool OctReader::passesPointFilter(PointRef& p, double x, double y, double z) const
{
    return passesBasePointFilter(p, x, y, z);
}


bool OctReader::passesBasePointFilter(PointRef& p, double x, double y, double z) const
{
    auto passesBoundsFilter = [this](double x, double y, double z)
    {
        if (!m_p->clip.box.valid())
            return true;
        m_p->clip.xform.transform(x, y, z);
        return m_p->clip.box.contains(x, y, z);
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

    // If there is a spatial filter, make sure it passes.
    if (hasSpatialFilter())
        if (!passesBoundsFilter(x, y, z) || !passesPolyFilter(x, y, z))
            return false;
    return true;
}


bool OctReader::processPoint(PointRef& dst, const Tile& tile)
{
    return baseProcessPoint(dst, tile);
}


// This code runs in a single thread, so doesn't need locking.
bool OctReader::baseProcessPoint(PointRef& dst, const Tile& tile)
{
    using namespace Dimension;

    PointRef p(tile.table(), m_p->tilePoints++);

    double x = p.getFieldAs<double>(Id::X);
    double y = p.getFieldAs<double>(Id::Y);
    double z = p.getFieldAs<double>(Id::Z);
    if (!passesPointFilter(p, x, y, z))
        return false;

    dst.setField(Id::X, x);
    dst.setField(Id::Y, y);
    dst.setField(Id::Z, z);
    return true;
}


point_count_t OctReader::read(PointViewPtr view, point_count_t count)
{
    return baseRead(view, count);
}


point_count_t OctReader::baseRead(PointViewPtr view, point_count_t count)
{
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
                const Tile& tile = *(m_p->contents.front());
                l.unlock();
                checkTile(tile);
                process(view, tile, count - numRead);
                numRead += tile.size();
                m_p->tileCount--;
                m_p->contents.pop();
            }
            else
                m_p->contentsCv.wait(l);
        } while (m_p->tileCount && numRead <= count);
    }

    // Wait for any running threads to finish and don't start any others.
    // Only relevant if we hit the count limit before reading all the tiles.
    m_p->pool->stop();

    return numRead;
}


// Put the contents of a tile into the destination point view.
void OctReader::process(PointViewPtr dstView, const Tile& tile, point_count_t count)
{
    m_p->tilePoints = 0;
    PointRef dstPoint(*dstView);
    for (PointId idx = 0; idx < tile.size(); ++idx)
    {
        if (count-- == 0)
            return;
        dstPoint.setPointId(dstView->size());
        processPoint(dstPoint, tile);
    }
}


bool OctReader::processOne(PointRef& point)
{
top:
    if (m_p->tileCount == 0)
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
                m_p->currentTile = std::move(m_p->contents.front());
                m_p->contents.pop();
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
    if (m_p->tilePoints == m_p->currentTile->size())
    {
        m_p->tilePoints = 0;
        m_p->currentTile.reset();
        --m_p->tileCount;
    }

    // If we didn't pass a point, try again.
    if (!ok)
        goto top;

    return true;
}

} // namespace pdal
