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

#include "CopcReader.hpp"

#include <functional>
#include <limits>

#include <nlohmann/json.hpp>

#include <lazperf/readers.hpp>
#include <lazperf/vlr.hpp>

#include <pdal/Polygon.hpp>
#include <pdal/Scaling.hpp>
#include <pdal/SrsBounds.hpp>
#include <pdal/util/Charbuf.hpp>
#include <pdal/util/ThreadPool.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/SrsTransform.hpp>
#include <io/LasUtils.hpp>

#include "private/copc/Connector.hpp"
#include "private/copc/Entry.hpp"
#include "private/copc/Tile.hpp"

namespace pdal
{

namespace
{

const StaticPluginInfo s_info
{
    "readers.copc",
    "COPC Reader",
    "http://pdal.io/stages/reader.copc.html",
    { "copc" }
};

}

CREATE_STATIC_STAGE(CopcReader, s_info);

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

struct CopcReader::Args
{
public:
    SrsBounds clip;
    std::size_t threads = 0;
    double resolution = 0;
    std::vector<Polygon> polys;
    bool fixNames;

    NL::json query;
    NL::json headers;
    NL::json ogr;
};

struct CopcReader::Private
{
public:
    std::unique_ptr<ThreadPool> pool;
    std::unique_ptr<copc::Tile> currentTile;

    copc::Connector connector;
    std::queue<copc::Tile> contents;
    copc::Hierarchy hierarchy;
    LasUtils::LoaderDriver loader;
    std::mutex mutex;
    std::condition_variable contentsCv;
    std::condition_variable consumedCv;
    std::vector<PolyXform> polys;
    BoxXform clip;
    int depthEnd;
    ExtraDims extraDims;
    BOX3D rootNodeExtent;
    Scaling scaling;
    uint64_t tileCount;
    int32_t tilePointNum;
    lazperf::header14 header;
    lazperf::copc_info_vlr copc_info;
};

CopcReader::CopcReader() : m_args(new CopcReader::Args), m_p(new CopcReader::Private)
{}


CopcReader::~CopcReader()
{}


std::string CopcReader::getName() const
{
    return s_info.name;
}


void CopcReader::addArgs(ProgramArgs& args)
{
    args.add("bounds", "Retangular clip region", m_args->clip);
    args.add("requests", "Number of worker threads", m_args->threads, (size_t)15);
    args.addSynonym("requests", "threads");
    args.add("resolution", "Resolution limit", m_args->resolution);
    args.add("polygon", "Bounding polygon(s) to crop requests",
        m_args->polys).setErrorText("Invalid polygon specification. Must be valid GeoJSON/WKT");
    args.add("header", "Header fields to forward with HTTP requests", m_args->headers);
    args.add("query", "Query parameters to forward with HTTP requests", m_args->query);
    args.add("ogr", "OGR filter geometries", m_args->ogr);
    args.add("fix_dims", "Make invalid dimension names valid by changing invalid "
        "characters to '_'", m_args->fixNames, true);
}


void CopcReader::setForwards(StringMap& headers, StringMap& query)
{
    try
    {
        if (!m_args->headers.is_null())
            headers = m_args->headers.get<StringMap>();
    }
    catch (const std::exception& err)
    {
        throwError(std::string("Error parsing 'headers': ") + err.what());
    }

    try
    {
        if (!m_args->query.is_null())
            query = m_args->query.get<StringMap>();
    }
    catch (const std::exception& err)
    {
        throwError(std::string("Error parsing 'query': ") + err.what());
    }
}


void CopcReader::initialize()
{
    const std::size_t threads((std::max)(m_args->threads, size_t(4)));
    if (threads > 100)
        log()->get(LogLevel::Warning) << "Using a large thread count: " <<
            threads << " threads" << std::endl;
    m_p->pool.reset(new ThreadPool(threads));

    StringMap headers;
    StringMap query;
    setForwards(headers, query);
    m_p->connector =  copc::Connector(m_filename, headers, query);

    fetchHeader();

    using namespace std::placeholders;
    LasUtils::VlrCatalog catalog(std::bind(&CopcReader::fetch, this, _1, _2));
    catalog.load(m_p->header.header_size, m_p->header.vlr_count, m_p->header.evlr_offset,
        m_p->header.evlr_count);
    fetchEbVlr(catalog);
    fetchSrsVlr(catalog);

    createSpatialFilters();

    // Calculate how many levels we need to descend to meet the resolution requirement.
    if (m_args->resolution < 0)
        throwError("Can't set `resolution` to a value less than 0.");
    m_p->depthEnd = m_args->resolution ?
        (std::max)(1, (int)ceil(log2(m_p->copc_info.spacing / m_args->resolution)) + 1) :
        0;
}


std::vector<char> CopcReader::fetch(uint64_t offset, int32_t size)
{
    return m_p->connector.getBinary(offset, size);
}


void CopcReader::fetchHeader()
{
    // Read the LAS header, COPC info VLR header and COPC VLR
    std::vector<char> data = fetch(0, 549);
    Charbuf buf(data);
    std::istream in(&buf);

    m_p->header.read(in);
    m_p->scaling.m_xXform = XForm(m_p->header.scale.x, m_p->header.offset.x);
    m_p->scaling.m_yXform = XForm(m_p->header.scale.y, m_p->header.offset.y);
    m_p->scaling.m_zXform = XForm(m_p->header.scale.z, m_p->header.offset.z);

    lazperf::vlr_header copc_info_header;
    copc_info_header.read(in);

    m_p->copc_info.read(in);
    m_p->rootNodeExtent = BOX3D(
        m_p->copc_info.center_x - m_p->copc_info.halfsize,
        m_p->copc_info.center_y - m_p->copc_info.halfsize,
        m_p->copc_info.center_z - m_p->copc_info.halfsize,
        m_p->copc_info.center_x + m_p->copc_info.halfsize,
        m_p->copc_info.center_y + m_p->copc_info.halfsize,
        m_p->copc_info.center_z + m_p->copc_info.halfsize);


    validateHeader(m_p->header);
    validateVlrInfo(copc_info_header, m_p->copc_info);
}


void CopcReader::fetchSrsVlr(const LasUtils::VlrCatalog& catalog)
{
    std::vector<char> buf = catalog.fetch("LASF_Projection", 2112);
    if (buf.empty())
        return;
    setSpatialReference(std::string(buf.begin(), buf.end()));
}


void CopcReader::fetchEbVlr(const LasUtils::VlrCatalog& catalog)
{
    std::vector<char> buf = catalog.fetch("LASF_Spec", 4);
    if (buf.empty())
        return;

    if (buf.size() % ExtraBytesSpecSize != 0)
    {
        log()->get(LogLevel::Warning) << "Bad size for extra bytes VLR.  Ignoring.";
        return;
    }
    m_p->extraDims = ExtraBytesIf::toExtraDims(buf.data(), buf.size(),
        lazperf::baseCount(m_p->header.pointFormat()));
}


void CopcReader::validateHeader(const lazperf::header14& h)
{
    if (std::string(h.magic, sizeof(h.magic)) != "LASF")
        throwError("Invalid LAS header in COPC file");
    int pdrf = h.pointFormat();
    if (pdrf < 6 || pdrf > 8)
        throwError("COPC file has invalid point format '" + std::to_string(pdrf) +
            "'. Must be 6-8.");
}


void CopcReader::validateVlrInfo(const lazperf::vlr_header& h, const lazperf::copc_info_vlr& i)
{
    if (h.user_id != "copc" || h.record_id != 1)
        throwError("COPC VLR invalid. Found user ID '" + h.user_id + "' and record ID '" +
            std::to_string(h.record_id) + "'. Expected 'copc' and '1'.");
}


// Create boxes/polygons and associated transforms for spatial filters.
void CopcReader::createSpatialFilters()
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
        if (getSpatialReference().valid() && boundsSrs.valid())
            m_p->clip.xform = SrsTransform(getSpatialReference(), boundsSrs);
    }

    // Read polygons from OGR and add to the polygon list.
    if (!m_args->ogr.is_null())
    {
        auto& plist = m_args->polys;
        std::vector<Polygon> ogrPolys = gdal::getPolygons(m_args->ogr);
        plist.insert(plist.end(), ogrPolys.begin(), ogrPolys.end());
    }

    // Create transform from the point source SRS to the poly SRS.
    for (Polygon& poly : m_args->polys)
    {
        if (!poly.valid())
            throwError("Geometrically invalid polygon in option 'polygon'.");

        // Get the sub-polygons from a multi-polygon.
        std::vector<Polygon> exploded = poly.polygons();
        SrsTransform xform;
        if (poly.srsValid() && getSpatialReference().valid())
            xform.set(getSpatialReference(), poly.getSpatialReference());
        for (Polygon& p : exploded)
        {
            PolyXform ps { std::move(p), xform };
            m_p->polys.push_back(ps);
        }
    }
}


QuickInfo CopcReader::inspect()
{
    QuickInfo qi;

    initialize();

    const lazperf::header14& h = m_p->header;
    qi.m_bounds = BOX3D(h.minx, h.miny, h.minz, h.maxx, h.maxy, h.maxz);
    qi.m_srs = getSpatialReference();
    qi.m_pointCount = h.point_count;

    PointLayoutPtr layout(new PointLayout);
    addDimensions(layout);
    for (Dimension::Id dim : layout->dims())
        qi.m_dimNames.push_back(layout->dimName(dim));

    // If there is a spatial filter from an explicit --bounds, an origin query,
    // or polygons, then we'll limit our number of points to be an upper bound,
    // and clip our bounds to the selected region.
    if (hasSpatialFilter())
    {
        loadHierarchy();

        //ABELL - Fix.
        //qi.m_pointCount = m_p->hierarchy->pointCount();

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


void CopcReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims(LasUtils::pdrfDims(m_p->header.pointFormat()));

    size_t ebLen = m_p->header.ebCount();
    for (auto& dim : m_p->extraDims)
    {
        if (dim.m_size > ebLen)
            throwError("Extra byte specification exceeds point length beyond base format length.");
        ebLen -= dim.m_size;

        Dimension::Type type = dim.m_dimType.m_type;

        // There is the awful concept of unspecified extra bytes. We don't register them.
        if (type == Dimension::Type::None)
            continue;
        if (dim.m_dimType.m_xform.nonstandard())
            type = Dimension::Type::Double;
        if (m_args->fixNames)
            dim.m_name = Dimension::fixName(dim.m_name);
        dim.m_dimType.m_id = layout->registerOrAssignDim(dim.m_name, type);
    }
}


void CopcReader::ready(PointTableRef table)
{
    // Determine all overlapping data files we'll need to fetch.
    try
    {
        loadHierarchy();
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }

    m_p->loader.init(m_p->header.pointFormat(), m_p->scaling, m_p->extraDims);

    point_count_t totalPoints = 0;
    for (const copc::Entry& entry : m_p->hierarchy)
        totalPoints += entry.m_pointCount;

    if (totalPoints > 1e8)
        log()->get(LogLevel::Warning) << totalPoints << " will be downloaded" << std::endl;

    m_p->tileCount = m_p->hierarchy.size();

    m_p->pool.reset(new ThreadPool(m_p->pool->numThreads()));
    for (const copc::Entry& entry : m_p->hierarchy)
        load(entry);
}


void CopcReader::loadHierarchy()
{
    // Determine all the keys that overlap the queried area by traversing the
    // hierarchy:
    copc::Key key;

    if (!passesFilter(key))
        return;

    copc::HierarchyPage page(fetch(m_p->copc_info.root_hier_offset, m_p->copc_info.root_hier_size));

    copc::Entry entry = page.find(key);
    if (!entry.valid())
        throwError("Root hierarchy page missing root entry.");
    loadHierarchy(m_p->hierarchy, page, entry);
    m_p->pool->await();
}


void CopcReader::loadHierarchy(copc::Hierarchy& hierarchy, const copc::HierarchyPage& page,
    const copc::Entry& entry)
{
    if (entry.isDataEntry())
    {
        for (int i = 0; i < 8; ++i)
        {
            copc::Key k = entry.m_key.child(i);
            if (passesFilter(k))
            {
                copc::Entry entry = page.find(k);
                if (entry.valid())
                    loadHierarchy(hierarchy, page, entry);
            }
        }

        if (entry.m_pointCount)
        {
            std::lock_guard<std::mutex> lock(m_p->mutex);
            hierarchy.insert(entry);
        }
    }
    else // New page
    {
        m_p->pool->add([this, &hierarchy, entry]()
        {
            copc::HierarchyPage page(fetch(entry.m_offset, entry.m_byteSize));
            copc::Entry rootDataEntry = page.find(entry.m_key);
            if (!rootDataEntry.valid())
                throwError("Hierarchy page " + entry.m_key.toString() + " missing root entry.");
            loadHierarchy(hierarchy, page, rootDataEntry);
        });
    }
}


bool CopcReader::passesFilter(const copc::Key& key) const
{
    return ((m_p->depthEnd == 0 || key.d < m_p->depthEnd) && passesSpatialFilter(key));
}


bool CopcReader::passesSpatialFilter(const copc::Key& key) const
{
    const BOX3D& tileBounds = key.bounds(m_p->rootNodeExtent);

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
        {
            if (!ps.poly.disjoint(reproject(tileBounds, ps.xform)))
                return true;
        }
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


bool CopcReader::hasSpatialFilter() const
{
    return !m_p->polys.empty() || m_p->clip.box.valid();
}


void CopcReader::load(const copc::Entry& entry)
{
    m_p->pool->add([this, entry]()
        {
            // Read the tile.
            copc::Tile tile(entry, m_p->connector, m_p->header);
            tile.read();

            // Put the tile on the output queue.
            std::unique_lock<std::mutex> l(m_p->mutex);
            while (m_p->contents.size() >= (std::max)((size_t)10, m_p->pool->numThreads()))
                m_p->consumedCv.wait(l);
            m_p->contents.push(std::move(tile));
            l.unlock();
            m_p->contentsCv.notify_one();
        }
    );
}


// This code runs in a single thread, so doesn't need locking.
bool CopcReader::processPoint(const char *inbuf, PointRef& dst)
{
    using namespace Dimension;

    // Extract XYZ to check if we want this point at all.
    LeExtractor in(inbuf, m_p->header.point_record_length);

    int32_t ix, iy, iz;
    in >> ix >> iy >> iz;

    double x = m_p->scaling.m_xXform.fromScaled(ix);
    double y = m_p->scaling.m_yXform.fromScaled(iy);
    double z = m_p->scaling.m_zXform.fromScaled(iz);

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

    m_p->loader.load(dst, inbuf, m_p->header.point_record_length);

    return true;
}


point_count_t CopcReader::read(PointViewPtr view, point_count_t count)
{
    if (m_p->tileCount == 0)
        return 0;

    point_count_t numRead = 0;

    // Pop tiles until there are no more, or wait for them to appear.
    // Exit when we've handled all the tiles or we've read enough points.
    // The mutex protects the tile queue (m_p->contents).
    do
    {
        std::unique_lock<std::mutex> l(m_p->mutex);
        if (m_p->contents.size())
        {
            copc::Tile tile = std::move(m_p->contents.front());
            m_p->contents.pop();
            m_p->consumedCv.notify_one();
            l.unlock();
            checkTile(tile);
            process(view, tile, count - numRead);
            numRead += tile.size();
            m_p->tileCount--;
        }
        else
            m_p->contentsCv.wait(l);
    } while (m_p->tileCount && numRead <= count);

    return numRead;
}

void CopcReader::checkTile(const copc::Tile& tile)
{
    if (tile.error().size())
    {
        m_p->pool->stop();
        throwError("Error reading tile: " + tile.error());
    }
}


// Put the contents of a tile into the destination point view.
void CopcReader::process(PointViewPtr dstView, const copc::Tile& tile, point_count_t count)
{
    PointRef dstPoint(*dstView);
    const char *p = tile.dataPtr();
    for (PointId idx = 0; idx < tile.size(); ++idx)
    {
        if (count-- == 0)
            return;
        dstPoint.setPointId(dstView->size());
        processPoint(p, dstPoint);
        p += m_p->header.point_record_length;
    }
}


bool CopcReader::processOne(PointRef& point)
{
top:
    // If we've processed all the tiles, return false to indicate that
    // we're done.
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
                m_p->currentTile.reset(new copc::Tile(std::move(m_p->contents.front())));
                m_p->contents.pop();
                break;
            }
            else
                m_p->contentsCv.wait(l);
        } while (true);
        m_p->consumedCv.notify_one();
        checkTile(*m_p->currentTile);
        m_p->tilePointNum = 0;
    }

    const char *p = m_p->currentTile->dataPtr() +
        (m_p->tilePointNum * m_p->header.point_record_length);
    bool ok = processPoint(p, point);
    m_p->tilePointNum++;

    // If we've processed all the points in the current tile, pop it.
    if ((size_t)m_p->tilePointNum == m_p->currentTile->size())
    {
        m_p->tilePointNum = 0;
        m_p->currentTile.reset();
        --m_p->tileCount;
    }

    // If we didn't pass a point, try again.
    if (!ok)
        goto top;

    return true;
}


void CopcReader::done(PointTableRef)
{
    m_p->pool->stop();
}

} // namespace pdal
