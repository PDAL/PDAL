/******************************************************************************
* Copyright (c) 2014, Connor Manning (connor@hobu.co)
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

#include "GreyhoundReader.hpp"

#include <sstream>

#include <pdal/pdal_macros.hpp>
#include <pdal/Compression.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.greyhound",
    "Greyhound Reader",
    "http://pdal.io/stages/readers.greyhound.html");

CREATE_SHARED_PLUGIN(1, 0, GreyhoundReader, Reader, s_info)

std::string GreyhoundReader::getName() const { return s_info.name; }

namespace
{
    Json::Value parse(const std::string& data)
    {
        Json::Value json;
        Json::Reader reader;

        if (data.size())
        {
            if (!reader.parse(data, json, false))
            {
                const std::string jsonError(reader.getFormattedErrorMessages());
                if (!jsonError.empty())
                {
                    throw std::runtime_error(
                            "Error during parsing: " + jsonError);
                }
            }
        }

        return json;
    }

    std::string write(const Json::Value& json)
    {
        Json::StreamWriterBuilder builder;
        builder.settings_["indentation"] = "";
        return Json::writeString(builder, json);
    }

    DimTypeList schemaToDims(const Json::Value& json)
    {
        DimTypeList output;

        // XYZ might be natively stored as integral values, typically when the
        // disk-storage for Entwine is scaled/offset.  Since we're abstracting
        // this out, always ask for XYZ as doubles.
        auto isXyz([](Dimension::Id id)
        {
            return
                id == Dimension::Id::X ||
                id == Dimension::Id::Y ||
                id == Dimension::Id::Z;
        });

        if (!json.isNull() && json.isArray())
        {
            for (const auto& jsonDim : json)
            {
                const Dimension::Id id(
                        Dimension::id(jsonDim["name"].asString()));

                const int baseType(
                        Utils::toNative(
                            Dimension::fromName(jsonDim["type"].asString())));

                const int size(jsonDim["size"].asUInt64());

                const Dimension::Type type(
                        isXyz(id) ?
                            Dimension::Type::Double :
                            static_cast<Dimension::Type>(baseType | size));

                output.emplace_back(id, type);
            }
        }

        return output;
    }

    Json::Value layoutToSchema(const PointLayout& layout)
    {
        Json::Value result;

        std::map<std::size_t, const Dimension::Detail> details;

        for (const Dimension::Id id : layout.dims())
        {
            const auto& d(*layout.dimDetail(id));
            details.emplace(d.offset(), d);
        }

        for (const auto& p : details)
        {
            const auto& d(p.second);
            Json::Value j;
            j["name"] = Dimension::name(d.id());
            j["type"] = Dimension::toName(base(d.type()));
            j["size"] = static_cast<int>(Dimension::size(d.type()));
            result.append(j);
        }

        return result;
    }

    greyhound::Bounds zoom(
            const greyhound::Bounds& queryBounds,
            const greyhound::Bounds& fullBounds,
            std::size_t& split)
    {
        auto currentBounds(fullBounds);

        greyhound::Dir dir(
                pdal::greyhound::getDirection(
                    currentBounds.mid(),
                    queryBounds.mid()));

        while (currentBounds.get(dir, true).contains(queryBounds))
        {
            currentBounds.go(dir, true);
            ++split;

            dir = pdal::greyhound::getDirection(
                    currentBounds.mid(),
                    queryBounds.mid());
        }

        return currentBounds;
    }

    std::string stringify(const greyhound::Bounds& bounds)
    {
        std::stringstream ss;
        ss << std::fixed;
        ss <<
            "[" <<
            bounds.min().x << "," << bounds.min().y << "," << bounds.min().z <<
            "," <<
            bounds.max().x << "," << bounds.max().y << "," << bounds.max().z <<
            "]";
        return ss.str();
    }

    BOX3D toBox(const greyhound::Bounds& bounds)
    {
        return BOX3D(
                bounds.min().x, bounds.min().y, bounds.min().z,
                bounds.max().x, bounds.max().y, bounds.max().z);
    }

    pdal::greyhound::Bounds toBounds(const BOX3D& box)
    {
        return greyhound::Bounds(
                box.minx, box.miny, box.minz,
                box.maxx, box.maxy, box.maxz);
    }

    const double dmin(std::numeric_limits<double>::lowest());
    const double dmax(std::numeric_limits<double>::max());
    const BOX3D everythingBox(dmin, dmin, dmin, dmax, dmax, dmax);
    const greyhound::Bounds everythingBounds(toBounds(everythingBox));
}

GreyhoundReader::GreyhoundReader()
    : Reader()
    , m_url()
    , m_resource()
    , m_depthBegin(0)
    , m_depthEnd(0)
    , m_baseDepth(0)
    , m_sparseDepth(0)
    , m_numPoints(0)
    , m_depthBeginArg(0)
    , m_depthEndArg(0)
{ }

void GreyhoundReader::initialize(PointTableRef table)
{
    if (m_url.find("http://") == std::string::npos &&
            m_url.find("https://") == std::string::npos)
    {
        m_url = "http://" + m_url;
    }

    Json::Value config;

    if (log()->getLevel() > LogLevel::Debug4)
    {
        config["arbiter"]["verbose"] = true;
    }

    m_arbiter.reset(new arbiter::Arbiter(config));

    std::string infoUrl = m_url + "/resource/" + m_resource + "/info";
    log()->get(LogLevel::Debug) << "Fetching info URL: " << infoUrl <<
        std::endl;
    m_info = parse(m_arbiter->get(infoUrl));

    m_depthBegin = m_depthBeginArg;
    m_depthEnd = m_depthEndArg;

    if (m_info.isMember("scale"))
    {
        m_scale.reset(new greyhound::Point(m_info["scale"]));
    }

    if (m_info.isMember("offset"))
    {
        m_offset.reset(new greyhound::Point(m_info["offset"]));
    }

    if (m_scale && !m_offset) m_offset.reset(new greyhound::Point(0, 0, 0));
    if (m_offset && !m_scale) m_scale.reset(new greyhound::Point(1, 1, 1));

    m_fullBounds = m_info["bounds"];

    if (m_scale)
    {
        // Unscale the full bounds.  Since the query bounds will come in as
        // native coordinates, don't modify those.
        m_fullBounds = m_fullBounds.unscale(*m_scale, *m_offset);

        // Now inverse our scale/offset.
        m_scale->x = 1.0 / m_scale->x;
        m_scale->y = 1.0 / m_scale->y;
        m_scale->z = 1.0 / m_scale->z;

        m_offset->x = -m_offset->x;
        m_offset->y = -m_offset->y;
        m_offset->z = -m_offset->z;
    }

    m_queryBounds = toBounds(m_queryBox).intersection(m_fullBounds);

    if (m_pathsArg.size())
    {
        if (m_pathsArg.size() == 1)
        {
            m_filter["Path"] = m_pathsArg.front();
        }
        else
        {
            for (const auto& p : m_pathsArg)
            {
                m_filter["Path"].append(p);
            }
        }
    }

    if (!m_filter.isNull())
    {
        log()->get(LogLevel::Debug) << "Filter: " << m_filter << std::endl;
    }

    m_dims = schemaToDims(m_info["schema"]);
    m_baseDepth = m_info["baseDepth"].asUInt64();
    m_sparseDepth = std::log(m_info["numPoints"].asUInt64()) / std::log(4) + 1;

    setSpatialReference(m_info["srs"].asString());
}

void GreyhoundReader::addArgs(ProgramArgs& args)
{
    args.add("url", "URL", m_url);
    args.add("resource", "Resource ID", m_resource);
    args.add("bounds", "Bounding cube", m_queryBox, everythingBox);
    args.add("depth_begin", "Beginning depth to query", m_depthBeginArg, 0u);
    args.add("depth_end", "Ending depth to query", m_depthEndArg, 0u);
    args.add("tile_path", "Index-optimized tile selection", m_pathsArg);
    args.add("filter", "Query filter", m_filter);
    args.add("threads", "Number of threads for HTTP requests", m_threadsArg, 4);
}

void GreyhoundReader::prepared(PointTableRef table)
{
    auto& layout(*table.layout());
    // Note that we must construct the schema (which drives the formatting of
    // Greyhound's responses) here, rather than just from the 'info' that comes
    // back from Greyhound.  This is because other Reader instances may add
    // dimensions that don't exist in this resource.  Also, the dimensions may
    // be re-ordered in the layout from what we obtained in 'info'.  Greyhound
    // will zero-fill non-existent dimentions in the responses, which will
    // compress to pretty much nothing.
    m_schema = layoutToSchema(layout);
    log()->get(LogLevel::Debug) << "Schema: " << m_schema << std::endl;

    m_dims.clear();
    for (const auto& j : m_schema)
    {
        m_dims.push_back(layout.findDimType(j["name"].asString()));

        if (m_dims.back().m_id == Dimension::Id::Unknown)
        {
            throw std::runtime_error(
                    "Could not find dimension " + j["name"].asString());
        }
    }
}

void GreyhoundReader::addDimensions(PointLayoutPtr layout)
{
    for (auto& dim : m_dims)
    {
        layout->registerDim(dim.m_id, dim.m_type);
    }
}

QuickInfo GreyhoundReader::inspect()
{
    QuickInfo qi;
    std::unique_ptr<PointLayout> layout(new PointLayout());

    PointTable table;
    initialize(table);
    addDimensions(layout.get());

    for (auto di = layout->dims().begin(); di != layout->dims().end(); ++di)
    {
        qi.m_dimNames.push_back(layout->dimName(*di));
    }

    qi.m_srs = getSpatialReference();
    qi.m_valid = true;

    std::size_t split(0);
    const greyhound::Bounds zoomBounds(
            zoom(m_queryBounds, m_fullBounds, split));

    if (split)
    {
        const std::size_t depthBegin(
                std::max(m_depthBegin, m_baseDepth + split));
        const std::size_t depthEnd(depthBegin + 32);

        const auto hierarchy(
                fetchVerticalHierarchy(zoomBounds, depthBegin, depthEnd));
        qi.m_pointCount =
            std::accumulate(hierarchy.begin(), hierarchy.end(), 0);

        // The hierarchy doesn't have the resolution we want at the upper
        // levels.  Estimate what's there based on the top level of information
        // that we have.
        if (hierarchy.size())
        {
            for (std::size_t i(0); i < split; ++i)
            {
                qi.m_pointCount += hierarchy.front() / ((i + 1) * 8);
            }
        }
    }
    else
    {
        qi.m_pointCount = m_info["numPoints"].asUInt64();
    }

    qi.m_bounds = toBox(m_fullBounds.intersection(toBounds(m_queryBox)));
    done(table);

    return qi;
}

point_count_t GreyhoundReader::read(PointViewPtr view, point_count_t count)
{
    std::size_t split(0);
    const greyhound::Bounds zoomBounds(
            zoom(m_queryBounds, m_fullBounds, split));

    // Greyhound's native chunking is pretty small to accomodate a renderer's
    // need for very fast response times.  Since we don't really care about
    // that, we'll let Greyhound do more work per query for better overall
    // throughput.
    split += 3;

    const std::size_t depthBegin(m_depthBegin);
    const std::size_t depthSplit(std::max(m_depthBegin, m_baseDepth + split));

    greyhound::Pool pool(m_threadsArg);

    if (depthSplit > depthBegin)
    {
        pool.add([this, &view, depthBegin, depthSplit]()
        {
            try
            {
                inc(fetchData(*view, m_queryBounds, depthBegin, depthSplit));
            }
            catch (std::exception& e)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_error.reset(new std::string(e.what()));
            }
            catch (...)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_error.reset(new std::string("Unknown error during read"));
            }
        });
    }

    launchPooledReads(*view, zoomBounds, depthSplit, pool);

    pool.await();
    if (m_error) throw pdal_error(*m_error);

    return m_numPoints;
}

void GreyhoundReader::launchPooledReads(
        PointView& view,
        const greyhound::Bounds& bounds,
        const std::size_t startDepth,
        greyhound::Pool& pool)
{
    Json::Value hierarchy(
            fetchHierarchy(bounds, startDepth, startDepth + m_hierarchyStep));

    m_tasks.emplace([this, &view, &hierarchy, bounds, startDepth]()
    {
        read(view, hierarchy, bounds, startDepth, startDepth);
    });

    while (m_running.size() || m_tasks.size())
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        if (m_tasks.size())
        {
            const std::size_t taskId(m_taskId);
            ++m_taskId;

            m_running[taskId] = std::move(m_tasks.front());
            auto& task(m_running[taskId]);
            m_tasks.pop();

            lock.unlock();

            pool.add([this, taskId, &task]()
            {
                try
                {
                    task();
                }
                catch (std::runtime_error& e)
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_error.reset(new std::string(
                                std::string("Greyhound read failed: ") +
                                e.what()));
                }
                catch (...)
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_error.reset(new std::string(
                                "Greyhound read failed: unknown error"));
                }

                std::lock_guard<std::mutex> lock(m_mutex);
                m_running.erase(taskId);
            });
        }
        else
        {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // If any tasks failed, rethrow in the main thread.
        lock.lock();
        if (m_error) throw pdal_error(*m_error);
    }
}

void GreyhoundReader::read(
        PointView& view,
        Json::Value& hierarchy,
        const greyhound::Bounds& bounds,
        const std::size_t startDepth,
        const std::size_t depth)
{
    // At the end of the query.
    if (m_depthEnd && depth >= m_depthEnd) return;
    if (!bounds.overlaps(m_queryBounds)) return;

    const greyhound::Bounds intersect(bounds.intersection(m_queryBounds));

    if (depth > m_sparseDepth)
    {
        // We're at the sparse depth, so request all remaining depths for the
        // current bounds.
        std::lock_guard<std::mutex> lock(m_mutex);
        m_tasks.emplace([this, &view, intersect, depth]()
        {
            inc(fetchData(view, intersect, depth, m_depthEnd));
        });
    }
    else
    {
        // If we're at the step size, fetch the next hierarchy range.
        if (hierarchy.isNull())
        {
            const auto diff(depth - startDepth);
            if (diff && diff % m_hierarchyStep == 0)
            {
                hierarchy = fetchHierarchy(
                        bounds, depth, depth + m_hierarchyStep);
            }
            else return;
        }

        if (hierarchy.isNull() || !hierarchy["n"].asUInt64()) return;

        const std::size_t nextDepth(depth + 1);

        auto next([this, &view, startDepth, nextDepth, bounds, &hierarchy]()
        {
            for (std::size_t d(0); d < greyhound::dirEnd(); ++d)
            {
                const auto dir(greyhound::toDir(d));
                const greyhound::Bounds nextBounds(bounds.get(dir));
                auto& nextHierarchy(hierarchy[greyhound::dirToString(dir)]);

                read(view, nextHierarchy, nextBounds, startDepth, nextDepth);
            }
        });

        // For the first level, kick off the next level without waiting for
        // the response.
        if (depth == startDepth) next();

        std::lock_guard<std::mutex> lock(m_mutex);
        m_tasks.emplace(
                [this, &view, intersect, depth, nextDepth, startDepth, next]()
        {
            const auto inc(fetchData(view, intersect, depth, nextDepth));
            if (inc && depth != startDepth) next();
        });
    }
}

std::vector<point_count_t> GreyhoundReader::fetchVerticalHierarchy(
        const greyhound::Bounds& bounds,
        std::size_t depthBegin,
        std::size_t depthEnd) const
{
    std::stringstream url;
    url << m_url << "/resource/" << m_resource;
    url << "/hierarchy?bounds=" << arbiter::http::sanitize(stringify(bounds));
    url << "&depthBegin=" << depthBegin;
    url << "&depthEnd=" << depthEnd;
    url << "&vertical=true";

    if (m_scale) url << "&scale=" << write(m_scale->toJson());
    if (m_offset) url << "&offset=" << write(m_offset->toJson());

    log()->get(LogLevel::Debug) << "Hierarchy: " << url.str() << std::endl;
    const Json::Value json(parse(m_arbiter->get(url.str())));

    std::vector<point_count_t> results;
    for (const auto& v : json) results.push_back(v.asUInt64());

    return results;
}

Json::Value GreyhoundReader::fetchHierarchy(
        const greyhound::Bounds& bounds,
        std::size_t depthBegin,
        std::size_t depthEnd) const
{
    std::stringstream url;
    url << m_url << "/resource/" << m_resource;
    url << "/hierarchy?bounds=" << arbiter::http::sanitize(stringify(bounds));
    url << "&depthBegin=" << depthBegin;
    url << "&depthEnd=" << depthEnd;

    if (m_scale) url << "&scale=" << write(m_scale->toJson());
    if (m_offset) url << "&offset=" << write(m_offset->toJson());

    log()->get(LogLevel::Debug) << "Hierarchy: " << url.str() << std::endl;
    return parse(m_arbiter->get(url.str()));
}

point_count_t GreyhoundReader::fetchData(
        PointView& view,
        const greyhound::Bounds& bounds,
        const std::size_t depthBegin,
        const std::size_t depthEnd)
{
    std::stringstream url;
    url << m_url << "/resource/" << m_resource;
    url << "/read?bounds=" << arbiter::http::sanitize(stringify(bounds));
    url << "&depthBegin=" << depthBegin;
    url << "&depthEnd=" << depthEnd;
    if (m_scale) url << "&scale=" << write(m_scale->toJson());
    if (m_offset) url << "&offset=" << write(m_offset->toJson());

#ifdef PDAL_HAVE_LAZPERF
    url << "&compress=true";
#endif
    if (!m_filter.isNull())
    {
        url << "&filter=" << arbiter::http::sanitize(write(m_filter));
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        log()->get(LogLevel::Debug) << "Reading: " << url.str() << std::endl;
    }

    url << "&schema=" << arbiter::http::sanitize(write(m_schema));

    auto response(m_arbiter->getBinary(url.str()));
    const std::size_t pointSize(view.layout()->pointSize());

    std::unique_lock<std::mutex> lock(m_mutex);

    const std::size_t numPoints(
            *reinterpret_cast<const uint32_t*>(
                response.data() + response.size() - sizeof(uint32_t)));

    log()->get(LogLevel::Debug) <<
        "Fetched " << numPoints << " points" << std::endl;
    log()->get(LogLevel::Debug) <<
        "Fetched " << response.size() << " bytes" << std::endl;

    response.resize(response.size() - sizeof(uint32_t));

    std::vector<char*> points;
    points.reserve(numPoints);

    const PointId startId(view.size());
    PointId id(startId);

    for (std::size_t i(0); i < numPoints; ++i)
    {
        points.push_back(view.getOrAddPoint(id));
        ++id;
    }

    lock.unlock();

    // Because we are requesting the data in the exact PointLayout of our
    // PointView, we can just decompress directly into that memory.  Any
    // non-existent dimensions (for example those added by other readers in the
    // pipeline) will be zero-filled.

#ifdef PDAL_HAVE_LAZPERF
    SignedLazPerfBuf buffer(response);
    LazPerfDecompressor<SignedLazPerfBuf> decompressor(buffer, m_dims);

    for (std::size_t i(0); i < numPoints; ++i)
    {
        decompressor.decompress(points[i], pointSize);
    }
#else
    const char* pos(response.data());
    const char* end(pos + numPoints * pointSize);
    std::size_t i(0);

    while (pos < end)
    {
        std::copy(pos, pos + pointSize, points[i]);
        ++i;
        pos += pointSize;
    }
#endif

    if (m_cb)
    {
        lock.lock();
        for (std::size_t i(0); i < numPoints; ++i)
        {
            m_cb(view, startId + i);
        }
    }

    return numPoints;
}

} // namespace pdal

