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
#include "LasReader.hpp"

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

    Json::Value parse(const std::string& data)
    {
        Json::Value json;
        Json::Reader reader;

        if (!reader.parse(data, json, false))
        {
            const std::string jsonError(reader.getFormattedErrorMessages());
            if (!jsonError.empty())
            {
                throw pdal_error("Error during parsing: " + jsonError);
            }
        }

        return json;
    }
}

CREATE_STATIC_STAGE(EptReader, s_info);

std::string EptReader::getName() const { return s_info.name; }

void EptReader::addArgs(ProgramArgs& args)
{
    args.add("bounds", "Bounds to fetch", m_args.boundsArg());
    args.add("origin", "Origin of source file to fetch", m_args.originArg());
    args.add("threads", "Number of worker threads", m_args.threadsArg());
}

void EptReader::initialize()
{
    m_root = m_filename;

    const std::string prefix("ept://");
    if (m_root.substr(0, prefix.size()) == prefix)
    {
        m_root = m_root.substr(prefix.size());
    }

    m_arbiter.reset(new arbiter::Arbiter());
    m_ep.reset(new arbiter::Endpoint(m_arbiter->getEndpoint(m_root)));

    log()->get(LogLevel::Debug) << "Endpoint: " << m_ep->prefixedRoot() <<
        std::endl;

    m_info = parse(m_ep->get("entwine.json"));
    m_bounds = toBox3d(m_info["bounds"]);
    m_hierarchyStep = m_info["hierarchyStep"].asUInt64();

    log()->get(LogLevel::Debug) << "Got info" << std::endl;

    try
    {
        setSpatialReference(m_info["srs"].asString());
    }
    catch (...)
    {
        log()->get(LogLevel::Error) << "Could not create an SRS" << std::endl;
    }

    m_queryBounds = m_args.bounds();
    if (m_queryBounds.empty())
    {
        double mn((std::numeric_limits<double>::lowest)());
        double mx((std::numeric_limits<double>::max)());
        m_queryBounds = BOX3D(mn, mn, mn, mx, mx, mx);
    }

    handleOriginQuery();

    log()->get(LogLevel::Debug) << "Threads: " << m_args.threads() << std::endl;
    log()->get(LogLevel::Debug) << "Query bounds: " << m_queryBounds <<
        std::endl;
}

void EptReader::handleOriginQuery()
{
    if (m_args.origin().empty()) return;

    const std::string search(m_args.origin());
    log()->get(LogLevel::Debug) << "Searching files for " << search <<
        std::endl;

    const Json::Value files(parse(m_ep->get("entwine-files.json")));
    log()->get(LogLevel::Debug) << "Fetched file manifest" << std::endl;

    if (!files.isArray())
    {
        throw pdal_error("Unexpected file manifest: " +
                files.toStyledString());
    }

    if (search.find_first_not_of("0123456789") == std::string::npos)
    {
        m_queryOriginId.reset(new uint64_t(std::stoull(search)));
    }
    else
    {
        for (Json::ArrayIndex i(0); i < files.size(); ++i)
        {
            const Json::Value& entry(files[i]);
            if (entry["path"].asString().find(search) != std::string::npos)
            {
                if (m_queryOriginId)
                {
                    throw pdal_error("Origin search path is not unique");
                }
                m_queryOriginId.reset(new uint64_t(i));
            }
        }
    }

    if (!m_queryOriginId)
    {
        throw pdal_error("Failed lookup of origin: " + search);
    }

    if (*m_queryOriginId >= files.size())
    {
        throw pdal_error("Invalid origin ID");
    }

    const Json::Value& found(files[static_cast<Json::ArrayIndex>(
                *m_queryOriginId)]);
    BOX3D q(toBox3d(found["bounds"]));

    if (m_info.isMember("scale"))
    {
        const Json::Value& s(m_info["scale"]);
        std::array<double, 3> scale;
        if (s.isArray())
        {
            scale[0] = s[0].asDouble();
            scale[1] = s[1].asDouble();
            scale[2] = s[2].asDouble();
        }
        else
        {
            scale[0] = scale[1] = scale[2] = s.asDouble();
        }

        q.minx -= scale[0];
        q.miny -= scale[1];
        q.minz -= scale[2];
        q.maxx += scale[0];
        q.maxy += scale[1];
        q.maxz += scale[2];
    }

    m_queryBounds.clip(q);

    log()->get(LogLevel::Debug) << "Query origin " <<
        *m_queryOriginId << ": " <<
        found.toStyledString() <<
        std::endl;
}

QuickInfo EptReader::inspect()
{
    QuickInfo qi;

    initialize();

    const Json::Value& bounds(m_info["boundsConforming"]);
    qi.m_bounds.minx = bounds[0].asDouble();
    qi.m_bounds.miny = bounds[1].asDouble();
    qi.m_bounds.minz = bounds[2].asDouble();
    qi.m_bounds.maxx = bounds[3].asDouble();
    qi.m_bounds.maxy = bounds[4].asDouble();
    qi.m_bounds.maxz = bounds[5].asDouble();

    qi.m_srs = m_info["srs"].asString();
    qi.m_pointCount = m_info["numPoints"].asUInt64();

    const Json::Value& schema(m_info["schema"]);
    for (Json::ArrayIndex i(0); i < schema.size(); ++i)
    {
        qi.m_dimNames.push_back(schema[i]["name"].asString());
    }

    qi.m_valid = true;

    return qi;
}

void EptReader::addDimensions(PointLayoutPtr layout)
{
    const Json::Value& schema(m_info["schema"]);
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);
    for (Json::ArrayIndex i(0); i < schema.size(); ++i)
    {
        const Json::Value& dim(schema[i]);
        const std::string name(dim["name"].asString());
        log()->get(LogLevel::Debug) << "Registering dim " << name << std::endl;
        const Dimension::Type type(Dimension::type(dim["type"].asString()));
        layout->registerOrAssignDim(name, type);
    }
}

void EptReader::ready(PointTableRef)
{
    const Key key(m_bounds);
    const Json::Value hier(parse(m_ep->get("h/" + key.toString() + ".json")));

    Pool pool(m_args.threads());
    overlaps(pool, hier, key);
    pool.await();

    log()->get(LogLevel::Debug) << "Overlap nodes: " << m_overlapKeys.size() <<
        std::endl;
    log()->get(LogLevel::Debug) << "Overlap points: " << m_overlapPoints <<
        std::endl;
}

void EptReader::overlaps(Pool& pool, const Json::Value& hier, const Key& key)
{
    if (!key.b.overlaps(m_queryBounds)) return;
    const uint64_t np(hier[key.toString()].asUInt64());
    if (!np) return;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_overlapKeys.insert(key);
        m_overlapPoints += np;
    }

    if (m_hierarchyStep != 0 && key.d % m_hierarchyStep == 0)
    {
        log()->get(LogLevel::Debug) << "Hierarchy: " <<
            key.toString() << std::endl;

        pool.add([this, &pool, key]()
        {
            const auto next(parse(m_ep->get("h/" + key.toString() + ".json")));
            for (uint64_t dir(0); dir < 8; ++dir)
            {
                overlaps(pool, next, key.bisect(dir));
            }
        });
    }
    else
    {
        for (uint64_t dir(0); dir < 8; ++dir)
        {
            overlaps(pool, hier, key.bisect(dir));
        }
    }
}

PointViewSet EptReader::run(PointViewPtr base)
{
    PointViewSet views;

    Pool pool(m_args.threads());

    uint64_t i(0);
    for (const Key& key : m_overlapKeys)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        auto view(base->makeNew());
        views.insert(view);
        lock.unlock();

        log()->get(LogLevel::Debug) << "Data " << ++i << "/" <<
            m_overlapKeys.size() << ": " << key.toString() << std::endl;

        pool.add([this, view, &key]()
        {
            readOne(view, key);
        });
    }

    pool.await();

    log()->get(LogLevel::Debug) << "Done reading!" << std::endl;

    return views;
}

void EptReader::readOne(PointViewPtr dst, const Key& key) const
{
    auto handle(m_ep->getLocalHandle(key.toString() + ".laz"));

    PointTable table;

    std::lock_guard<std::mutex> lock(m_mutex);

    Options options;
    options.add("filename", handle->localPath());
    options.add("use_eb_vlr", true);

    LasReader reader;
    reader.setOptions(options);
    reader.prepare(table);

    double x, y, z;
    uint64_t o;

    for (auto& src : reader.execute(table))
    {
        for (uint64_t i(0); i < src->size(); ++i)
        {
            x = src->getFieldAs<double>(Dimension::Id::X, i);
            y = src->getFieldAs<double>(Dimension::Id::Y, i);
            z = src->getFieldAs<double>(Dimension::Id::Z, i);
            o = src->getFieldAs<uint64_t>(Dimension::Id::OriginId, i);
            const bool withinOrigin = !m_queryOriginId || o == *m_queryOriginId;

            if (withinOrigin && m_queryBounds.contains(x, y, z))
            {
                char* pos(dst->getOrAddPoint(dst->size()));

                for (const DimType& dim : dst->dimTypes())
                {
                    src->getField(pos + dst->layout()->dimOffset(dim.m_id),
                            dim.m_id, dim.m_type, i);
                }
            }
        }
    }
}

} // namespace pdal

