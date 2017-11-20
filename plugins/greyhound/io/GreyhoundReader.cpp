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

#include <regex>
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
                throw pdal_error("Error during parsing: " + jsonError);
            }
        }
    }

    return json;
}

std::string dense(const Json::Value& json)
{
    Json::StreamWriterBuilder builder;
    builder.settings_["indentation"] = "";
    return Json::writeString(builder, json);
}

} // unnamed namespace

void GreyhoundReader::addArgs(ProgramArgs& args)
{
    args.add("url", "URL", m_args.url);
    args.add("resource", "Resource name", m_args.resource);
    args.add("bounds", "Bounds", m_args.sbounds);
    args.add("depth_begin", "Beginning depth to query", m_args.depthBegin);
    args.add("depth_end", "Ending depth to query", m_args.depthEnd);
    args.add("tile_path", "Index-optimized tile selection", m_args.tilePath);
    args.add("filter", "Query filter", m_args.filter);
    args.add("schema", "Greyhound schema", m_args.schema);
}

std::string GreyhoundReader::Args::base() const
{
    // The url may be of the form:
    //      server.com
    //      server.com/resource/asdf/read?<query>
    //
    // We want to extract, from either form (resource may be specified
    // separately):
    //      server.com/resource/asdf/

    // Strip off query string.
    std::string s(url.substr(0, url.find('?')));

    if (std::regex_match(s, std::regex(".+/resource/.+/read$")))
    {
        // If the URL is a fully-specified READ query, strip off the "read".
        if (resource.size())
            throw pdal_error("Cannot specify resource twice");

        s = s.substr(0, s.size() - 4);
    }

    if (s.empty())
        throw pdal_error("No server specified");

    if (resource.size())
    {
        // Otherwise, if resource is specified separately, add it now.
        s = s + (s.back() == '/' ? "" : "/") + "resource/" + resource;
    }

    // Make sure we end up with a trailing slash.
    if (s.back() != '/')
        s = s + '/';

    // Finally, ensure an HTTP prefix so arbiter interprets it correctly.
    if (s.find("http://") == 0 || s.find("https://") == 0)
        return s;
    else return "http://" + s;
}

std::string GreyhoundReader::Args::query() const
{
    const std::size_t q(url.find('?'));
    std::string s(url.substr(q != std::string::npos ? q : url.size()));

    auto add([&s](std::string k, std::string v)
    {
        s += (s.size() ? '&' : '?') + k + '=' + v;
    });

    if (sbounds.size())
    {
        greyhound::Bounds gbounds;
        if (sbounds.find('(') != std::string::npos)
        {
            // This is a PDAL-specified bounds.
            Bounds pdalBounds;
            std::istringstream iss(sbounds);
            iss >> pdalBounds;
            if (pdalBounds.is3d())
            {
                const auto box(pdalBounds.to3d());
                gbounds = greyhound::Bounds(box.minx, box.miny, box.minz,
                        box.maxx, box.maxy, box.maxz);
            }
            else
            {
                const auto box(pdalBounds.to2d());
                gbounds = greyhound::Bounds(box.minx, box.miny,
                        box.maxx, box.maxy);
            }
        }
        else
        {
            gbounds = greyhound::Bounds(parse(sbounds));
        }

        add("bounds", dense(gbounds.toJson()));
    }

    if (depthBegin)
        add("depthBegin", std::to_string(depthBegin));
    if (depthEnd)
        add("depthEnd", std::to_string(depthEnd));

    Json::Value f(filter);

    if (f.isString())
        f = parse(f.asString());

    if (tilePath.size())
        f["Path"] = tilePath;

    if (!f.isNull())
        add("filter", dense(f));

    if (!schema.isNull())
        add("schema", dense(schema));

#ifdef PDAL_HAVE_LAZPERF
    add("compress", "true");
#endif

    return s;
}

void GreyhoundReader::initialize(PointTableRef table)
{
    Json::Value config;
    if (log()->getLevel() > LogLevel::Debug4)
        config["arbiter"]["verbose"] = true;
    m_arbiter.reset(new arbiter::Arbiter(config));

    if (m_filename.size() && m_args.url.empty())
    {
        m_args.url = m_filename;
        const std::string pre("greyhound://");
        if (m_args.url.find(pre) == 0)
            m_args.url = m_args.url.substr(pre.size());
    }

    log()->get(LogLevel::Debug) << "Fetching info from " << m_args.base() <<
        std::endl;

    try
    {
        m_info = parse(m_arbiter->get(m_args.base() + "info"));
    }
    catch (std::exception& e)
    {
        throw pdal_error(std::string("Failed to fetch info: ") + e.what());
    }

    if (m_info.isMember("srs"))
        setSpatialReference(m_info["srs"].asString());
}

void GreyhoundReader::addDimensions(PointLayoutPtr layout)
{
    auto isXyz([](const std::string& name)
    {
        return name == "X" || name == "Y" || name == "Z";
    });

    for (const auto& d : m_info["schema"])
    {
        const auto name(d["name"].asString());

        const int baseType(
                Utils::toNative(Dimension::fromName(d["type"].asString())));

        const int size(d["size"].asInt());

        const Dimension::Type type(
                isXyz(name) ?
                    Dimension::Type::Double :
                    static_cast<Dimension::Type>(baseType | size));

        layout->registerOrAssignDim(name, type);
    }
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

    std::map<std::size_t, const Dimension::Detail> details;

    for (const Dimension::Id id : layout.dims())
    {
        const auto& detail(*layout.dimDetail(id));
        details.emplace(detail.offset(), detail);
    }

    m_args.schema = Json::Value();
    for (const auto& p : details)
    {
        const Dimension::Detail& d(p.second);
        const std::string name(layout.dimName(d.id()));

        Json::Value j;
        j["name"] = name;
        j["type"] = Dimension::toName(base(d.type()));
        j["size"] = static_cast<int>(Dimension::size(d.type()));
        m_args.schema.append(j);

        m_dims.push_back(layout.findDimType(name));
        if (m_dims.back().m_id == Dimension::Id::Unknown)
            throw pdal_error("Could not find dimension " + name);
    }

    log()->get(LogLevel::Debug) << "Schema: " << m_args.schema << std::endl;
}

point_count_t GreyhoundReader::read(PointViewPtr view, point_count_t count)
{
    const std::string url(m_args.base() + "read" + m_args.query());
    log()->get(LogLevel::Debug) << "Reading: " << url << std::endl;

    auto response(m_arbiter->getBinary(url));
    const std::size_t pointSize(view->layout()->pointSize());

    uint32_t numPoints(0);
    std::copy(
            response.data() + response.size() - sizeof(uint32_t),
            response.data() + response.size(),
            reinterpret_cast<char*>(&numPoints));

    log()->get(LogLevel::Debug) <<
        "Fetched " << numPoints << " points" << std::endl;
    log()->get(LogLevel::Debug) <<
        "Fetched " << response.size() << " bytes" << std::endl;

    response.resize(response.size() - sizeof(uint32_t));

    std::vector<char*> points;
    points.reserve(numPoints);

    const PointId startId(view->size());
    PointId id(startId);
    size_t pointNum(0);

    for (std::size_t i(0); i < numPoints; ++i)
    {
        points.push_back(view->getOrAddPoint(id));
        ++id;
    }

    // Because we are requesting the data in the exact PointLayout of our
    // PointView, we can just decompress directly into that memory.  Any
    // non-existent dimensions (for example those added by other readers in the
    // pipeline) will be zero-filled.

#ifdef PDAL_HAVE_LAZPERF

    /**
    SignedLazPerfBuf buffer(response);
    LazPerfDecompressor<SignedLazPerfBuf> decompressor(buffer, m_dims);
    **/

    auto cb = [&points, &pointNum](char *buf, size_t bufsize)
    {
        std::copy(buf, buf + bufsize, points[pointNum++]);
    };
    LazPerfDecompressor(cb, m_dims, numPoints).
        decompress(response.data(), response.size());
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
        for (std::size_t i(0); i < numPoints; ++i)
        {
            m_cb(*view, startId + i);
        }
    }

    return numPoints;
}

} // namespace pdal

