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

#include "GreyhoundReader.hpp"

#include <pdal/pdal_macros.hpp>
#include <pdal/compression/LazPerfCompression.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.greyhound",
    "Greyhound Reader",
    "http://pdal.io/stages/readers.greyhound.html");

CREATE_SHARED_PLUGIN(1, 0, GreyhoundReader, Reader, s_info)

std::string GreyhoundReader::getName() const { return s_info.name; }

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

void GreyhoundReader::initialize(PointTableRef table)
{
    Json::Value config;
    if (log()->getLevel() > LogLevel::Debug4)
        config["arbiter"]["verbose"] = true;
    m_arbiter.reset(new arbiter::Arbiter(config));

    // If this stage was parsed from a string parameter rather than JSON object
    // specification, normalize it to our URL.
    if (m_filename.size() && m_args.url.empty())
    {
        m_args.url = m_filename;
        const std::string pre("greyhound://");
        if (m_args.url.find(pre) == 0)
            m_args.url = m_args.url.substr(pre.size());
    }

    m_params = GreyhoundParams(m_args);

    log()->get(LogLevel::Debug) << "Fetching info from " << m_params.root() <<
        std::endl;

    try
    {
        m_info = parse(m_arbiter->get(m_params.root() + "info"));
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

    // TODO We should probably take a `dims` parameter rather than a full
    // schema, similar to the GreyhoundWriter.
    if (m_params["schema"].isNull())
        m_params["schema"] = m_info["schema"];

    for (const auto& d : m_params["schema"])
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
        m_nativeLayout.registerOrAssignDim(name, type);
    }
}

void GreyhoundReader::prepared(PointTableRef table)
{
    // Build the Schema for our request from the native schema from `info`.
    auto& layout(m_nativeLayout);
    m_args.schema = Json::Value();

    for (const Dimension::Id id : layout.dims())
    {
        const Dimension::Detail& d(*layout.dimDetail(id));
        const std::string name(layout.dimName(id));

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

    MetadataNode queryNode(table.privateMetadata("greyhound"));
    queryNode.add("info", dense(m_info));
    queryNode.add("root", m_params.root());
    queryNode.add("params", dense(m_params.toJson()));
}

point_count_t GreyhoundReader::read(PointViewPtr view, point_count_t count)
{
    const std::string url(m_params.root() + "read" + m_params.qs());
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

#ifdef PDAL_HAVE_LAZPERF
    auto cb = [this, &view, numPoints](char *buf, size_t bufsize)
    {
        view->setPackedPoint(m_dims, view->size(), buf);
        if (m_cb)
            m_cb(*view, view->size() - 1);
    };
    LazPerfDecompressor(cb, m_dims, numPoints).
        decompress(response.data(), response.size());
#else
    const char* end(response.data() + response.size());
    for (const char* pos(response.data()); pos < end; pos += pointSize)
    {
        view->setPackedPoint(m_dims, view->size(); pos);
        if (m_cb)
            m_cb(*view, view->size() - 1);
    }
#endif

    return numPoints;
}

} // namespace pdal

