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

#include "GreyhoundWriter.hpp"

#include <pdal/pdal_macros.hpp>

#include <arbiter/arbiter.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.greyhound",
    "Greyhound Writer",
    "http://pdal.io/stages/writers.greyhound.html");

CREATE_SHARED_PLUGIN(1, 0, GreyhoundWriter, Writer, s_info)

std::string GreyhoundWriter::getName() const { return s_info.name; }

namespace
{

bool existsIn(const Json::Value& schema, const std::string name)
{
    for (const auto& d : schema)
    {
        if (d["name"].asString() == name)
            return true;
    }
    return false;
}

} // unnamed namespace

void GreyhoundWriter::addArgs(ProgramArgs& args)
{
    args.add("name", "Name for the addon dimension set", m_name);
}

void GreyhoundWriter::initialize(PointTableRef table)
{ }

void GreyhoundWriter::prepared(PointTableRef table)
{
    auto g(table.privateMetadata("greyhound"));

    m_info = parse(g.findChild("info").value<std::string>());
    m_params = GreyhoundParams(
            g.findChild("root").value<std::string>(),
            parse(g.findChild("params").value<std::string>()));

    // TODO Add `dims` parameter for explicit writing.
    Json::Value nativeSchema(m_info["schema"]);

    // Create our writeLayout, which is the dimensions that do not exist in
    // the original schema.
    auto& layout(*table.layout());
    for (const Dimension::Id id : layout.dims())
    {
        const Dimension::Detail& d(*layout.dimDetail(id));
        const std::string name(layout.dimName(id));

        if (!existsIn(nativeSchema, name))
        {
            m_writeLayout.registerOrAssignDim(name, d.type());
        }
    }

    m_writeLayout.finalize();

    // Create our JSON schema to send along with the /write call matching the
    // formatting of what we'll be writing.
    Json::Value writeSchema;
    for (const Dimension::Id id : m_writeLayout.dims())
    {
        const Dimension::Detail& d(*m_writeLayout.dimDetail(id));
        const std::string name(layout.dimName(id));

        Json::Value dim;
        dim["name"] = name;
        dim["type"] = Dimension::toName(base(d.type()));
        dim["size"] = static_cast<int>(Dimension::size(d.type()));
        writeSchema.append(dim);
    }

    m_params["schema"] = writeSchema;
    m_params["name"] = m_name;
}

void GreyhoundWriter::write(const PointViewPtr view)
{
    /*
    std::cout << "Writing" << std::endl;
    std::cout << m_params.root() << std::endl;
    std::cout << m_params.qs() << std::endl;
    */

    const std::size_t pointSize(m_writeLayout.pointSize());
    std::vector<char> data(view->size() * pointSize, 0);
    char* pos(data.data());

    for (std::size_t i(0); i < view->size(); ++i)
    {
        view->getPackedPoint(m_writeLayout.dimTypes(), i, pos);
        pos += pointSize;
    }

    arbiter::Arbiter a;
    const std::string url(m_params.root() + "write" + m_params.qs());
    log()->get(LogLevel::Debug) << "Writing: " << url << std::endl;
    a.put(url, data);
}

} // namespace pdal

