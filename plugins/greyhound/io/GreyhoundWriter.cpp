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

void GreyhoundWriter::addArgs(ProgramArgs& args)
{
    args.add("name", "Name for the addon dimension set", m_name);
    args.add("dims", "Dimension names to write", m_writeDims);
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

    // Don't include the addons here, since addons may be written more than
    // once.
    std::map<std::string, const Json::Value*> remote;
    for (const auto& d : m_info["schema"])
        remote[d["name"].asString()] = &d;

    auto& layout(*table.layout());

    // If no dimensions-to-write are explicitly passed in the options, use the
    // diff between the remote dimensions and the table dimensions.
    if (m_writeDims.isNull())
    {
        for (const Dimension::Id id : layout.dims())
        {
            const std::string name(layout.dimName(id));
            if (!remote.count(name))
            {
                m_writeDims.append(name);
            }
        }
    }

    for (const Json::Value& j : m_writeDims)
    {
        const auto name(j.asString());
        m_writeLayout.registerOrAssignDim(
                name,
                layout.dimType(layout.findDim(name)));
    }

    m_params["schema"] = layoutToSchema(m_writeLayout);
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

