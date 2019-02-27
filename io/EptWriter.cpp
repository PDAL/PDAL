/******************************************************************************
* Copyright (c) 2019, Connor Manning (connor@hobu.co)
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

#include "EptWriter.hpp"

#include <cassert>

#include <json/json.h>

#include <arbiter/arbiter.hpp>

#include "private/EptSupport.hpp"

namespace pdal
{

namespace
{
    const StaticPluginInfo s_info
    {
        "writers.ept",
        "EPT Writer",
        "http://pdal.io/stages/writers.ept.html",
        { "ept" }
    };

    std::string getTypeString(Dimension::Type t)
    {
        const auto base(Dimension::base(t));
        if (base == Dimension::BaseType::Signed) return "signed";
        if (base == Dimension::BaseType::Unsigned) return "unsigned";
        if (base == Dimension::BaseType::Floating) return "float";
        throw std::runtime_error("Invalid dimension type");
    }
}

CREATE_STATIC_STAGE(EptWriter, s_info)

EptWriter::EptWriter()
    : m_addonsArg(new Json::Value())
{ }

EptWriter::~EptWriter()
{ }

std::string EptWriter::getName() const { return s_info.name; }

void EptWriter::addArgs(ProgramArgs& args)
{
    args.add("addons", "Mapping of output locations to their dimension names",
            *m_addonsArg);
    args.add("threads", "Number of worker threads", m_numThreads);
}

void EptWriter::addDimensions(PointLayoutPtr layout)
{
    m_nodeIdDim = layout->registerOrAssignDim("EptNodeId",
            Dimension::Type::Unsigned32);
    m_pointIdDim = layout->registerOrAssignDim("EptPointId",
            Dimension::Type::Unsigned32);
}

void EptWriter::ready(PointTableRef table)
{
    m_arbiter.reset(new arbiter::Arbiter());
    m_pool.reset(new Pool(std::max<uint64_t>(m_numThreads, 4)));

    MetadataNode meta(table.privateMetadata("ept"));
    const auto info(parse(meta.findChild("info").value<std::string>()));
    const auto keys(parse(meta.findChild("keys").value<std::string>()));
    m_hierarchyStep = meta.findChild("step").value<uint64_t>();
    m_info.reset(new EptInfo(info));

    for (const std::string s : keys.getMemberNames())
    {
        m_hierarchy[Key(s)] = keys[s].asUInt64();
    }

    const PointLayout& layout(*table.layout());
    for (const std::string path : m_addonsArg->getMemberNames())
    {
        const auto endpoint(
                m_arbiter->getEndpoint(arbiter::fs::expandTilde(path)));
        const std::string dimName((*m_addonsArg)[path].asString());
        const Dimension::Id id(layout.findDim(dimName));
        m_addons.emplace_back(new Addon(layout, endpoint, id));
    }
}

void EptWriter::write(const PointViewPtr view)
{
    for (const auto& addon : m_addons)
    {
        log()->get(LogLevel::Debug) << "Writing addon dimension " <<
            addon->name() << " to " << addon->ep().prefixedRoot() << std::endl;

        writeOne(view, *addon);

        log()->get(LogLevel::Debug) << "\tWritten" << std::endl;
    }
}

void EptWriter::writeOne(const PointViewPtr view, const Addon& addon) const
{
    std::vector<std::vector<char>> buffers;
    buffers.reserve(m_hierarchy.size());

    // Create an addon buffer for each node we're going to write.
    for (const auto& p : m_hierarchy)
    {
        buffers.emplace_back(p.second * addon.size(), 0);
    }

    // Fill in our buffers with the data from the view.
    PointRef pr(*view);
    uint64_t nodeId(0);
    uint64_t pointId(0);
    for (uint64_t i(0); i < view->size(); ++i)
    {
        pr.setPointId(i);
        nodeId = pr.getFieldAs<uint64_t>(m_nodeIdDim);

        // Node IDs are 1-based to distinguish points that do not come from the
        // EPT reader.
        if (!nodeId) continue;

        nodeId -= 1;
        pointId = pr.getFieldAs<uint64_t>(m_pointIdDim);

        auto& buffer(buffers.at(nodeId));
        assert(pointId * addon.size() + addon.size() <= buffer.size());
        char* dst = buffer.data() + pointId * addon.size();
        pr.getField(dst, addon.id(), addon.type());
    }

    const arbiter::Endpoint& ep(addon.ep());
    const arbiter::Endpoint dataEp(ep.getSubEndpoint("ept-data"));
    const arbiter::Endpoint hierEp(ep.getSubEndpoint("ept-hierarchy"));

    if (ep.isLocal())
    {
        arbiter::fs::mkdirp(dataEp.root());
        arbiter::fs::mkdirp(hierEp.root());
    }

    // Write the binary dimension data for the addon.
    nodeId = 0;
    for (const auto& p : m_hierarchy)
    {
        const Key key(p.first);

        m_pool->add([&dataEp, &buffers, key, nodeId]()
        {
            dataEp.put(key.toString() + ".bin", buffers.at(nodeId));
        });

        ++nodeId;
    }

    m_pool->await();

    // Write the addon hierarchy data.
    Json::Value h;
    const Key key(m_info->bounds());
    writeHierarchy(h, Key(m_info->bounds()), hierEp);
    hierEp.put(key.toString() + ".json", h.toStyledString());

    m_pool->await();

    // Write the top-level addon metadata.
    Json::Value meta;
    meta["type"] = getTypeString(addon.type());
    meta["size"] = static_cast<Json::UInt64>(addon.size());
    meta["version"] = "1.0.0";
    meta["dataType"] = "binary";

    ep.put("ept-addon.json", meta.toStyledString());
}

void EptWriter::writeHierarchy(Json::Value& curr, const Key& key,
        const arbiter::Endpoint& hierEp) const
{
    const std::string keyName(key.toString());
    if (!m_hierarchy.count(keyName)) return;

    const uint64_t np(m_hierarchy.at(keyName));
    if (!np) return;

    if (m_hierarchyStep && key.d && key.d == m_hierarchyStep)
    {
        curr[keyName] = -1;

        // Create a new hierarchy subtree.
        Json::Value next;
        next[keyName] = static_cast<Json::UInt64>(np);
        for (uint64_t dir(0); dir < 8; ++dir)
        {
            writeHierarchy(next, key.bisect(dir), hierEp);
        }

        m_pool->add([&hierEp, keyName, next]()
        {
            hierEp.put(keyName + ".json", stringify(next));
        });
    }
    else
    {
        curr[keyName] = static_cast<Json::UInt64>(np);
        for (uint64_t dir(0); dir < 8; ++dir)
        {
            writeHierarchy(curr, key.bisect(dir), hierEp);
        }
    }
}

}

