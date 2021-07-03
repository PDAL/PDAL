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

#include "EptAddonWriter.hpp"

#include <cassert>

#include <nlohmann/json.hpp>

#include <pdal/ArtifactManager.hpp>
#include <pdal/util/ThreadPool.hpp>

#include "private/ept/Addon.hpp"
#include "private/ept/EptArtifact.hpp"
#include "private/ept/Connector.hpp"

namespace pdal
{

namespace
{
    const StaticPluginInfo s_info
    {
        "writers.ept_addon",
        "EPT Writer",
        "http://pdal.io/stages/writers.ept.html",
        { "ept_addon", "ept-addon" }
    };
}

CREATE_STATIC_STAGE(EptAddonWriter, s_info)

struct EptAddonWriter::Args
{
    NL::json m_addons;
    std::size_t m_numThreads;
};

EptAddonWriter::EptAddonWriter() : m_args(new Args)
{}

EptAddonWriter::~EptAddonWriter()
{}

std::string EptAddonWriter::getName() const { return s_info.name; }

void EptAddonWriter::addArgs(ProgramArgs& args)
{
    args.add("addons", "Mapping of output locations to their dimension names",
            m_args->m_addons).setPositional();
    args.add("threads", "Number of worker threads", m_args->m_numThreads);
}

void EptAddonWriter::addDimensions(PointLayoutPtr layout)
{
    m_nodeIdDim = layout->registerOrAssignDim("EptNodeId",
            Dimension::Type::Unsigned32);
    m_pointIdDim = layout->registerOrAssignDim("EptPointId",
            Dimension::Type::Unsigned32);
}

void EptAddonWriter::prepared(PointTableRef table)
{
    const std::size_t threads(std::max<std::size_t>(m_args->m_numThreads, 4));
    if (threads > 100)
    {
        log()->get(LogLevel::Warning) << "Using a large thread count: " <<
            threads << " threads" << std::endl;
    }
    m_pool.reset(new ThreadPool(threads));

    // Note that we use a generic connector here.  In ready() we steal
    // the connector that was used in the EptReader that uses any
    // headers/query that was set.
    m_connector.reset(new Connector());
    m_addons = Addon::store(*m_connector, m_args->m_addons, *(table.layout()));
}

void EptAddonWriter::ready(PointTableRef table)
{
    EptArtifactPtr eap = table.artifactManager().get<EptArtifact>("ept");
    if (!eap)
    {
        throwError(
                "Cannot use writers.ept_addon without reading using "
                "readers.ept");
    }

    m_hierarchyStep = eap->m_hierarchyStep;
    m_info = std::move(eap->m_info);
    m_hierarchy = std::move(eap->m_hierarchy);
    m_connector = std::move(eap->m_connector);
}

void EptAddonWriter::write(const PointViewPtr view)
{
    for (const auto& addon : m_addons)
    {
        log()->get(LogLevel::Debug) << "Writing addon dimension " <<
            addon.name() << " to " << addon.filename() << std::endl;

        writeOne(view, addon);

        log()->get(LogLevel::Debug) << "\tWritten" << std::endl;
    }
}

void EptAddonWriter::writeOne(const PointViewPtr view, const Addon& addon) const
{
    std::vector<std::vector<char>> buffers(m_hierarchy->size());

    // Create an addon buffer for each node we're going to write.

    size_t itemSize = Dimension::size(addon.type());
    for (const Overlap& overlap : *m_hierarchy)
    {
        std::vector<char>& b = buffers[overlap.m_nodeId - 1];
        b.resize(overlap.m_count * itemSize);
    }

    // Fill in our buffers with the data from the view.
    PointRef pr(*view);
    uint64_t nodeId(0);
    uint64_t pointId(0);
    for (PointId i = 0; i < view->size(); ++i)
    {
        pr.setPointId(i);
        nodeId = pr.getFieldAs<uint64_t>(m_nodeIdDim);

        // Node IDs are 1-based to distinguish points that do not come from the
        // EPT reader.
        if (!nodeId)
            continue;

        pointId = pr.getFieldAs<uint64_t>(m_pointIdDim);

        auto& buffer(buffers.at(nodeId - 1));
        assert(pointId * itemSize + itemSize <= buffer.size());
        char* dst = buffer.data() + pointId * itemSize;
        pr.getField(dst, addon.externalId(), addon.type());
    }

    std::string dataDir = addon.dataDir();

    m_connector->makeDir(dataDir);

    // Write the binary dimension data for the addon.
    for (const Overlap& overlap : *m_hierarchy)
    {
        std::vector<char>& buffer = buffers.at(overlap.m_nodeId - 1);
        std::string filename = dataDir + overlap.m_key.toString() + ".bin";
        m_pool->add([this, filename, &buffer]()
        {
            m_connector->put(filename, buffer);
        });
    }

    m_pool->await();

    // Write the addon hierarchy data.
    NL::json h;
    Key key;
    key.b = m_info->bounds();

    std::string hierarchyDir = addon.hierarchyDir();
    m_connector->makeDir(hierarchyDir);

    writeHierarchy(hierarchyDir, h, key);
    std::string filename = hierarchyDir + key.toString() + ".json";
    m_connector->put(filename, h.dump());

    m_pool->await();

    // Write the top-level addon metadata.
    NL::json meta;
    meta["type"] = Dimension::toName(Dimension::base(addon.type()));
    meta["size"] = Dimension::size(addon.type());
    meta["version"] = "1.0.0";
    meta["dataType"] = "binary";

    m_connector->put("ept-addon.json", meta.dump());
}

void EptAddonWriter::writeHierarchy(const std::string& directory,
    NL::json& curr, const Key& key) const
{
    auto it = m_hierarchy->find(key);
    if (it == m_hierarchy->end())
        return;

    const Overlap& overlap = *it;
    if (!overlap.m_count)
        return;

    const std::string keyName = key.toString();
    if (m_hierarchyStep && key.d && (key.d % m_hierarchyStep == 0))
    {
        curr[keyName] = -1;

        // Create a new hierarchy subtree.
        NL::json next {{ keyName, overlap.m_count }};

        for (uint64_t dir(0); dir < 8; ++dir)
            writeHierarchy(directory, next, key.bisect(dir));

        std::string filename = directory + keyName + ".json";
        std::string data = next.dump();
        m_pool->add([this, filename, data]()
        {
            m_connector->put(filename, data);
        });
    }
    else
    {
        curr[keyName] = overlap.m_count;
        for (uint64_t dir(0); dir < 8; ++dir)
            writeHierarchy(directory, curr, key.bisect(dir));
    }
}

}

