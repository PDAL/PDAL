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

#pragma once

#include <cstddef>
#include <memory>
#include <unordered_set>
#include <vector>

#include <pdal/Writer.hpp>

namespace pdal
{

class ThreadPool;

namespace connector
{
    class Connector;
}

namespace ept
{
    class EptInfo;
    class Key;
    class Addon;
    struct Overlap;
    using Hierarchy = std::unordered_set<Overlap>;
    using AddonList = std::vector<Addon>;
}

class PDAL_EXPORT EptAddonWriter : public NoFilenameWriter
{
public:
    EptAddonWriter();
    virtual ~EptAddonWriter();

    std::string getName() const override;

private:
    struct Args;
    std::unique_ptr<Args> m_args;

    virtual void addArgs(ProgramArgs& args) override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void prepared(PointTableRef table) override;
    virtual void ready(PointTableRef table) override;
    virtual void write(const PointViewPtr view) override;

    struct HierarchyWriter;
    void writeOne(const PointViewPtr view, const ept::Addon& addon, HierarchyWriter& writer) const;
    std::string getTypeString(Dimension::Type t) const;

    Dimension::Id m_nodeIdDim = Dimension::Id::Unknown;
    Dimension::Id m_pointIdDim = Dimension::Id::Unknown;

    std::unique_ptr<connector::Connector> m_connector;
    std::unique_ptr<ThreadPool> m_pool;
    std::unique_ptr<ept::EptInfo> m_info;
    std::unique_ptr<ept::Hierarchy> m_hierarchy;
    ept::AddonList m_addons;
    uint64_t m_hierarchyStep = 0;
};

}

