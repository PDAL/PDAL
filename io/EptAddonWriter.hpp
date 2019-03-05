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

#include <pdal/Writer.hpp>

namespace Json { class Value; }

namespace pdal
{

namespace arbiter
{
    class Arbiter;
    class Endpoint;
}

class Addon;
class EptInfo;
class Key;
class Pool;

class PDAL_DLL EptAddonWriter : public Writer
{
public:
    EptAddonWriter();
    virtual ~EptAddonWriter();

    std::string getName() const override;
    virtual void addArgs(ProgramArgs& args) override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void prepared(PointTableRef table) override;
    virtual void ready(PointTableRef table) override;
    virtual void write(const PointViewPtr view) override;

private:
    std::unique_ptr<arbiter::Arbiter> m_arbiter;
    std::unique_ptr<Pool> m_pool;

    std::unique_ptr<Json::Value> m_addonsArg;
    std::vector<std::unique_ptr<Addon>> m_addons;

    void writeOne(const PointViewPtr view, const Addon& addon) const;
    void writeHierarchy(Json::Value& hier, const Key& key,
            const arbiter::Endpoint& hierEp) const;
    std::string getTypeString(Dimension::Type t) const;

    std::size_t m_numThreads = 0;

    Dimension::Id m_nodeIdDim = Dimension::Id::Unknown;
    Dimension::Id m_pointIdDim = Dimension::Id::Unknown;

    std::unique_ptr<EptInfo> m_info;
    std::map<Key, uint64_t> m_hierarchy;
    uint64_t m_hierarchyStep = 0;
};

}

