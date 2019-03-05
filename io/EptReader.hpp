/******************************************************************************
* Copyright (c) 2018, Connor Manning (connor@hobu.co)
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
*       the documentation and/or key materials provided
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

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/Bounds.hpp>

namespace Json
{
    class Value;
}

namespace pdal
{

namespace arbiter
{
    class Arbiter;
    class Endpoint;
}

class Addon;
class EptInfo;
class FixedPointLayout;
class Key;
class Pool;

class PDAL_DLL EptReader : public Reader
{
public:
    EptReader();
    virtual ~EptReader();
    std::string getName() const override;

    virtual void addArgs(ProgramArgs& args) override;
    virtual void initialize() override;
    virtual QuickInfo inspect() override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void ready(PointTableRef table) override;
    virtual PointViewSet run(PointViewPtr view) override;

private:
    // If argument "origin" is specified, this function will clip the query
    // bounds to the bounds of the specified origin and set m_queryOriginId to
    // the selected OriginId value.  If the selected origin is not found, throw.
    void handleOriginQuery();

    // Aggregate all EPT keys overlapping our query bounds and their number of
    // points from a walk through the hierarchy.  Each of these keys will be
    // downloaded during the 'read' section.
    void overlaps();
    void overlaps(
            const arbiter::Endpoint& ep, std::map<Key, uint64_t>& target,
            const Json::Value& current, const Key& key);

    uint64_t readLaszip(PointView& view, const Key& key, uint64_t nodeId) const;
    uint64_t readBinary(PointView& view, const Key& key, uint64_t nodeId) const;
    void process(PointView& view, PointRef& pr, uint64_t nodeId,
            uint64_t pointId) const;

    void readAddon(PointView& dst, const Key& key, const Addon& addon,
            uint64_t startId) const;

    std::string m_root;

    std::unique_ptr<arbiter::Arbiter> m_arbiter;
    std::unique_ptr<arbiter::Endpoint> m_ep;
    std::unique_ptr<EptInfo> m_info;

    class Args
    {
    public:
        Args();

        Bounds& boundsArg() { return m_bounds; }
        std::string& originArg() { return m_origin; }
        std::size_t& threadsArg() { return m_threads; }
        double& resolutionArg() { return m_resolution; }
        Json::Value& addonsArg() { return *m_addons; }

        BOX3D bounds() const;
        std::string origin() const { return m_origin; }
        std::size_t threads() const
        {
            return std::max<std::size_t>(4, m_threads);
        }
        double resolution() const { return m_resolution; }
        const Json::Value& addons() const { return *m_addons; }

    private:
        Bounds m_bounds;
        std::string m_origin;
        std::size_t m_threads = 0;
        double m_resolution = 0;
        std::unique_ptr<Json::Value> m_addons;
    };

    Args m_args;
    BOX3D m_queryBounds;
    int64_t m_queryOriginId = -1;
    std::unique_ptr<Pool> m_pool;
    std::vector<std::unique_ptr<Addon>> m_addons;

    mutable std::mutex m_mutex;

    std::map<Key, uint64_t> m_overlaps;
    uint64_t m_depthEnd = 0;    // Zero indicates selection of all depths.
    uint64_t m_hierarchyStep = 0;

    std::unique_ptr<FixedPointLayout> m_remoteLayout;
    DimTypeList m_dimTypes;
    std::array<XForm, 3> m_xyzTransforms;

    Dimension::Id m_nodeIdDim = Dimension::Id::Unknown;
    Dimension::Id m_pointIdDim = Dimension::Id::Unknown;
};

} // namespace pdal

