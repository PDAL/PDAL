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
#include <condition_variable>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <pdal/JsonFwd.hpp>
#include <pdal/Polygon.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/Bounds.hpp>

namespace pdal
{

namespace arbiter
{
    class Arbiter;
    class Endpoint;
    class LocalHandle;
}

class Addon;
class EptInfo;
class FixedPointLayout;
class Key;
class Pool;
class VectorPointTable;

class PDAL_DLL EptReader : public Reader, public Streamable
{
    FRIEND_TEST(EptReaderTest, getRemoteType);
    FRIEND_TEST(EptReaderTest, getCoercedType);

public:
    EptReader();
    virtual ~EptReader();
    std::string getName() const override;

private:
    virtual void addArgs(ProgramArgs& args) override;
    virtual void initialize() override;
    virtual QuickInfo inspect() override;
    virtual void addDimensions(PointLayoutPtr layout) override;
    virtual void ready(PointTableRef table) override;
    virtual PointViewSet run(PointViewPtr view) override;

    // Users may supply header and query parameters to be forwarded with remote
    // requests, deconstruct their JSON into our member maps.
    void initializeHttpForwards();

    // If argument "origin" is specified, this function will clip the query
    // bounds to the bounds of the specified origin and set m_queryOriginId to
    // the selected OriginId value.  If the selected origin is not found, throw.
    void handleOriginQuery();

    // Aggregate all EPT keys overlapping our query bounds and their number of
    // points from a walk through the hierarchy.  Each of these keys will be
    // downloaded during the 'read' section.
    void overlaps();
    void overlaps(const arbiter::Endpoint& ep, std::map<Key, uint64_t>& target,
            const NL::json& current, const Key& key);

    PointId readLaszip(PointView& view, const Key& key, uint64_t nodeId) const;
    PointId readBinary(PointView& view, const Key& key, uint64_t nodeId) const;
    PointId readZstandard(PointView& view, const Key& key, uint64_t nodeId)
        const;
    PointId processPackedData(PointView& view, uint64_t nodeId, char* data,
        uint64_t size) const;
    void process(PointView& view, PointRef& pr, uint64_t nodeId,
        PointId pointId) const;

    void readAddon(PointView& dst, const Key& key, const Addon& addon,
        PointId startId = 0) const;

    // To allow testing of hidden getRemoteType() and getCoercedType().
    static Dimension::Type getRemoteTypeTest(const NL::json& dimInfo);
    static Dimension::Type getCoercedTypeTest(const NL::json& dimInfo);

    // For streaming operation.
    struct NodeBuffer;
    using NodeBufferList = std::list<std::unique_ptr<NodeBuffer>>;
    using NodeBufferIt = NodeBufferList::iterator;

    virtual bool processOne(PointRef& point) override;
    void load();    // Asynchronously fetch EPT nodes for streaming use.
    bool next();    // Acquire an already-fetched node for processing.
    NodeBufferIt findBuffer();  // Find a fully acquired node.

    // Data fetching - these forward user-specified query/header params.
    std::string get(std::string path) const;
    std::vector<char> getBinary(std::string path) const;
    std::unique_ptr<arbiter::LocalHandle> getLocalHandle(std::string path)
        const;

    std::string m_root;

    std::unique_ptr<arbiter::Arbiter> m_arbiter;
    std::unique_ptr<arbiter::Endpoint> m_ep;
    std::unique_ptr<EptInfo> m_info;

    struct Args;

    std::unique_ptr<Args> m_args;

    BOX3D m_queryBounds;
    int64_t m_queryOriginId = -1;
    std::unique_ptr<Pool> m_pool;
    std::vector<std::unique_ptr<Addon>> m_addons;

    using StringMap = std::map<std::string, std::string>;
    StringMap m_headers;
    StringMap m_query;

    mutable std::mutex m_mutex;
    mutable std::condition_variable m_cv;

    using Overlaps = std::map<Key, uint64_t>;
    Overlaps m_overlaps;
    uint64_t m_depthEnd = 0;    // Zero indicates selection of all depths.
    uint64_t m_hierarchyStep = 0;

    std::unique_ptr<FixedPointLayout> m_remoteLayout;
    DimTypeList m_dimTypes;
    std::array<XForm, 3> m_xyzTransforms;

    Dimension::Id m_nodeIdDim = Dimension::Id::Unknown;
    Dimension::Id m_pointIdDim = Dimension::Id::Unknown;

    // The below are for streaming operation only.
    PointLayout* m_userLayout = nullptr;

    // These represent a lookahead of asynchronously loaded nodes, when we have
    // finished processing a streaming node we will wait for something to be
    // loaded here.
    NodeBufferList m_upcomingNodeBuffers;

    // This is the node we are currently processing in streaming mode, which is
    // plucked out of our upcoming node buffers when we have finished our
    // current buffer.
    std::unique_ptr<NodeBuffer> m_currentNodeBuffer;

    // The below represent our current state in streaming operation - in normal
    // mode we use local variables for these.
    Overlaps::const_iterator m_overlapIt;
    uint64_t m_nodeId = 1;
    PointId m_pointId = 0;
};

} // namespace pdal

