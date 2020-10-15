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

#include <map>
#include <string>
#include <memory>
#include <unordered_set>

#include <pdal/JsonFwd.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/Bounds.hpp>

namespace pdal
{

class Connector;
class EptInfo;
class Key;
class TileContents;
struct Overlap;
using Hierarchy = std::unordered_set<Overlap>;
using StringMap = std::map<std::string, std::string>;

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
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual bool processOne(PointRef& point) override;

    // If argument "origin" is specified, this function will clip the query
    // bounds to the bounds of the specified origin and set m_queryOriginId to
    // the selected OriginId value.  If the selected origin is not found, throw.
    void handleOriginQuery();
    void setForwards(StringMap& headers, StringMap& query);

    // Aggregate all EPT keys overlapping our query bounds and their number of
    // points from a walk through the hierarchy.  Each of these keys will be
    // downloaded during the 'read' section.
    void overlaps();
    void overlaps(Hierarchy& target, const NL::json& current, const Key& key);
    void process(PointViewPtr dstView, const TileContents& tile, point_count_t count);
    bool processPoint(PointRef& dst, const TileContents& tile);
    void load(const Overlap& overlap);
    void checkTile(const TileContents& tile);

    struct Args;
    std::unique_ptr<Args> m_args;
    struct Private;
    std::unique_ptr<Private> m_p;

    uint64_t m_tileCount;
    BOX3D m_queryBounds;
    int64_t m_queryOriginId = -1;

    uint64_t m_depthEnd = 0;    // Zero indicates selection of all depths.
    uint64_t m_hierarchyStep = 0;

    Dimension::Id m_nodeIdDim = Dimension::Id::Unknown;
    Dimension::Id m_pointIdDim = Dimension::Id::Unknown;

    ArtifactManager *m_artifactMgr;
    PointId m_pointId = 0;
    uint64_t m_nodeId;
};

} // namespace pdal

