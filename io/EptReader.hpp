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

#include "OctReader.hpp"

namespace pdal
{

class Connector;
class EptInfo;
class Key;
class Tile;
class Accessor;
struct EptArgs;
struct EptPrivate;
using TilePtr = std::unique_ptr<Tile>;
using StringMap = std::map<std::string, std::string>;

class PDAL_DLL EptReader : public OctReader
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
    virtual bool passesPointFilter(PointRef& p, double x, double y, double z) const override;
    virtual bool processPoint(PointRef& dst, const Tile& tile) override;
    virtual TilePtr makeTile(const Accessor& accessor) const override;
    virtual HierarchyPage fetchHierarchyPage(Hierarchy& hierarchy,
        const Accessor& k) const override;
    virtual double rootNodeHalfWidth() const override;
    virtual void rootNodeCenter(double& x, double& y, double &z) const override;
    virtual double rootNodeSpacing() const override;
    virtual BOX3D pointBounds() const override;
    virtual point_count_t pointCount() const override;
    virtual StringList dimNames() const override;
    virtual void calcOverlaps() override;

    // If argument "origin" is specified, this function will clip the query
    // bounds to the bounds of the specified origin and set m_queryOriginId to
    // the selected OriginId value.  If the selected origin is not found, throw.
    void handleOriginQuery();
    void setForwards(StringMap& headers, StringMap& query);

    std::unique_ptr<EptArgs> m_eptArgs;
    std::unique_ptr<EptPrivate> m_e;

    int64_t m_queryOriginId = -1;

    Dimension::Id m_nodeIdDim = Dimension::Id::Unknown;
    Dimension::Id m_pointIdDim = Dimension::Id::Unknown;

    ArtifactManager *m_artifactMgr;
    PointId m_pointId = 0;
    uint64_t m_nodeId;
};

} // namespace pdal

