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

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>

//#include "private/octree/Types.hpp"

namespace pdal
{

class Accessor;
class Hierarchy;
using HierarchyPage = Hierarchy;
struct OctArgs;
struct OctPrivate;
class Key;
class Tile;
using TilePtr = std::unique_ptr<Tile>;

class PDAL_DLL OctReader : public Reader, public Streamable
{
protected:
    OctReader();
    virtual ~OctReader();

    void addBaseArgs(ProgramArgs& args);
    void initializeBase();
    QuickInfo inspectBase();
    void readyBase(PointTableRef table);
    void baseCalcOverlaps(Hierarchy& hierarchy, const Accessor& acc);
    bool passesBasePointFilter(PointRef& p, double x, double y, double z) const;
    bool baseProcessPoint(PointRef& dst, const Tile& tile);
    point_count_t baseRead(PointViewPtr view, point_count_t count);

    std::unique_ptr<OctArgs> m_args;
    std::unique_ptr<OctPrivate> m_p;

private:
    virtual point_count_t read(PointViewPtr view, point_count_t count) override;
    virtual bool processOne(PointRef& point) override;
    virtual bool passesHierarchyFilter(const Key& k) const;
    virtual bool passesPointFilter(PointRef& p, double x, double y, double z) const;
    virtual bool processPoint(PointRef& dst, const Tile& tile);
    virtual TilePtr makeTile(const Accessor& accessor) const = 0;
    virtual HierarchyPage fetchHierarchyPage(Hierarchy& hierarchy, const Accessor& key) const = 0;
    virtual double rootNodeHalfWidth() const = 0;
    virtual void rootNodeCenter(double& x, double& y, double& z) const = 0;
    virtual double rootNodeSpacing() const = 0;
    virtual BOX3D pointBounds() const = 0;
    virtual point_count_t pointCount() const = 0;
    virtual StringList dimNames() const = 0;
    virtual void calcOverlaps() = 0;

    void createSpatialFilters();
    bool hasSpatialFilter() const;
    bool passesSpatialFilter(const Key& key) const;
    void calcOverlaps(Hierarchy& target, const HierarchyPage& page, const Accessor& acc);
    void checkTile(const Tile& tile);
    void process(PointViewPtr dstView, const Tile& tile, point_count_t count);
    void load(const Accessor& acc);
};

} // namespace pdal

