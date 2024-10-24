/******************************************************************************
* Copyright (c) 2018, Hobu Inc.
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
*     * Neither the name of Hobu, Inc. nor the
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

#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>

namespace pdal
{

class BOX3D;

// This is just a pass-through filter, which collects some data about
// the points that are fed through it
class PDAL_EXPORT InfoFilter : public Filter, public Streamable
{
    struct NearPoint
    {
        NearPoint(PointId id, double dist, std::vector<char>&& data) :
            m_id(id), m_dist(dist), m_data(data)
        {}

        PointId m_id;
        double m_dist;
        std::vector<char> m_data;

        bool operator < (const NearPoint& other) const
            { return m_dist < other.m_dist; }
    };

public:
    InfoFilter() :
        m_pointRoot("point"), m_queryCount(10),
        m_queryZ(std::numeric_limits<double>::quiet_NaN())
    {}

    std::string getName() const;
    BOX3D bounds() const
        { return m_bounds; }

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize(PointTableRef table);
    virtual void ready(PointTableRef table);
    virtual bool processOne(PointRef& point);
    virtual void prepared(PointTableRef table);
    virtual void done(PointTableRef table);
    virtual void filter(PointView& view);

    void parsePointSpec();
    void parseQuerySpec();

    MetadataNode m_pointRoot;

    std::string m_querySpec;
    point_count_t m_queryCount;
    double m_queryX;
    double m_queryY;
    double m_queryZ;
    std::list<NearPoint> m_results;

    std::string m_pointSpec;
    PointIdList m_idList;
    PointIdList::const_iterator m_idCur;
    DimTypeList m_dims;
    size_t m_pointSize;
    PointId m_count;

    BOX3D m_bounds;
};

} // namespace pdal
