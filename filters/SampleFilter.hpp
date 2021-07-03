/******************************************************************************
 * Copyright (c) 2016, 2020 Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>

namespace pdal
{

class PDAL_DLL SampleFilter : public Filter, public Streamable
{
    using Voxel = std::tuple<int, int, int>;
    using Coord = std::tuple<double, double, double>;
    using CoordList = std::vector<Coord>;

public:
    SampleFilter() : Filter() {}
    SampleFilter& operator=(const SampleFilter&) = delete;
    SampleFilter(const SampleFilter&) = delete;

    std::string getName() const;

private:
    double m_cell;
    Arg* m_cellArg;
    double m_radius;
    double m_radiusSqr;
    Arg* m_radiusArg;
    double m_originX;
    double m_originY;
    double m_originZ;
    std::map<Voxel, CoordList> m_populatedVoxels;

    virtual void addArgs(ProgramArgs& args);
    virtual void prepared(PointTableRef table);
    virtual bool processOne(PointRef& point);
    virtual void ready(PointTableRef);
    virtual PointViewSet run(PointViewPtr view);

    bool voxelize(PointRef& point);
};

} // namespace pdal
