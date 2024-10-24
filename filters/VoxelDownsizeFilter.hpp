/******************************************************************************
 * Copyright (c) 2019, Helix.re
 * Contact Person : Pravin Shinde (pravin@helix.re,
 *                 https://github.com/pravinshinde825)
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

class PointLayout;
class PointView;

class PDAL_EXPORT VoxelDownsizeFilter : public Filter, public Streamable
{
    using Voxel = std::tuple<int, int, int>;
    enum class Mode
    {
        First,
        Center
    };
public:
    VoxelDownsizeFilter();
    VoxelDownsizeFilter& operator=(const VoxelDownsizeFilter&) = delete;
    VoxelDownsizeFilter(const VoxelDownsizeFilter&) = delete;

    std::string getName() const override;

private:
    virtual void addArgs(ProgramArgs& args) override;
    virtual PointViewSet run(PointViewPtr view) override;
    virtual void ready(PointTableRef) override;
    virtual bool processOne(PointRef& point) override;

    bool voxelize(PointRef& point);

    double m_cell;
    double m_originX;
    double m_originY;
    double m_originZ;
    std::set<Voxel> m_populatedVoxels;
    Mode m_mode;

    friend std::istream& operator>>(std::istream& in,
        VoxelDownsizeFilter::Mode&);
    friend std::ostream& operator<<(std::ostream& out,
        const VoxelDownsizeFilter::Mode&);    
};

} // namespace pdal
