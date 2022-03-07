/******************************************************************************
* Copyright (c) 2021, Hobu Inc. (info@hobu.co)
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

#include <array>

#include <pdal/util/Bounds.hpp>

#include "VoxelKey.hpp"

namespace pdal
{
namespace copcwriter
{

// A 3-D grid of voxels.
class Grid
{
public:
    Grid(const BOX3D& bounds, size_t points);

    int calcLevel();
    void resetLevel(int level);
    VoxelKey key(double x, double y, double z);
    pdal::BOX3D processingBounds() const
        { return m_cubic ? m_cubicBounds : m_bounds; }
    pdal::BOX3D cubicBounds() const
        { return m_cubicBounds; }
    pdal::BOX3D conformingBounds() const
        { return m_bounds; }
    void scale(std::array<double, 3>& vals);
    void offset(std::array<double, 3>& vals);

    int maxLevel() const
        { return m_maxLevel; }
    void setCubic(bool cubic)
        { m_cubic = cubic; }

private:
    int m_gridSize;
    int m_maxLevel;
    pdal::BOX3D m_bounds;
    pdal::BOX3D m_cubicBounds;
    size_t m_millionPoints;
    bool m_cubic;
    double m_xsize;
    double m_ysize;
    double m_zsize;
};

} // namespace copcwriter
} // namespace pdal

