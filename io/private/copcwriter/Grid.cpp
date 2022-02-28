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


#include <cmath>
#include <cstdint>

#include "Common.hpp"
#include "Grid.hpp"

namespace pdal
{
namespace copcwriter
{

Grid::Grid(const BOX3D& bounds, size_t points) : m_gridSize(-1), m_cubic(true)
{
    m_bounds.grow(bounds);
    double xside = m_bounds.maxx - m_bounds.minx;
    double yside = m_bounds.maxy - m_bounds.miny;
    double zside = m_bounds.maxz - m_bounds.minz;
    double side = (std::max)(xside, (std::max)(yside, zside));
    m_cubicBounds = BOX3D(m_bounds.minx, m_bounds.miny, m_bounds.minz,
        m_bounds.minx + side, m_bounds.miny + side, m_bounds.minz + side);

    // Here this function only gets called once, so this is simply a rounding to
    // the N-million points.
    m_millionPoints = size_t(points / 1'000'000.0);

    resetLevel(calcLevel());
}

int Grid::calcLevel()
{
    int level = 0;
    double mp = (double)m_millionPoints;

    double xside = m_bounds.maxx - m_bounds.minx;
    double yside = m_bounds.maxy - m_bounds.miny;
    double zside = m_bounds.maxz - m_bounds.minz;

    double side = (std::max)(xside, (std::max)(yside, zside));

    while (mp > MaxPointsPerNode / 1'000'000.0)
    {
        if (m_cubic)
        {
            if (xside >= side)
                mp /= 2;
            if (yside >= side)
                mp /= 2;
            if (zside >= side)
                mp /= 2;
        }
        else
            mp /= 8;
        side /= 2;
        level++;
    }

    return level;
}

void Grid::resetLevel(int level)
{
    // We have to have at least level 1 or things break when sampling.
    m_maxLevel = (std::max)(level, 1);
    m_gridSize = (int)std::pow(2, level);

    if (m_cubic)
    {
        m_xsize = (m_cubicBounds.maxx - m_cubicBounds.minx) / m_gridSize;
        m_ysize = m_xsize;
        m_zsize = m_xsize;
    }
    else
    {
        m_xsize = (m_bounds.maxx - m_bounds.minx) / m_gridSize;
        m_ysize = (m_bounds.maxy - m_bounds.miny) / m_gridSize;
        m_zsize = (m_bounds.maxz - m_bounds.minz) / m_gridSize;
    }
}

VoxelKey Grid::key(double x, double y, double z)
{
    int xi = (int)std::floor((x - m_bounds.minx) / m_xsize);
    int yi = (int)std::floor((y - m_bounds.miny) / m_ysize);
    int zi = (int)std::floor((z - m_bounds.minz) / m_zsize);
    xi = (std::min)((std::max)(0, xi), m_gridSize - 1);
    yi = (std::min)((std::max)(0, yi), m_gridSize - 1);
    zi = (std::min)((std::max)(0, zi), m_gridSize - 1);

    return VoxelKey(xi, yi, zi, m_maxLevel);
}

void Grid::offset(std::array<double, 3>& vals)
{
    // Divide before sum to avoid overflow.
    vals[0] = m_bounds.maxx / 2 + m_bounds.minx / 2;
    vals[1] = m_bounds.maxy / 2 + m_bounds.miny / 2;
    vals[2] = m_bounds.maxz / 2 + m_bounds.minz / 2;
}

void Grid::scale(std::array<double, 3>& vals)
{
    auto calcScale = [](double low, double high)
    {
        // 2 billion is a little less than the int limit.  We center the data around 0 with the
        // offset, so we're applying the scale to half the range of the data.
        double val = high / 2 - low / 2;
        double power = std::ceil(std::log10(val / 2'000'000'000.0));

        // Set an arbitrary limit on scale of 1e10-4.
        return std::pow(10, (std::max)(power, -4.0));
    };

    vals[0] = calcScale(m_bounds.minx, m_bounds.maxx);
    vals[1] = calcScale(m_bounds.miny, m_bounds.maxy);
    vals[2] = calcScale(m_bounds.minz, m_bounds.maxz);
}

} // namespace copcwriter
} // namespace pdal
