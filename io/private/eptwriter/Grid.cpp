/*****************************************************************************
 *   Copyright (c) 2020, Hobu, Inc. (info@hobu.co)                           *
 *                                                                           *
 *   All rights reserved.                                                    *
 *                                                                           *
 *   This program is free software; you can redistribute it and/or modify    *
 *   it under the terms of the GNU General Public License as published by    *
 *   the Free Software Foundation; either version 3 of the License, or       *
 *   (at your option) any later version.                                     *
 *                                                                           *
 ****************************************************************************/


#include <cmath>
#include <cstdint>

#include "Ept.hpp"
#include "Grid.hpp"

using namespace pdal;

namespace pdal
{
namespace ept
{

void Grid::expand(const BOX3D& bounds, size_t points)
{
    m_bounds.grow(bounds);
    double xside = m_bounds.maxx - m_bounds.minx;
    double yside = m_bounds.maxy - m_bounds.miny;
    double zside = m_bounds.maxz - m_bounds.minz;
    std::cerr << "X/Y/Z len = " << xside << "/" << yside << "/" << zside << "!\n";
    double side = (std::max)(xside, (std::max)(yside, zside));
    m_cubicBounds = BOX3D(m_bounds.minx, m_bounds.miny, m_bounds.minz,
        m_bounds.minx + side, m_bounds.miny + side, m_bounds.minz + side);
    m_millionPoints += size_t(points / 1000000.0);

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

    std::cerr << "Million = " << mp << "!\n";
    while (mp > MaxPointsPerNode / 1000000.0)
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
        double power = std::ceil(std::log10(val / 2000000000.0));

        // Set an arbitrary limit on scale of 1e10-4.
        return std::pow(10, (std::max)(power, -4.0));
    };

    vals[0] = calcScale(m_bounds.minx, m_bounds.maxx);
    vals[1] = calcScale(m_bounds.miny, m_bounds.maxy);
    vals[2] = calcScale(m_bounds.minz, m_bounds.maxz);
}

} // namespace ept
} // namespace pdal
