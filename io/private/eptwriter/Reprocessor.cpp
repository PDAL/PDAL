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


// This include is necessary for released PDAL 2.0 and earlier, as it wasn't included in
// FileUtils.hpp.
#include <vector>
#include <pdal/util/FileUtils.hpp>

#include "Ept.hpp"
#include "Reprocessor.hpp"

namespace pdal
{
namespace ept
{

Reprocessor::Reprocessor(CellManager& mgr, PointViewPtr srcView, Grid grid) :
    m_mgr(mgr), m_srcView(srcView), m_grid(grid)
{
    // We make an assumption that at most twice the number of points will be in a cell
    // than there would be if the distribution was uniform, so we calculate based on
    // each level breaking the points into 4.
    // So, to find the number of levels, we need to solve for n:
    //
    // numPoints / (4^n) = MaxPointsPerNode
    //  =>
    // numPoints / MaxPointsPerNode = 2^(2n)
    //  =>
    // log2(numPoints / MaxPointsPerNode) = 2n

    m_levels = (int)std::ceil(log2((double)srcView->size() / MaxPointsPerNode) / 2);

    // We're going to steal points from the leaf nodes for sampling, so unless the
    // spatial distribution is really off, this should be fine and pretty conservative.

    m_grid.resetLevel(m_grid.maxLevel() + m_levels);
}

void Reprocessor::run()
{
    // Remove the reprocessed cell from th4e map as its 
    for (PointRef p : *m_srcView)
    {
        double x = p.getFieldAs<double>(Dimension::Id::X);
        double y = p.getFieldAs<double>(Dimension::Id::Y);
        double z = p.getFieldAs<double>(Dimension::Id::Z);
        VoxelKey k = m_grid.key(x, y, z);
        PointViewPtr& cell = m_mgr.get(k);
        cell->appendPoint(*cell, p.pointId());
    }
}

} // namespace epf
} // namespace untwine
