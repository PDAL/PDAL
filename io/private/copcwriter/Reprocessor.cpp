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


#include <pdal/util/FileUtils.hpp>

#include "Common.hpp"
#include "Reprocessor.hpp"

namespace pdal
{
namespace copcwriter
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
        cell->appendPoint(*m_srcView, p.pointId());
    }
}

} // namespace copcwriter
} // namespace pdal
