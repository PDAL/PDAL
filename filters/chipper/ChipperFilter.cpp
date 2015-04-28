/******************************************************************************
 * Copyright (c) 2010, Andrew Bell
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
 *     * Neither the name of the Andrew Bell or libLAS nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#include "ChipperFilter.hpp"

#include <iostream>
#include <limits>

/**
The objective is to split the region into non-overlapping blocks, each
containing approximately the same number of points, as specified by the
user.  We'd also like the blocks closer to square than not.

First, the points are read into arrays - one for the x direction, and one for
the y direction.  The arrays are sorted and are initialized with indices into
the other array of the location of the other coordinate of the same point.

Partitions are created that place the maximum number of points in a
block, subject to the user-defined threshold, using a cumulate and round
procedure.

The distance of the point-space is checked in each direction and the
wider dimension is chosen for splitting at an appropriate partition point.
The points in the narrower direction are copied to locations in the spare
array at one side or the other of the chosen partition, and that portion
of the spare array then becomes the active array for the narrow direction.
This avoids resorting of the arrays, which are already sorted.

This procedure is then recursively applied to the created blocks until
they contains only one or two partitions.  In the case of one partition,
we are done, and we simply store away the contents of the block.  If there are
two partitions in a block, we avoid the recopying the narrow array to the
spare since the wide array already contains the desired points partitioned
into two blocks.  We simply need to locate the maximum and minimum values
from the narrow array so that the approriate extrema of the block can
be stored.
**/

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.chipper",
    "Organize points into spatially contiguous, squarish, and non-overlapping chips.",
    "http://pdal.io/stages/filters.chipper.html" );

CREATE_STATIC_PLUGIN(1, 0, ChipperFilter, Filter, s_info)

std::string ChipperFilter::getName() const { return s_info.name; }

void ChipperFilter::processOptions(const Options& options)
{
    m_threshold = options.getValueOrDefault<uint32_t>("capacity", 5000u);
}


Options ChipperFilter::getDefaultOptions()
{
    Options options;
    Option capacity("capacity", 5000u, "Tile capacity");
    options.add(capacity);
    return options;
}


PointViewSet ChipperFilter::run(PointViewPtr view)
{
    if (view->size() == 0)
        return m_outViews;

    m_inView = view;
    load(*view.get(), m_xvec, m_yvec, m_spare);
    partition(m_xvec.size());
    decideSplit(m_xvec, m_yvec, m_spare, 0, m_partitions.size() - 1);
    return m_outViews;
}


void ChipperFilter::load(PointView& view, ChipRefList& xvec, ChipRefList& yvec,
    ChipRefList& spare)
{
    point_count_t idx;
    std::vector<ChipPtRef>::iterator it;

    xvec.reserve(view.size());
    yvec.reserve(view.size());
    spare.resize(view.size());

    for (PointId i = 0; i < view.size(); ++i)
    {
        ChipPtRef xref;

        xref.m_pos = view.getFieldAs<double>(Dimension::Id::X, i);
        xref.m_ptindex = i;
        xvec.push_back(xref);

        ChipPtRef yref;

        yref.m_pos = view.getFieldAs<double>(Dimension::Id::Y, i);
        yref.m_ptindex = i;
        yvec.push_back(yref);
    }

    // Sort xvec and assign other index in yvec to sorted indices in xvec.
    std::stable_sort(xvec.begin(), xvec.end());
    for (size_t i = 0; i < xvec.size(); ++i)
    {
        idx = xvec[i].m_ptindex;
        yvec[idx].m_oindex = i;
    }

    // Sort yvec.
    std::stable_sort(yvec.begin(), yvec.end());

    // Iterate through the yvector, setting the xvector appropriately.
    for (size_t i = 0; i < yvec.size(); ++i)
        xvec[yvec[i].m_oindex].m_oindex = i;
}


// Build a list of partitions.  The partition is the size of each block in
// the x and y directions in number of points.
void ChipperFilter::partition(point_count_t size)
{
    size_t num_partitions;

    num_partitions = size / m_threshold;
    if (size % m_threshold)
        num_partitions++;

    // This is a standard statistics cumulate and round.  It distributes
    // the points into partitions such the "extra" points are reasonably
    // distributed among the partitions.
    double total(0.0);
    double partition_size = static_cast<double>(size) / num_partitions;
    m_partitions.push_back(0);
    for (size_t i = 0; i < num_partitions; ++i)
    {
        total += partition_size;
        size_t itotal = lround(total);
        m_partitions.push_back(itotal);
    }
}


void ChipperFilter::decideSplit(ChipRefList& v1, ChipRefList& v2, ChipRefList& spare,
    PointId pleft, PointId pright)
{
    double v1range;
    double v2range;
    uint32_t left = m_partitions[pleft];
    uint32_t right = m_partitions[pright] - 1;

    // Decide the wider direction of the block, and split in that direction
    // to maintain squareness.
    v1range = v1[right].m_pos - v1[left].m_pos;
    v2range = v2[right].m_pos - v2[left].m_pos;
    if (v1range > v2range)
        split(v1, v2, spare, pleft, pright);
    else
        split(v2, v1, spare, pleft, pright);
}

void ChipperFilter::split(ChipRefList& wide, ChipRefList& narrow, ChipRefList& spare,
    PointId pleft, PointId pright)
{
    PointId lstart;
    PointId rstart;
    PointId pcenter;
    PointId left;
    PointId right;
    PointId center;

    left = m_partitions[pleft];
    right = m_partitions[pright] - 1;

    // There are two cases in which we are done.
    // 1) We have a distance of two between left and right.
    // 2) We have a distance of three between left and right.

    if (pright - pleft == 1)
        emit(wide, left, right);
    else if (pright - pleft == 2)
        finalSplit(wide, narrow, pleft, pright);
    else
    {
        pcenter = (pleft + pright) / 2;
        center = m_partitions[pcenter];

        // We are splitting in the wide direction - split elements in the
        // narrow array by copying them to the spare array in the correct
        // partition.  The spare array then becomes the active narrow array
        // for the [left,right] partition.
        lstart = left;
        rstart = center;
        for (PointId i = left; i <= right; ++i)
        {
            if (narrow[i].m_oindex < center)
            {
                spare[lstart] = narrow[i];
                wide[narrow[i].m_oindex].m_oindex = lstart;
                lstart++;
            }
            else
            {
                spare[rstart] = narrow[i];
                wide[narrow[i].m_oindex].m_oindex = rstart;
                rstart++;
            }
        }

        // Save away the direction so we know which array is X and which is Y
        // so that when we emit, we can properly label the max/min points.
        Direction dir = narrow.m_dir;
        spare.m_dir = dir;
        decideSplit(wide, spare, narrow, pleft, pcenter);
        decideSplit(wide, spare, narrow, pcenter, pright);
        narrow.m_dir = dir;
    }
}

// In this case the wide array is like we want it.  The narrow array is
// ordered, but not for our split, so we have to find the max/min entries
// for each partition in the final split.
void ChipperFilter::finalSplit(ChipRefList& wide, ChipRefList& narrow,
    PointId pleft, PointId pright)
{

    int64_t left1 = -1;
    int64_t left2 = -1;
    int64_t right1 = -1;
    int64_t right2 = -1;

    // It appears we're using int64_t here because we're using -1 as
    // an indicator.  I'm not 100% sure that i ends up <0, but I don't
    // think so.  These casts will at least shut up the compiler, but
    // I think this code should be revisited to use std::vector<uint32_t>::const_iterator
    // or std::vector<uint32_t>::size_type instead of this int64_t stuff -- hobu 11/15/10
    int64_t left = m_partitions[pleft];
    int64_t right = static_cast<int64_t>(m_partitions[pright] - 1);
    int64_t center = static_cast<int64_t>(m_partitions[pright - 1]);

    // Find left values for the partitions.
    for (int64_t i = left; i <= right; ++i)
    {
        int64_t idx = static_cast<int64_t>(narrow[static_cast<uint32_t>(i)].m_oindex);
        if (left1 < 0 && (idx < center))
        {
            left1 = i;
            if (left2 >= 0)
                break;
        }
        else if (left2 < 0 && (idx >= center))
        {
            left2 = i;
            if (left1 >= 0)
                break;
        }
    }
    // Find right values for the partitions.
    for (int64_t i = right; i >= left; --i)
    {
        int64_t idx = static_cast<int64_t>(narrow[static_cast<uint32_t>(i)].m_oindex);
        if (right1 < 0 && (idx < center))
        {
            right1 = i;
            if (right2 >= 0)
                break;
        }
        else if (right2 < 0 && (idx >= center))
        {
            right2 = i;
            if (right1 >= 0)
                break;
        }
    }

    // Emit results.
    emit(wide,
         left,
         center - 1);
    emit(wide,
         center,
         right);
}

void ChipperFilter::emit(ChipRefList& wide, PointId widemin, PointId widemax)
{
    PointViewPtr view = m_inView->makeNew();
    for (size_t idx = widemin; idx <= widemax; ++idx)
        view->appendPoint(*m_inView.get(), wide[idx].m_ptindex);

    m_outViews.insert(view);
}

} // namespace pdal

