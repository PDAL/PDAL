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

#include <pdal/filters/Chipper.hpp>

#include <boost/scoped_ptr.hpp>

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
namespace filters
{


void Chipper::processOptions(const Options& options)
{
    m_threshold = options.getValueOrDefault<boost::uint32_t>("capacity", 5000u);
}


Options Chipper::getDefaultOptions()
{
    Options options;
    Option capacity("capacity", 5000u, "Tile capacity");
    options.add(capacity);
    return options;
}


PointBufferSet Chipper::run(PointBufferPtr buffer)
{
    m_inbuf = buffer;
    load(*buffer, m_xvec, m_yvec, m_spare);
    partition(m_xvec.size());
    decideSplit(m_xvec, m_yvec, m_spare, 0, m_partitions.size() - 1);
    return m_buffers;
}


void Chipper::load(PointBuffer& buffer, ChipRefList& xvec, ChipRefList& yvec, 
    ChipRefList& spare)
{
    ChipPtRef ref;
    boost::uint32_t idx;
    std::vector<ChipPtRef>::iterator it;

    xvec.reserve(buffer.size());
    yvec.reserve(buffer.size());
    spare.resize(buffer.size());

    for (PointId i = 0; i < buffer.size(); ++i)
    {
        ChipPtRef xref;

        xref.m_pos = buffer.getFieldAs<double>(Dimension::Id::X, i);
        xref.m_ptindex = i;
        xvec.push_back(xref);

        ChipPtRef yref;

        yref.m_pos = buffer.getFieldAs<double>(Dimension::Id::Y, i);
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
void Chipper::partition(point_count_t size)
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


void Chipper::decideSplit(ChipRefList& v1, ChipRefList& v2, ChipRefList& spare,
    PointId pleft, PointId pright)
{
    double v1range;
    double v2range;
    boost::uint32_t left = m_partitions[pleft];
    boost::uint32_t right = m_partitions[pright] - 1;

    // Decide the wider direction of the block, and split in that direction
    // to maintain squareness.
    v1range = v1[right].m_pos - v1[left].m_pos;
    v2range = v2[right].m_pos - v2[left].m_pos;
    if (v1range > v2range)
        split(v1, v2, spare, pleft, pright);
    else
        split(v2, v1, spare, pleft, pright);
}

void Chipper::split(ChipRefList& wide, ChipRefList& narrow, ChipRefList& spare,
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
        emit(wide, left, right, narrow, left, right);
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
void Chipper::finalSplit(ChipRefList& wide, ChipRefList& narrow,
    PointId pleft, PointId pright)
{

    boost::int64_t left1 = -1;
    boost::int64_t left2 = -1;
    boost::int64_t right1 = -1;
    boost::int64_t right2 = -1;

    // It appears we're using int64_t here because we're using -1 as
    // an indicator.  I'm not 100% sure that i ends up <0, but I don't
    // think so.  These casts will at least shut up the compiler, but
    // I think this code should be revisited to use std::vector<boost::uint32_t>::const_iterator
    // or std::vector<boost::uint32_t>::size_type instead of this int64_t stuff -- hobu 11/15/10
    boost::int64_t left = m_partitions[pleft];
    boost::int64_t right = static_cast<boost::int64_t>(m_partitions[pright] - 1);
    boost::int64_t center = static_cast<boost::int64_t>(m_partitions[pright - 1]);

    // Find left values for the partitions.
    for (boost::int64_t i = left; i <= right; ++i)
    {
        boost::int64_t idx = static_cast<boost::int64_t>(narrow[static_cast<boost::uint32_t>(i)].m_oindex);
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
    for (boost::int64_t i = right; i >= left; --i)
    {
        boost::int64_t idx = static_cast<boost::int64_t>(narrow[static_cast<boost::uint32_t>(i)].m_oindex);
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
         center - 1,
         narrow,
         left1,
         right1);
    emit(wide,
         center,
         right,
         narrow,
         left2,
         right2);
}

void Chipper::emit(ChipRefList& wide, PointId widemin, PointId widemax,
    ChipRefList& narrow, PointId narrowmin, PointId narrowmax)
{
    PointBufferPtr buf = m_inbuf->makeNew();
    for (size_t idx = widemin; idx <= widemax; ++idx)
        buf->appendPoint(*m_inbuf, wide[idx].m_ptindex);

    /**
    // We currently don't write the bounds in the buffer.
    //
    Bounds<double> bounds;
    if (wide.m_dir == DIR_X)
        bounds = Bounds<double>(wide[widemin].m_pos, narrow[narrowmin].m_pos,
            wide[widemax].m_pos, narrow[narrowmax].m_pos);
    else
        bounds = Bounds<double>(narrow[narrowmin].m_pos, wide[widemin].m_pos,
            narrow[narrowmax].m_pos, wide[widemax].m_pos);
    **/
    m_buffers.insert(buf);
}


} // namespace filters
} // namespace pdal
