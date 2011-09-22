/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  Point Partitioning/blocking for OPC
 * Author:   Andrew Bell andrew.bell.ia at gmail.com
 *
 ******************************************************************************
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

#include <pdal/filters/ChipperIterator.hpp>

#include <boost/scoped_ptr.hpp>

#include <iostream>
#include <limits>

using namespace std;
using namespace pdal::filters::chipper;

/**
The objective is to split the region into non-overlapping blocks, each
containing approximately the same number of points, as specified by the
user.

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

namespace pdal { namespace filters {

vector<boost::uint32_t> Block::GetIDs() const
{
    vector<boost::uint32_t> ids;

    for (boost::uint32_t i = m_left; i <= m_right; ++i)
        ids.push_back((*m_list_p)[i].m_ptindex);
    return ids;
}

void Block::GetBuffer( Stage const& stage, PointBuffer& buffer, boost::uint32_t block_id, boost::int32_t indexId, boost::int32_t indexBlockId) const
{
    pdal::Schema const& schema = buffer.getSchema();

    boost::int32_t size = m_right - m_left + 1;
    if (size < 0)
        throw pdal_error("m_right - m_left + 1 was less than 0 in Block::GetBuffer()!");

    if (!stage.supportsIterator(StageIterator_Random))
        throw pdal_error("Chipper GetBuffer is unable to read data source randomly!");
            
    boost::scoped_ptr<StageRandomIterator> iter(stage.createRandomIterator());

    std::vector<boost::uint32_t> ids = GetIDs();
    pdal::PointBuffer one_point(schema, 1);

    std::vector<boost::uint32_t>::const_iterator it;
    boost::uint32_t count = 0;
    
    // const int indexId = schema.getDimensionIndex(Dimension_User1, Dimension::Int32);
    // const int indexBlockId = schema.getDimensionIndex(Dimension_User2, Dimension::Int32);
    
    boost::uint8_t* p_block_id = reinterpret_cast<boost::uint8_t*>(&block_id);
    for (it = ids.begin(); it != ids.end(); it++)
    {
        boost::uint32_t id = *it;
        iter->seek(id);
        iter->read(one_point);
        
        boost::uint8_t* p_id = reinterpret_cast<boost::uint8_t*>(&id);
        if (indexId != -1)
        {
            one_point.setFieldData(0, indexId, p_id);
        }
        if (indexBlockId != -1)
        {
            one_point.setFieldData(0, indexBlockId, p_block_id);
        }
        // one_point.setField(0, indexId, *it);
        // one_point.setField(0, indexBlockId, block_id);
        
        // put single point onto our block
        buffer.copyPointsFast(static_cast<std::size_t>(count), static_cast<std::size_t>(0), one_point, 1); 
        buffer.setNumPoints(count + 1);

        count++;
    }
    
}



Chipper::Chipper(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
    , m_xvec(chipper::DIR_X)
    , m_yvec(chipper::DIR_Y)
    , m_spare(chipper::DIR_NONE)
{
    m_threshold = options.getValueOrThrow<boost::uint32_t>("capacity");

}


void Chipper::initialize()
{
    Filter::initialize();

    checkImpedance();
    setPointCountType(PointCount_Fixed);
    setNumPoints(0);

    return;
}


const Options Chipper::getDefaultOptions() const
{
    Options options;
    return options;
}


void Chipper::Chip()
{
    Load(m_xvec, m_yvec, m_spare);
    Partition(m_xvec.size());
    DecideSplit(m_xvec, m_yvec, m_spare, 0, m_partitions.size() - 1);
}

void Chipper::Load(RefList& xvec, RefList& yvec, RefList& spare )
{
    PtRef ref;
    boost::uint32_t idx;
    vector<PtRef>::iterator it;
   
    pdal::Schema const& schema = getPrevStage().getSchema();
    pdal::SchemaLayout const& schemaLayout(schema);
    
    boost::uint64_t count = getPrevStage().getNumPoints();
    if (count > std::numeric_limits<std::size_t>::max())
        throw pdal_error("numPoints too large for Chipper");
    boost::uint32_t count32 = (boost::uint32_t)count;
    xvec.reserve(count32);
    yvec.reserve(count32);
    spare.resize(count32);
    
    // boost::uint32_t chunks = count/m_threshold;

    const int indexX = schemaLayout.getDimensionIndex(Dimension::Id_X_i32);
    const int indexY = schemaLayout.getDimensionIndex(Dimension::Id_Y_i32);

    Dimension const& dimX = schemaLayout.getDimension(indexX);
    Dimension const& dimY = schemaLayout.getDimension(indexY);
    
    double xscale = dimX.getNumericScale();
    double yscale = dimY.getNumericScale();
    
    double xoffset = dimX.getNumericOffset();
    double yoffset = dimY.getNumericOffset();

    std::size_t num_points_loaded = 0;
    std::size_t num_points_to_load = count32;
    
    boost::scoped_ptr<StageSequentialIterator> iter(getPrevStage().createSequentialIterator());
    
    boost::uint32_t counter = 0;
    while (num_points_loaded < num_points_to_load)
    {
        boost::uint64_t num_remaining = num_points_to_load - num_points_loaded;
        boost::uint32_t num_to_read = static_cast<boost::uint32_t>(std::min<boost::uint64_t>(num_remaining, m_threshold));

        PointBuffer buffer(schema, num_to_read);
        
        
        
        boost::uint32_t num_read =  iter->read(buffer);
        
        assert(num_read <= num_to_read);

        for (boost::uint32_t j = 0; j < num_read; j++)
        {

            // (v * m_header->GetScaleX()) + m_header->GetOffsetX();
            const double x = (buffer.getField<boost::int32_t>(j, indexX) * xscale) + xoffset;
            const double y = (buffer.getField<boost::int32_t>(j, indexY) * yscale) + yoffset;
            // const double z = (buffer.getField<boost::int32_t>(j, indexZ) * zscale) + zoffset;
            ref.m_pos = x;
            ref.m_ptindex = counter;
            xvec.push_back(ref);
            
            ref.m_pos = y;
            yvec.push_back(ref);
            counter++;

        }

        num_points_loaded += num_read;
    

        if (iter->atEnd())
        {
            break;
        }
    }
    
    // while (m_reader->ReadNextPoint()) {
    //     const liblas::Point& pt = m_reader->GetPoint();
    // 
    //     ref.m_pos = pt.GetX();
    //     ref.m_ptindex = count;
    //     xvec.push_back(ref);
    // 
    //     ref.m_pos = pt.GetY();
    //     yvec.push_back(ref);
    //     count++;
    // }
    // Sort xvec and assign other index in yvec to sorted indices in xvec.
    sort(xvec.begin(), xvec.end());
    for (boost::uint32_t i = 0; i < xvec.size(); ++i) {
        idx = xvec[i].m_ptindex;
        yvec[idx].m_oindex = i;
    }

    // Sort yvec.
    sort(yvec.begin(), yvec.end());

    //Iterate through the yvector, setting the xvector appropriately.
    for (boost::uint32_t i = 0; i < yvec.size(); ++i)
        xvec[yvec[i].m_oindex].m_oindex = i;
}

void Chipper::Partition(boost::uint32_t size)
{
    boost::uint32_t num_partitions;

    num_partitions = size / m_threshold;
    if ( size % m_threshold )
        num_partitions++;
    double total = 0;
    double partition_size = static_cast<double>(size) / num_partitions;
    m_partitions.push_back(0);
    for (boost::uint32_t i = 0; i < num_partitions; ++i) {
        total += partition_size;
        boost::uint32_t itotal = static_cast<boost::uint32_t>(pdal::Utils::sround(total));
        m_partitions.push_back(itotal);
    }
}

void Chipper::DecideSplit(RefList& v1, RefList& v2, RefList& spare,
    boost::uint32_t pleft, boost::uint32_t pright)
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
        Split(v1, v2, spare, pleft, pright);
    else
        Split(v2, v1, spare, pleft, pright);
}

void Chipper::Split(RefList& wide, RefList& narrow, RefList& spare,
    boost::uint32_t pleft, boost::uint32_t pright)
{
    boost::uint32_t lstart;
    boost::uint32_t rstart;
    boost::uint32_t pcenter;
    boost::uint32_t left;
    boost::uint32_t right;
    boost::uint32_t center;

    left = m_partitions[pleft];
    right = m_partitions[pright] - 1;

    // There are two cases in which we are done.
    // 1) We have a distance of two between left and right.
    // 2) We have a distance of three between left and right.

    if (pright - pleft == 1)
        Emit(wide, left, right, narrow, left, right);
    else if (pright - pleft == 2)
        FinalSplit(wide, narrow, pleft, pright);
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
        for (boost::uint32_t i = left; i <= right; ++i)
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
        DecideSplit(wide, spare, narrow, pleft, pcenter);
        DecideSplit(wide, spare, narrow, pcenter, pright);
        narrow.m_dir = dir;
    }
}

// In this case the wide array is like we want it.  The narrow array is
// ordered, but not for our split, so we have to find the max/min entries
// for each partition in the final split.
void Chipper::FinalSplit(RefList& wide, RefList& narrow,
    boost::uint32_t pleft, boost::uint32_t pright)  
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
    boost::int64_t left = static_cast<boost::int64_t>(m_partitions[pleft]);
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
    Emit(wide, 
         static_cast<boost::uint32_t>(left), 
         static_cast<boost::uint32_t>(center - 1), 
         narrow, 
         static_cast<boost::uint32_t>(left1), 
         static_cast<boost::uint32_t>(right1) );
    Emit(wide, 
         static_cast<boost::uint32_t>(center), 
         static_cast<boost::uint32_t>(right), 
         narrow, 
         static_cast<boost::uint32_t>(left2), 
         static_cast<boost::uint32_t>(right2) );
}

void Chipper::Emit(RefList& wide, boost::uint32_t widemin, boost::uint32_t widemax,
    RefList& narrow, boost::uint32_t narrowmin, boost::uint32_t narrowmax )
{
    Block b;

    b.m_list_p = &wide;
    if (wide.m_dir == DIR_X) { 
        
        // minx, miny, maxx, maxy
        pdal::Bounds<double> bnd(wide[widemin].m_pos, narrow[narrowmin].m_pos, wide[widemax].m_pos,  narrow[narrowmax].m_pos);
        b.SetBounds(bnd);

        // b.m_xmin = wide[widemin].m_pos;
        // b.m_xmax = wide[widemax].m_pos;
        // b.m_ymin = narrow[narrowmin].m_pos;
        // b.m_ymax = narrow[narrowmax].m_pos;
    }
    else {
        pdal::Bounds<double> bnd(narrow[narrowmin].m_pos, wide[widemin].m_pos, narrow[narrowmax].m_pos, wide[widemax].m_pos);
        b.SetBounds(bnd);

        // b.m_xmin = narrow[narrowmin].m_pos;
        // b.m_xmax = narrow[narrowmax].m_pos;
        // b.m_ymin = wide[widemin].m_pos;
        // b.m_ymax = wide[widemax].m_pos;
    }
    b.m_left = widemin;
    b.m_right = widemax;
    m_blocks.push_back(b);
}


pdal::StageRandomIterator* Chipper::createRandomIterator() const
{
    return 0;
}

pdal::StageSequentialIterator* Chipper::createSequentialIterator() const
{
    return new ChipperSequentialIterator(*this);
}

void Chipper::checkImpedance()
{
    Schema& schema = getSchemaRef();

    if (!schema.hasDimension(Dimension::Id_Chipper_1)) // block id
    {
        schema.appendDimension(Dimension::Id_Chipper_1);
    }
    if (!schema.hasDimension(Dimension::Id_Chipper_2)) // point id
    {
        schema.appendDimension(Dimension::Id_Chipper_1);
    }
    
    return;
}


}} // namespace liblas::chipper
