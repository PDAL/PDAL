/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/PointBuffer.hpp>
#include <pdal/filters/Decimation.hpp>

#ifdef PDAL_HAVE_PCL
#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wfloat-equal"
#endif
#include <pdal/PCLConversions.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic pop
#endif
#endif

namespace pdal
{
namespace filters
{

Decimation::Decimation(const Options& options) : pdal::Filter(options)
{}


void Decimation::processOptions(const Options& options)
{
    m_step = options.getValueOrDefault<uint32_t>("step", 1);
    m_offset = options.getValueOrDefault<uint32_t>("offset", 0);
    m_leaf_size = options.getValueOrDefault<double>("leaf_size", 1);
    m_method = options.getValueOrDefault<std::string>("method", "RankOrder");
}


PointBufferSet Decimation::run(PointBufferPtr buffer)
{
    PointBufferSet pbSet;
    PointBufferPtr output = buffer->makeNew();
    if (boost::iequals(m_method, "VoxelGrid"))
#ifdef PDAL_HAVE_PCL
        voxel_grid(*buffer, *output);
#else
        throw pdal_error("VoxelGrid requested without PCL support");
#endif
    else if (boost::iequals(m_method, "RankOrder"))
        decimate(*buffer, *output);
    else
        throw pdal_error("decimation method unspecified");
    pbSet.insert(output);
    return pbSet;
}


void Decimation::decimate(PointBuffer& input, PointBuffer& output)
{
    for (PointId idx = m_offset; idx < input.size(); idx += m_step)
        output.appendPoint(input, idx);
}

#ifdef PDAL_HAVE_PCL
void Decimation::voxel_grid(PointBuffer& input, PointBuffer& output)
{
    BOX3D const& buffer_bounds = input.calculateBounds();

    // create PCL cloud objects
    typedef pcl::PointCloud<pcl::PointNormal> Cloud;
    Cloud::Ptr cloud(new Cloud);
    Cloud::Ptr cloud_f(new Cloud);

    // convert PointBuffer to PointCloud
    PDALtoPCD(input, *cloud, buffer_bounds);

    // apply the voxel grid
    pcl::VoxelGrid<pcl::PointNormal> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(m_leaf_size,m_leaf_size,m_leaf_size);
    vg.filter(*cloud_f);

    // and convert PointCloud back to PointBuffer
    PCDtoPDAL(*cloud_f, output, buffer_bounds);
}
#endif

} // filters
} // pdal
