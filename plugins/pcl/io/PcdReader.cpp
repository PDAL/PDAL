/******************************************************************************
* Copyright (c) 2014, Brad Chambers (brad.chambers@gmail.com)
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

#include "PcdReader.hpp"
#include "point_types.hpp"

#include <boost/algorithm/string.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>

#include <pdal/PointView.hpp>
#include "PCLConversions.hpp"

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.pcd",
    "Read data in the Point Cloud Library (PCL) format.",
    "http://pdal.io/stages/readers.pclvisualizer.html" );

CREATE_SHARED_PLUGIN(1, 0, PcdReader, Reader, s_info)

std::string PcdReader::getName() const { return s_info.name; }

void PcdReader::ready(PointTableRef table)
{
    pcl::PCLPointCloud2 cloud;
    pcl::PCDReader r;
    r.readHeader(m_filename, cloud);
    m_numPts = cloud.height * cloud.width;
}


void PcdReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims(getDefaultDimensions());
}


point_count_t PcdReader::read(PointViewPtr view, point_count_t /*count*/)
{
    pcl::PointCloud<XYZIRGBA>::Ptr cloud(new pcl::PointCloud<XYZIRGBA>);

    pcl::PCDReader r;
    r.read<XYZIRGBA>(m_filename, *cloud);

    pclsupport::PCDtoPDAL(*cloud, view);

    return cloud->points.size();
}

} // namespace pdal
