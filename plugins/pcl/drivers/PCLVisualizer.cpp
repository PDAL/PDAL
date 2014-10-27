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

#include "PCLVisualizer.hpp"

#include <boost/thread/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../PCLConversions.hpp"
#include <pdal/PointBuffer.hpp>
#include <pdal/StageFactory.hpp>

CREATE_WRITER_PLUGIN(pclvisualizer, pdal::drivers::pclvisualizer::PclVisualizer)

namespace pdal
{
namespace drivers
{
namespace pclvisualizer
{


void PclVisualizer::write(const PointBuffer& data)
{
    // Determine XYZ bounds
    BOX3D const& buffer_bounds = data.calculateBounds();

    // Convert PointBuffer to a PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pclsupport::PDALtoPCD(const_cast<PointBuffer&>(data), *cloud, buffer_bounds);

    // Create PCLVisualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> p(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // Set background to black
    p->setBackgroundColor(0, 0, 0);

    // Use Z dimension to colorize points
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color(cloud, "z");

    // Add point cloud to the viewer with the Z dimension color handler
    p->addPointCloud<pcl::PointXYZ> (cloud, color, "cloud");

    while (!p->wasStopped())
    {
        p->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}


}
}
} // namespaces
