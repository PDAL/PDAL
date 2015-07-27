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

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/impl/point_cloud_handlers.hpp>

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>

#include "PCLConversions.hpp"
#include "point_types.hpp"

#include <chrono>
#include <memory>
#include <thread>

bool
isValidFieldName(const std::string &field)
{
    if (field == "_")
        return (false);

    return (true);
}

namespace pcl
{
namespace visualization
{
template <typename PointT>
class PointCloudColorHandlerIntensity : public PointCloudColorHandler<PointT>
{
    using PointCloudColorHandler<PointT>::capable_;
    using PointCloudColorHandler<PointT>::cloud_;

    typedef typename PointCloudColorHandler<PointT>::PointCloud::ConstPtr PointCloudConstPtr;

public:
    typedef std::shared_ptr<PointCloudColorHandlerIntensity<PointT> > Ptr;
    typedef std::shared_ptr<const PointCloudColorHandlerIntensity<PointT> > ConstPtr;

    PointCloudColorHandlerIntensity(const PointCloudConstPtr& cloud) :
        PointCloudColorHandler<PointT> (cloud)
    {
        capable_ = true;
    }

    virtual bool
    getColor(vtkSmartPointer<vtkDataArray> &scalars) const
    {
        if (!capable_ || !cloud_)
            return (false);

        if (!scalars)
            scalars = vtkSmartPointer<vtkUnsignedCharArray>::New();
        scalars->SetNumberOfComponents(3);

        vtkIdType nr_points = cloud_->width * cloud_->height;
        reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples(nr_points);
        unsigned char* colors = reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->GetPointer(0);

        int int_idx = pcl::getFieldIndex(*cloud_, "intensity");

        if (int_idx != -1)
        {
            float int_data;
            int int_point_offset = cloud_->fields[int_idx].offset;
            for (vtkIdType cp = 0; cp < nr_points; ++cp, int_point_offset += cloud_->point_step)
            {
                int idx = cp * 3;
                memcpy(&int_data, &cloud_->data[int_point_offset], sizeof(float));
                colors[idx + 0] = int_data * 255;
                colors[idx + 1] = int_data * 255;
                colors[idx + 2] = int_data * 255;
            }
        }
        return (true);
    }

private:
    virtual std::string getFieldName() const
    {
        return ("intensity");
    }
    virtual inline std::string getName() const
    {
        return ("PointCloudColorHandlerIntensity");
    }
};
}
}

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.pclvisualizer",
    "PCL Visualizer",
    "http://pdal.io/stages/writers.pclvisualizer.html" );

CREATE_SHARED_PLUGIN(1, 0, PclVisualizer, Writer, s_info)

std::string PclVisualizer::getName() const { return s_info.name; }

void PclVisualizer::write(const PointViewPtr view)
{
    // Determine XYZ bounds
    BOX3D buffer_bounds;

    view->calculateBounds(buffer_bounds);

    typedef XYZIRGBA PointType;

    // Convert PointView to a PCL PointCloud
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    pclsupport::PDALtoPCD(view, *cloud, buffer_bounds);

    // Create PCLVisualizer
    std::shared_ptr<pcl::visualization::PCLVisualizer> p(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // Set background to black
    p->setBackgroundColor(0, 0, 0);

    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
    toPCLPointCloud2(*cloud, *cloud_blob);

    typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;
    typedef ColorHandler::Ptr ColorHandlerPtr;
    ColorHandlerPtr color_handler;
    for (size_t f = 0; f < cloud_blob->fields.size(); ++f)
    {
        if (cloud_blob->fields[f].name == "rgb" || cloud_blob->fields[f].name == "rgba")
        {
            color_handler.reset(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2> (cloud_blob));
        }
        else if (cloud_blob->fields[f].name == "intensity")
        {
            color_handler.reset(new pcl::visualization::PointCloudColorHandlerIntensity<pcl::PCLPointCloud2> (cloud_blob));
        }
        else
        {
            if (!isValidFieldName(cloud_blob->fields[f].name))
                continue;
            color_handler.reset(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (cloud_blob, cloud_blob->fields[f].name));
        }
        p->addPointCloud(cloud_blob, color_handler, cloud->sensor_origin_, cloud->sensor_orientation_);
    }
    p->updateColorHandlerIndex("cloud", 2);

    while (!p->wasStopped())
    {
        p->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }
}


} // namespaces
