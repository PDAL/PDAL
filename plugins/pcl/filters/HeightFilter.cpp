/******************************************************************************
* Copyright (c) 2015, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "HeightFilter.hpp"

// c++
#include <memory>
#include <string>
#include <vector>

// project
#include "PCLConversions.hpp"
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>

// other
#include <pcl/console/print.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#define DBG log()->get(LogLevel::Debug)

namespace pdal
{

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

static PluginInfo const s_info =
    PluginInfo("filters.height", "Height Filter", "");

CREATE_SHARED_PLUGIN(1, 0, HeightFilter, Filter, s_info)

std::string HeightFilter::getName() const
{
    return s_info.name;
}

void HeightFilter::addDimensions(PointLayoutPtr layout)
{
    m_heightDim = layout->registerOrAssignDim("Height", Dimension::Type::Double);
}

void HeightFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    if (!layout->hasDim(Dimension::Id::Classification))
        throw pdal_error("HeightFilter: missing Classification dimension in input PointView");
}

void HeightFilter::filter(PointView& view)
{
    bool logOutput = log()->getLevel() > LogLevel::Debug1;
    if (logOutput)
        log()->floatPrecision(8);

    DBG << "Computing normalized heights...\n";

    BOX3D bounds;
    view.calculateBounds(bounds);

    Cloud::Ptr cloud_in(new Cloud);
    pclsupport::PDALtoPCD(std::make_shared<PointView>(view), *cloud_in, bounds);

    pcl::PointIndices::Ptr ground(new pcl::PointIndices());
    ground->indices.reserve(view.size());

    std::vector<PointId> nonground;
    nonground.reserve(view.size());

    for (PointId id = 0; id < view.size(); ++id)
    {
        double c = view.getFieldAs<double>(Dimension::Id::Classification, id);

        if (c == 2)
            ground->indices.push_back(id);
        else
            nonground.push_back(id);
    }

    if (ground->indices.size()==0)
        throw pdal_error("HeightFilter: the input PointView does not appear to have any points classified as ground");

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(ground);

    Cloud::Ptr cloud_ground(new Cloud);
    extract.setNegative(false);
    extract.filter(*cloud_ground);

    Cloud::Ptr cloud_nonground(new Cloud);
    extract.setNegative(true);
    extract.filter(*cloud_nonground);

    // project both ground and non-ground into XY plane

    // Create a set of planar coefficients with X=Y=0,Z=1
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);

    Cloud::Ptr cloud_ground_projected(new Cloud);
    proj.setInputCloud(cloud_ground);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_ground_projected);

    Cloud::Ptr cloud_nonground_projected(new Cloud);
    proj.setInputCloud(cloud_nonground);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_nonground_projected);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr ground_tree;
    ground_tree.reset(new pcl::search::KdTree<pcl::PointXYZ> (false));
    ground_tree->setInputCloud(cloud_ground_projected);

    for (size_t i = 0; i < cloud_nonground_projected->size(); ++i)
    {
        pcl::PointXYZ nonground_query = cloud_nonground_projected->points[i];
        std::vector<int> neighbors(1);
        std::vector<float> sqr_distances(1);
        ground_tree->nearestKSearch(nonground_query, 1, neighbors, sqr_distances);

        double nonground_Z = view.getFieldAs<double>(Dimension::Id::Z, nonground[i]);
        double ground_Z = view.getFieldAs<double>(Dimension::Id::Z, ground->indices[neighbors[0]]);
        double height = nonground_Z - ground_Z;

        view.setField(m_heightDim, nonground[i], height);
    }

    for (auto const& ground_idx : ground->indices)
        view.setField(m_heightDim, ground_idx, 0.0);
}


} // namespace pdal
