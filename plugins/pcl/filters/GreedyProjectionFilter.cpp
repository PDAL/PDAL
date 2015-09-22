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

#include "GreedyProjectionFilter.hpp"

#include "PCLConversions.hpp"
#include "PCLPipeline.h"

#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.greedyprojection", "Grid Projection filter",
               "http://pdal.io/stages/filters.greedyprojection.html");

CREATE_SHARED_PLUGIN(1, 0, GreedyProjectionFilter, Filter, s_info)

std::string GreedyProjectionFilter::getName() const
{
    return s_info.name;
}

Options GreedyProjectionFilter::getDefaultOptions()
{
    Options options;
    // options.add("leaf_x", 1.0, "Leaf size in X dimension");
    // options.add("leaf_y", 1.0, "Leaf size in Y dimension");
    // options.add("leaf_z", 1.0, "Leaf size in Z dimension");
    return options;
}

/** \brief This method processes the PointView through the given pipeline. */

void GreedyProjectionFilter::processOptions(const Options& options)
{
    // m_leaf_x = options.getValueOrDefault<double>("leaf_x", 1.0);
    // m_leaf_y = options.getValueOrDefault<double>("leaf_y", 1.0);
    // m_leaf_z = options.getValueOrDefault<double>("leaf_z", 1.0);
}

PointViewSet GreedyProjectionFilter::run(PointViewPtr input)
{
    PointViewPtr output = input->makeNew();
    PointViewSet viewSet;
    viewSet.insert(output);

    bool logOutput = log()->getLevel() > LogLevel::Debug1;
    if (logOutput)
        log()->floatPrecision(8);

    log()->get(LogLevel::Debug2) << "Process GreedyProjectionFilter..." << std::endl;

    BOX3D buffer_bounds;
    input->calculateBounds(buffer_bounds);

    // convert PointView to PointNormal
    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    Cloud::Ptr cloud(new Cloud);
    pclsupport::PDALtoPCD(input, *cloud, buffer_bounds);

    int level = log()->getLevel();
    switch (level)
    {
        case 0:
            pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
            break;
        case 1:
            pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
            break;
        case 2:
            pcl::console::setVerbosityLevel(pcl::console::L_WARN);
            break;
        case 3:
            pcl::console::setVerbosityLevel(pcl::console::L_INFO);
            break;
        case 4:
            pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
            break;
        default:
            pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
            break;
    }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2;

    // Create search tree
    tree.reset(new pcl::search::KdTree<pcl::PointXYZ> (false));
    tree->setInputCloud(cloud);

    // Normal estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal> ());
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    // Concatenate XYZ and normal information
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // Create search tree
    tree2.reset(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // initial setup
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.setSearchRadius(0.025);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // create PointCloud for results
    pcl::PolygonMesh grid;
    gp3.reconstruct(grid);

    Cloud::Ptr cloud_f(new Cloud);
    pcl::fromPCLPointCloud2(grid.cloud, *cloud_f);

    if (cloud_f->points.empty())
    {
        log()->get(LogLevel::Debug2) << "Filtered cloud has no points!" << std::endl;
        return viewSet;
    }

    pclsupport::PCDtoPDAL(*cloud_f, output, buffer_bounds);

    log()->get(LogLevel::Debug2) << cloud->points.size() << " before, " <<
                                 cloud_f->points.size() << " after" << std::endl;
    log()->get(LogLevel::Debug2) << output->size() << std::endl;

    return viewSet;
}

} // namespace pdal
