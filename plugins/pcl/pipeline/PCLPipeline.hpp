/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014-2016, RadiantBlue Technologies, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#pragma once

#include <exception>

#include <pcl/for_each_type.h>
#include <pcl/point_traits.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/surface/mls.h>

#include "PCLPipeline.h"

#include <pdal/util/Utils.hpp>

#include <nlohmann/json.hpp>

using namespace pdal;


template <typename PointT> void
pcl::Pipeline<PointT>::applyPassThrough(PointCloudConstPtr cloud,
    PointCloud &output, const NL::json& vt)
{
    // initial setup
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);

    // parse params
    std::string field = vt["setFilterFieldName"].get<std::string>();
    float m1 = vt["setFilterLimits"].value("min", -(std::numeric_limits<float>::max)());
    float m2 = vt["setFilterLimits"].value("max", (std::numeric_limits<float>::max)());

    // summarize settings
    PCL_DEBUG("\tField name: %s\n", field.c_str());
    PCL_DEBUG("\tLimits: %f, %f\n", m1, m2);

    if (field.compare("x") == 0)
    {
        if (m1 != -(std::numeric_limits<float>::max)())
            m1 -= x_offset_;
        if (m2 != (std::numeric_limits<float>::max)())
            m2 -= x_offset_;
    }

    if (field.compare("y") == 0)
    {
        if (m1 != -(std::numeric_limits<float>::max)())
            m1 -= y_offset_;
        if (m2 != (std::numeric_limits<float>::max)())
            m2 -= y_offset_;
    }

    if (field.compare("z") == 0)
    {
        if (m1 != -(std::numeric_limits<float>::max)())
            m1 -= z_offset_;
        if (m2 != (std::numeric_limits<float>::max)())
            m2 -= z_offset_;
    }

    // set params and apply filter
    pass.setFilterFieldName(field);
    pass.setFilterLimits(m1, m2);
    pass.filter(output);

    PCL_DEBUG("\t%d filtered to %d in passthrough\n", cloud->points.size(),
        output.points.size());

    return;
}

template <typename PointT> void
pcl::Pipeline<PointT>::applyStatisticalOutlierRemoval(PointCloudConstPtr cloud,
    PointCloud &output, const NL::json& vt)
{
    // initial setup
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);

    // parse params
    int nr_k = vt.value("setMeanK", 2);
    double stddev_mult = vt.value("setStddevMulThresh", 0.0);

    // summarize settings
    PCL_DEBUG("\t%d neighbors and %f multiplier\n", nr_k, stddev_mult);

    // set params and apply filter
    sor.setMeanK(nr_k);
    sor.setStddevMulThresh(stddev_mult);
    sor.filter(output);

    PCL_DEBUG("\t%d points filtered to %d following outlier removal\n",
        cloud->points.size(), output.points.size());

    return;
}

template <typename PointT> void
pcl::Pipeline<PointT>::applyRadiusOutlierRemoval(PointCloudConstPtr cloud,
    PointCloud &output, const NL::json& vt)
{
    // initial setup
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud);

    // parse params
    int min_neighbors = vt.value("setMinNeighborsInRadius", 2);
    double radius = vt.value("setRadiusSearch", 1.0);

    // summarize settings
    PCL_DEBUG("\t%d neighbors and %f radius\n", min_neighbors, radius);

    // set params and apply filter
    ror.setMinNeighborsInRadius(min_neighbors);
    ror.setRadiusSearch(radius);
    ror.filter(output);

    PCL_DEBUG("\t%d points filtered to %d following outlier removal\n",
        cloud->points.size(), output.points.size());

    return;
}

template <typename PointT> void
pcl::Pipeline<PointT>::applyVoxelGrid(PointCloudConstPtr cloud,
    PointCloud &output, const NL::json& vt)
{
    // initial setup
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);

    // parse params
    float x = vt["setLeafSize"].value("x", 1.0f);
    float y = vt["setLeafSize"].value("y", 1.0f);
    float z = vt["setLeafSize"].value("z", 1.0f);

    // summarize settings
    PCL_DEBUG("\tleaf size: %f, %f, %f\n", x, y, z);

    // set params and apply filter
    vg.setLeafSize(x, y, z);
    vg.filter(output);

    return;
}

template <typename PointT> void
pcl::Pipeline<PointT>::applyGridMinimum(PointCloudConstPtr cloud,
    PointCloud &output, const NL::json& vt)
{
    // parse params
    float r = vt.value("setResolution", 1.0f);

    // summarize settings
    PCL_DEBUG("\tresolution: %f\n", r);

    // initial setup
    pcl::GridMinimum<PointT> vgm(r);
    vgm.setInputCloud(cloud);

    // apply filter
    vgm.filter(output);

    return;
}

template <typename PointT> void
pcl::Pipeline<PointT>::applyApproximateProgressiveMorphologicalFilter(
    PointCloudConstPtr cloud, PointCloud &output, const NL::json& vt)
{
    pcl::ApproximateProgressiveMorphologicalFilter<PointT> pmf;

    // parse params
    int w = vt.value("setMaxWindowSize", 33);
    float s = vt.value("setSlope", 1.0f);
    float md = vt.value("setMaxDistance", 2.5f);
    float id = vt.value("setInitialDistance", 0.15f);
    float c = vt.value("setCellSize", 1.0f);
    float b = vt.value("setBase", 2.0f);
    bool e = vt.value("setExponential", true);
    bool n = vt.value("setNegative", false);

    // summarize settings
    PCL_DEBUG("\tmax window size: %d\n", w);
    PCL_DEBUG("\tslope: %f\n", s);
    PCL_DEBUG("\tmax distance: %f\n", md);
    PCL_DEBUG("\tinitial distance: %f\n", id);
    PCL_DEBUG("\tcell size: %f\n", c);
    PCL_DEBUG("\tbase: %f\n", b);
    PCL_DEBUG("\texponential: %s\n", e?"true":"false");
    PCL_DEBUG("\tnegative: %s\n", n?"true":"false");

    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(w);
    pmf.setSlope(s);
    pmf.setMaxDistance(md);
    pmf.setInitialDistance(id);
    pmf.setCellSize(c);
    pmf.setBase(b);
    pmf.setExponential(e);

    PointIndicesPtr idx(new PointIndices);
    pmf.extract(idx->indices);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(idx);
    extract.setNegative(n);
    extract.filter(output);

    PCL_DEBUG("\t%d points filtered to %d following approximate PMF\n",
        cloud->points.size(), output.points.size());

    return;
}

template <typename PointT> void
pcl::Pipeline<PointT>::applyProgressiveMorphologicalFilter(
    PointCloudConstPtr cloud, PointCloud &output, const NL::json& vt)
{
    pcl::ProgressiveMorphologicalFilter<PointT> pmf;

    // parse params
    int w = vt.value("setMaxWindowSize", 33);
    float s = vt.value("setSlope", 1.0f);
    float md = vt.value("setMaxDistance", 2.5f);
    float id = vt.value("setInitialDistance", 0.15f);
    float c = vt.value("setCellSize", 1.0f);
    float b = vt.value("setBase", 2.0f);
    bool e = vt.value("setExponential", true);
    bool n = vt.value("setNegative", false);

    // summarize settings
    PCL_DEBUG("\tmax window size: %d\n", w);
    PCL_DEBUG("\tslope: %f\n", s);
    PCL_DEBUG("\tmax distance: %f\n", md);
    PCL_DEBUG("\tinitial distance: %f\n", id);
    PCL_DEBUG("\tcell size: %f\n", c);
    PCL_DEBUG("\tbase: %f\n", b);
    PCL_DEBUG("\texponential: %s\n", e?"true":"false");
    PCL_DEBUG("\tnegative: %s\n", n?"true":"false");

    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(w);
    pmf.setSlope(s);
    pmf.setMaxDistance(md);
    pmf.setInitialDistance(id);
    pmf.setCellSize(c);
    pmf.setBase(b);
    pmf.setExponential(e);

    PointIndicesPtr idx(new PointIndices);
    pmf.extract(idx->indices);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(idx);
    extract.setNegative(n);
    extract.filter(output);

    PCL_DEBUG("\t%d points filtered to %d following PMF\n",
        cloud->points.size(), output.points.size());

    return;
}

template <typename PointT> void
pcl::Pipeline<PointT>::applyFilter(PointCloud &output)
{
    // Has the input dataset been set already?
    if (!input_)
    {
        PCL_WARN("[pcl::%s::applyFilter] No input dataset given!\n",
            getClassName().c_str());
        output.width = output.height = 0;
        output.points.clear();
        return;
    }

    // Do we have a JSON string to process?
    if (!json_set_)
    {
        PCL_WARN("[pcl::%s::applyFilter] No input JSON given!\n",
            getClassName().c_str());
        output = *input_;
        return;
    }

    output.is_dense = true;

    try
    {
        typedef pcl::PointCloud<PointT> Cloud;
        typename Cloud::Ptr cloud(new Cloud);
        typename Cloud::Ptr cloud_f(new Cloud);

        pcl::copyPointCloud<PointT> (*input_, *cloud);

        for (auto const& vt : pt_)
        {
            std::string name(vt.value("name", ""));

            using namespace Utils;

            if (iequals(name, "PassThrough"))
                applyPassThrough(cloud, *cloud_f, vt);
            else if (iequals(name, "StatisticalOutlierRemoval"))
                applyStatisticalOutlierRemoval(cloud, *cloud_f, vt);
            else if (iequals(name, "RadiusOutlierRemoval"))
                applyRadiusOutlierRemoval(cloud, *cloud_f, vt);
            else if (iequals(name, "VoxelGrid"))
                applyVoxelGrid(cloud, *cloud_f, vt);
            else if (iequals(name, "GridMinimum"))
                applyGridMinimum(cloud, *cloud_f, vt);
            else if (iequals(name, "ApproximateProgressiveMorphologicalFilter"))
                applyApproximateProgressiveMorphologicalFilter(cloud, *cloud_f,
                    vt);
            else if (iequals(name, "ProgressiveMorphologicalFilter"))
                applyProgressiveMorphologicalFilter(cloud, *cloud_f, vt);
            else
                PCL_WARN("Requested filter `%s` not implemented! Skipping...\n",
                    name.c_str());

            cloud.swap(cloud_f);

            if (cloud->points.size() == 0)
            {
                PCL_WARN("No points in filtered cloud. Aborting!\n");
                break;
            }
        }

        output += *cloud;
    }
    catch (std::exception const& e)
    {
        PCL_WARN("[pcl::%s] Error parsing JSON and creating pipeline! %s\n",
            getClassName().c_str(), e.what());
    }

    // Resize the output arrays
    output.width = static_cast<uint32_t>(output.points.size());
}

#define PCL_INSTANTIATE_Pipeline(T) template class PCL_EXPORTS pcl::Pipeline<T>;
