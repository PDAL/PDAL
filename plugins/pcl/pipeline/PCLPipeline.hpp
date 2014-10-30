/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
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

#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

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


namespace pdal
{
namespace pclsupport
{


struct tile_point_idx
{
    unsigned int tile_idx;
    unsigned int point_idx;

    tile_point_idx(unsigned int tile_idx_, unsigned int point_idx_) : tile_idx(tile_idx_), point_idx(point_idx_) {}
    bool operator < (const tile_point_idx &p) const
    {
        return (tile_idx < p.tile_idx);
    }
};


} // namespace pclsupport
} // namespace pdal


template <typename PointT> void
pcl::Pipeline<PointT>::dumper(PointCloud &cloud)
{
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        if (pcl::traits::has_xyz<PointT>::value)
        {
            std::cout << cloud.points[i].x << " "
                      << cloud.points[i].y << " "
                      << cloud.points[i].z << " ";
        }
        //if (pcl::traits::has_normal<PointT>::value)
        //{
        //    std::cout << cloud.points[i].normal[0] << " "
        //              << cloud.points[i].normal[1] << " "
        //              << cloud.points[i].normal[2] << " ";
        //}
        std::cout << std::endl;
    }
}


template <typename PointT> void
pcl::Pipeline<PointT>::generateTileIndices(PointCloudConstPtr cloud, const float& resolution, std::vector<PointIndices> &tile_indices)
{
    Eigen::Vector4f leaf_size;
    Eigen::Array4f inverse_leaf_size;
    Eigen::Vector4i min_b, max_b, div_b, divb_mul;

    leaf_size[0] = resolution;
    leaf_size[1] = resolution;
    leaf_size[2] = resolution;
    leaf_size[3] = resolution;

    inverse_leaf_size = Eigen::Array4f::Ones() / leaf_size.array();

    PCL_DEBUG("experimental, tiling with leaf size %f\n", resolution);

    Eigen::Vector4f min_p, max_p;
    // Get the minimum and maximum dimensions
    pcl::getMinMax3D<PointT> (*cloud, *indices_, min_p, max_p);

    // Check that the leaf size is not too small, given the size of the data
    int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size[0])+1;
    int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size[1])+1;

    if ((dx*dy) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
    {
        PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName().c_str());
        // is this really relevant anymore? don't think so, but maybe something like it
        //output = *input_;
        return;
    }

    // Compute the minimum and maximum bounding box values
    min_b[0] = static_cast<int>(floor(min_p[0] * inverse_leaf_size[0]));
    max_b[0] = static_cast<int>(floor(max_p[0] * inverse_leaf_size[0]));
    min_b[1] = static_cast<int>(floor(min_p[1] * inverse_leaf_size[1]));
    max_b[1] = static_cast<int>(floor(max_p[1] * inverse_leaf_size[1]));

    // Compute the number of divisions needed along all axis
    div_b = max_b - min_b + Eigen::Vector4i::Ones();
    div_b[3] = 0;

    PCL_DEBUG("%d and %d divisions in x and y\n", div_b[0], div_b[1]);

    // Set up the division multiplier
    divb_mul = Eigen::Vector4i(1, div_b[0], div_b[0] * div_b[1], 0);

    std::vector<pdal::pclsupport::tile_point_idx> index_vector;
    index_vector.reserve(indices_->size());

    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
    {
        if (!input_->is_dense)
            // Check if the point is invalid
            if (!pcl_isfinite(input_->points[*it].x) ||
                    !pcl_isfinite(input_->points[*it].y) ||
                    !pcl_isfinite(input_->points[*it].z))
                continue;

        int ijk0 = static_cast<int>(floor(input_->points[*it].x * inverse_leaf_size[0]) - static_cast<float>(min_b[0]));
        int ijk1 = static_cast<int>(floor(input_->points[*it].y * inverse_leaf_size[1]) - static_cast<float>(min_b[1]));

        // Compute the centroid leaf index
        int idx = ijk0 * divb_mul[0] + ijk1 * divb_mul[1];
        index_vector.push_back(pdal::pclsupport::tile_point_idx(static_cast<unsigned int>(idx), *it));
    }

    // Second pass: sort the index_vector vector using value representing target cell as index
    // in effect all points belonging to the same output cell will be next to each other
    std::sort(index_vector.begin(), index_vector.end(), std::less<pdal::pclsupport::tile_point_idx> ());

    // Third pass: count output cells
    // we need to skip all the same, adjacenent idx values
    unsigned int index = 0;
    // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
    // index_vector belonging to the voxel which corresponds to the i-th output point,
    // and of the first point not belonging to.
    std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
    // Worst case size
    first_and_last_indices_vector.reserve(index_vector.size());
    while (index < index_vector.size())
    {
        unsigned int i = index + 1;
        while (i < index_vector.size() && index_vector[i].tile_idx == index_vector[index].tile_idx)
            ++i;
        first_and_last_indices_vector.push_back(std::pair<unsigned int, unsigned int> (index, i));
        index = i;
    }

    tile_indices.resize(first_and_last_indices_vector.size());

    for (unsigned int cp = 0; cp < first_and_last_indices_vector.size(); ++cp)
    {
        // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
        unsigned int first_index = first_and_last_indices_vector[cp].first;
        unsigned int last_index = first_and_last_indices_vector[cp].second;

        tile_indices[cp].indices.resize(last_index - first_index);

        PCL_DEBUG("tile %d will have %d points\n", cp, last_index - first_index);

        index = 0;

        for (unsigned int i = first_index + 1; i < last_index; ++i)
        {
            tile_indices[cp].indices[index] = index_vector[i].point_idx;
            ++index;
        }
    }

    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::Pipeline<PointT>::applyPassThrough(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt)
{
    // initial setup
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);

    // parse params
    std::string field = vt.second.get<std::string> ("setFilterFieldName");
    float m1 = vt.second.get<float> ("setFilterLimits.min", -std::numeric_limits<float>::max());
    float m2 = vt.second.get<float> ("setFilterLimits.max", std::numeric_limits<float>::max());

    // summarize settings
    PCL_DEBUG("      Field name: %s\n", field.c_str());
    PCL_DEBUG("      Limits: %f, %f\n", m1, m2);

    if (field.compare("x") == 0)
    {
        if (m1 != -std::numeric_limits<float>::max()) m1 -= x_offset_;
        if (m2 != std::numeric_limits<float>::max()) m2 -= x_offset_;
    }

    if (field.compare("y") == 0)
    {
        if (m1 != -std::numeric_limits<float>::max()) m1 -= y_offset_;
        if (m2 != std::numeric_limits<float>::max()) m2 -= y_offset_;
    }

    if (field.compare("z") == 0)
    {
        if (m1 != -std::numeric_limits<float>::max()) m1 -= z_offset_;
        if (m2 != std::numeric_limits<float>::max()) m2 -= z_offset_;
    }

    // set params and apply filter
    pass.setFilterFieldName(field);
    pass.setFilterLimits(m1, m2);
    pass.filter(output);

    PCL_DEBUG("%d filtered to %d in passthrough\n", cloud->points.size(), output.points.size());

    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::Pipeline<PointT>::applyStatisticalOutlierRemoval(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt)
{
    // initial setup
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);

    // parse params
    int nr_k = vt.second.get<int> ("setMeanK", 2);
    double stddev_mult = vt.second.get<double> ("setStddevMulThresh", 0.0);

    // summarize settings
    PCL_DEBUG("      %d neighbors and %f multiplier\n", nr_k, stddev_mult);

    // set params and apply filter
    sor.setMeanK(nr_k);
    sor.setStddevMulThresh(stddev_mult);
    sor.filter(output);

    PCL_DEBUG("      %d points filtered to %d following outlier removal\n", cloud->points.size(), output.points.size());

    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::Pipeline<PointT>::applyRadiusOutlierRemoval(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt)
{
    // initial setup
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud);

    // parse params
    int min_neighbors = vt.second.get<int> ("setMinNeighborsInRadius", 2);
    double radius = vt.second.get<double> ("setRadiusSearch", 1.0);

    // summarize settings
    PCL_DEBUG("      %d neighbors and %f radius\n", min_neighbors, radius);

    // set params and apply filter
    ror.setMinNeighborsInRadius(min_neighbors);
    ror.setRadiusSearch(radius);
    ror.filter(output);

    PCL_DEBUG("      %d points filtered to %d following outlier removal\n", cloud->points.size(), output.points.size());

    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::Pipeline<PointT>::applyVoxelGrid(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt)
{
    // initial setup
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);

    // parse params
    float x = vt.second.get<float> ("setLeafSize.x", 1.0);
    float y = vt.second.get<float> ("setLeafSize.y", 1.0);
    float z = vt.second.get<float> ("setLeafSize.z", 1.0);

    // summarize settings
    PCL_DEBUG("      leaf size: %f, %f, %f\n", x, y, z);

    // set params and apply filter
    vg.setLeafSize(x, y, z);
    vg.filter(output);

    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::Pipeline<PointT>::applyGridMinimum(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt)
{
    // parse params
    float r = vt.second.get<float> ("setResolution", 1.0);

    // summarize settings
    PCL_DEBUG("      resolution: %f\n", r);

    // initial setup
    pcl::GridMinimum<PointT> vgm(r);
    vgm.setInputCloud(cloud);

    // apply filter
    vgm.filter(output);

    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::Pipeline<PointT>::applyApproximateProgressiveMorphologicalFilter(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt)
{
    pcl::ApproximateProgressiveMorphologicalFilter<PointT> pmf;

    // parse params
    int w = vt.second.get<int> ("setMaxWindowSize", 33);
    float s = vt.second.get<float> ("setSlope", 1.0);
    float md = vt.second.get<float> ("setMaxDistance", 2.5);
    float id = vt.second.get<float> ("setInitialDistance", 0.15);
    float c = vt.second.get<float> ("setCellSize", 1.0);
    float b = vt.second.get<float> ("setBase", 2.0);
    bool e = vt.second.get<bool> ("setExponential", true);
    bool n = vt.second.get<bool> ("setNegative", false);

    // summarize settings
    PCL_DEBUG("      max window size: %d\n", w);
    PCL_DEBUG("      slope: %f\n", s);
    PCL_DEBUG("      max distance: %f\n", md);
    PCL_DEBUG("      initial distance: %f\n", id);
    PCL_DEBUG("      cell size: %f\n", c);
    PCL_DEBUG("      base: %f\n", b);
    PCL_DEBUG("      exponential: %s\n", e?"true":"false");
    PCL_DEBUG("      negative: %s\n", n?"true":"false");

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

    PCL_DEBUG("      %d points filtered to %d following approximate progressive morphological filter\n", cloud->points.size(), output.points.size());

    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::Pipeline<PointT>::applyProgressiveMorphologicalFilter(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt)
{
    pcl::ProgressiveMorphologicalFilter<PointT> pmf;

    // parse params
    int w = vt.second.get<int> ("setMaxWindowSize", 33);
    float s = vt.second.get<float> ("setSlope", 1.0);
    float md = vt.second.get<float> ("setMaxDistance", 2.5);
    float id = vt.second.get<float> ("setInitialDistance", 0.15);
    float c = vt.second.get<float> ("setCellSize", 1.0);
    float b = vt.second.get<float> ("setBase", 2.0);
    bool e = vt.second.get<bool> ("setExponential", true);
    bool n = vt.second.get<bool> ("setNegative", false);

    // summarize settings
    PCL_DEBUG("      max window size: %d\n", w);
    PCL_DEBUG("      slope: %f\n", s);
    PCL_DEBUG("      max distance: %f\n", md);
    PCL_DEBUG("      initial distance: %f\n", id);
    PCL_DEBUG("      cell size: %f\n", c);
    PCL_DEBUG("      base: %f\n", b);
    PCL_DEBUG("      exponential: %s\n", e?"true":"false");
    PCL_DEBUG("      negative: %s\n", n?"true":"false");

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

    PCL_DEBUG("      %d points filtered to %d following progressive morphological filter\n", cloud->points.size(), output.points.size());

    return;
}

/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::Pipeline<PointT>::applyNormalEstimation(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt)
{
    if (pcl::traits::has_normal<PointT>::value && pcl::traits::has_curvature<PointT>::value)
    {
        // parse params
        float r = vt.second.get<float> ("setRadiusSearch", 1.0);
        float k = vt.second.get<float> ("setKSearch", 0);

        PCL_DEBUG("      radius: %f\n", r);

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        //ne.setSearchSurface (cloud); // or maybe not
        ne.setViewPoint(0.0f, 0.0f, std::numeric_limits<float>::max());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(r);
        ne.setKSearch(k);
        ne.compute(*normals);

        pcl::concatenateFields(*cloud, *normals, output);
    }
    else
    {
        PCL_ERROR("Requested point type does not support NormalEstimation...\n");
    }

    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::Pipeline<PointT>::applyConditionalRemoval(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt)
{
    typename pcl::ConditionAnd<PointT>::Ptr cond(new pcl::ConditionAnd<PointT> ());

    if (pcl::traits::has_normal<PointT>::value)
    {
        // parse params
        float m1 = vt.second.get<float> ("normalZ.min", 0);
        float m2 = vt.second.get<float> ("normalZ.max", std::numeric_limits<float>::max());

        // summarize settings
        PCL_DEBUG("      Limits: %f, %f\n", m1, m2);

        typedef typename pcl::traits::fieldList<PointT>::type FieldList;
        float min_normal_z = std::numeric_limits<float>::max();
        float max_normal_z = -std::numeric_limits<float>::max();
        for (size_t ii = 0; ii < cloud->points.size(); ++ii)
        {
            bool has_normal_z = false;
            float normal_z_val = 0.0f;
            pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<PointT, float> (cloud->points[ii], "normal_z", has_normal_z, normal_z_val));
            if (has_normal_z)
            {
                if (normal_z_val < min_normal_z) min_normal_z = normal_z_val;
                if (normal_z_val > max_normal_z) max_normal_z = normal_z_val;
            }
        }
        PCL_DEBUG("min/max normal_z [%f, %f]\n", min_normal_z, max_normal_z);

        cond->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT> ("normal_z", pcl::ComparisonOps::GT, m1)));
        cond->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT> ("normal_z", pcl::ComparisonOps::LT, m2)));
    }
    else
    {
        PCL_WARN("Requested point type does not support ConditionalRemoval by normals...\n");
    }

    pcl::ConditionalRemoval<PointT> condrem;
    condrem.setCondition(cond);
    condrem.setInputCloud(cloud);
    condrem.filter(output);

    return;
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::Pipeline<PointT>::applyMovingLeastSquares(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt)
{
        // parse params
//        float r = vt.second.get<float> ("setRadiusSearch", 1.0);
//        float k = vt.second.get<float> ("setKSearch", 0);

//        PCL_DEBUG("      radius: %f\n", r);

//        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
        pcl::MovingLeastSquares<PointT, PointT> mls;
        mls.setInputCloud(cloud);
        //mls.setSearchMethod(tree);
        mls.setSearchRadius(1);
        mls.setPolynomialFit(true);
        mls.setPolynomialOrder(2);
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
        //mls.setDilationIterations(1);
        //mls.setDilationVoxelSize(0.5);
        //mls.setPointDensity(20);
        mls.setUpsamplingRadius(2);
        mls.setUpsamplingStepSize(1);
        PCL_DEBUG("%f radius \n", mls.getUpsamplingRadius());
        PCL_DEBUG("%f step\n", mls.getUpsamplingStepSize());
        mls.process(output);

    PCL_DEBUG("%d filtered to %d in moving least squares\n", cloud->points.size(), output.points.size());

    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::Pipeline<PointT>::applyFilter(PointCloud &output)
{
    // Has the input dataset been set already?
    if (!input_)
    {
        PCL_WARN("[pcl::%s::applyFilter] No input dataset given!\n", getClassName().c_str());
        output.width = output.height = 0;
        output.points.clear();
        return;
    }

    // Do we have a JSON string to process?
    if (!json_set_)
    {
        PCL_WARN("[pcl::%s::applyFilter] No input JSON given!\n", getClassName().c_str());
        output = *input_;
        return;
    }

    output.is_dense = true;

    try
    {
        PCL_DEBUG("\n");
        PCL_DEBUG("--------------------------------------------------------------------------------\n");
        PCL_DEBUG("NAME:   %s (%s)\n", pt_.get<std::string> ("pipeline.name","").c_str(), pt_.get<std::string> ("pipeline.version","").c_str());
        PCL_DEBUG("HELP:   %s\n", pt_.get<std::string> ("pipeline.help","").c_str());
        PCL_DEBUG("AUTHOR: %s\n", pt_.get<std::string> ("pipeline.author","").c_str());
        PCL_DEBUG("--------------------------------------------------------------------------------\n");

        // generate tile indices, can use vector of PointIndices (itself a vector of indices belonging to each tile)
        std::vector<PointIndices> tile_indices;
        PointIndices foo; // okay, this is totally ugly. i just want to make sure we have a single tile of all indices if no tile_size is specified.
        foo.indices = *indices_;
        tile_indices.push_back(foo);
        float ts = pt_.get<float> ("pipeline.tile_size", 0.0f);
        if (ts > 0.0f)
            generateTileIndices(input_, ts, tile_indices);

        // loop over each tile (each PointIndices)
        #pragma omp parallel for shared(output) num_threads(4)
        for (size_t tile = 0; tile < tile_indices.size(); ++tile)
        {
            typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            typename pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>);

            // move the copyPointCloud at line 84 here, and only copy the indices belonging to the current tile (and somehow marry that with the *indices_)
            pcl::copyPointCloud<PointT> (*input_, tile_indices[tile].indices, *cloud);

            PCL_DEBUG("process tile %d through the pipeline\n", tile);

            int step = 1;

            BOOST_FOREACH(boost::property_tree::ptree::value_type &vt, pt_.get_child("pipeline.filters"))
            {
                std::string name = vt.second.get<std::string> ("name", "");
                std::string help = vt.second.get<std::string> ("help", "");

                PCL_DEBUG("\n");
                PCL_DEBUG("   Step %d) %s\n", step++, name.c_str());
                PCL_DEBUG("      %s\n", help.c_str());

                if (boost::iequals(name, "PassThrough"))
                    applyPassThrough(cloud, *cloud_f, vt);
                else if (boost::iequals(name, "StatisticalOutlierRemoval"))
                    applyStatisticalOutlierRemoval(cloud, *cloud_f, vt);
                else if (boost::iequals(name, "RadiusOutlierRemoval"))
                    applyRadiusOutlierRemoval(cloud, *cloud_f, vt);
                else if (boost::iequals(name, "VoxelGrid"))
                    applyVoxelGrid(cloud, *cloud_f, vt);
                else if (boost::iequals(name, "GridMinimum"))
                    applyGridMinimum(cloud, *cloud_f, vt);
                else if (boost::iequals(name, "ApproximateProgressiveMorphologicalFilter"))
                    applyApproximateProgressiveMorphologicalFilter(cloud, *cloud_f, vt);
                else if (boost::iequals(name, "ProgressiveMorphologicalFilter"))
                    applyProgressiveMorphologicalFilter(cloud, *cloud_f, vt);
                //else if (boost::iequals(name, "NormalEstimation"))
                //    applyNormalEstimation(cloud, *cloud_f, vt);
                //else if (boost::iequals(name, "ConditionalRemoval"))
                //    applyConditionalRemoval(cloud, *cloud_f, vt);
                else if (boost::iequals(name, "MovingLeastSquares"))
                    applyMovingLeastSquares(cloud, *cloud_f, vt);
                else
                    PCL_WARN("Requested filter `%s` not implemented! Skipping...\n", name.c_str());

                cloud.swap(cloud_f);

                if (cloud->points.size() == 0)
                {
                    PCL_WARN("No points in filtered cloud. Skipping remaining filters...\n");
                    break;
                }
            }

            // after processing each tile, we need to merge them back together into output
            #pragma omp critical
            output += *cloud;

            PCL_DEBUG("tile %d filtered to %d points, adding to output, which now has %d points\n", tile, cloud->points.size(), output.points.size());
        }

        PCL_DEBUG("\n");
    }
    catch (std::exception const& e)
    {
        PCL_WARN("[pcl::%s::applyFilterIndices] Error parsing JSON and creating pipeline! %s\n", getClassName().c_str(), e.what());
    }

    // Resize the output arrays
    output.width = static_cast<uint32_t>(output.points.size());
}

#define PCL_INSTANTIATE_Pipeline(T) template class PCL_EXPORTS pcl::Pipeline<T>;
