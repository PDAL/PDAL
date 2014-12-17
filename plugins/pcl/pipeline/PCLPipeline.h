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

#include <memory>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <pcl/filters/filter_indices.h>


// Pipeline needs to be in the `pcl` namespace in order for the instantiation
// macros to work
namespace pcl
{


/** \brief @b Pipeline composes a linear series of PCL operations to be applied to the source point cloud.
* \details TBD
* <br><br>
* Usage example:
* \code
* pcl::Pipeline<PointType> pipeline (true); // Initializing with true will allow us to extract the removed indices
* pipeline.setInputCloud (cloud_in);
* pipeline.setFilename (filename);
* pipeline.filter (*cloud_out);
* \endcode
* \author Bradley J Chambers
* \ingroup pipeline
*/
template <typename PointT>
class Pipeline : public Filter<PointT>
{
protected:
    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef typename pcl::traits::fieldList<PointT>::type FieldList;

public:

    typedef boost::shared_ptr< Pipeline<PointT> > Ptr;
    typedef boost::shared_ptr< const Pipeline<PointT> > ConstPtr;


    /** \brief Constructor.
      * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
      */
    Pipeline()
    {
        filter_name_ = "Pipeline";
        filename_set_ = false;
        json_set_ = false;
    }

    /** \brief Provide the name of the JSON file describing the PCL pipeline.
      * \param[in] filename the name of the JSON file describing the PCL pipeline
      */
    inline void
    setFilename(const std::string &filename)
    {
        filename_set_ = true;
        boost::property_tree::read_json(filename.c_str(), pt_);
        json_set_ = true;
    }

    inline void
    setJSON(const std::string &json)
    {
        if (filename_set_)
            PCL_WARN("Filename and JSON string have both been specified. Using only the string!\n");
        std::stringstream ss(json);
        boost::property_tree::read_json(ss, pt_);
        json_set_ = true;
    }

    /** \brief Set the offsets to the data in the x, y, and z dimension.
      * \param[in] x the x offset
      * \param[in] y the y offset
      * \param[in] z the z offset
      */
    inline void
    setOffsets(const double x, const double y, const double z)
    {
        x_offset_ = x;
        y_offset_ = y;
        z_offset_ = z;
    }

    void dumper(PointCloud &output);

protected:
    using PCLBase<PointT>::input_;
    using PCLBase<PointT>::indices_;
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;

    /** \brief Filtered results are stored in a separate point cloud.
      * \param[out] output The resultant point cloud.
      */
    void
    applyFilter(PointCloud &output);

    /** \brief Apply passthrough filter to input cloud, using parameters specified in property tree.
     *  \param[in] cloud The input point cloud.
     *  \param[out] output The resultant point cloud.
     *  \param[in] value_type The parameters.
     */
    void
    applyPassThrough(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt);

    /** \brief Apply statistical outlier removal filter to input cloud, using parameters specified in property tree.
     *  \param[in] cloud The input point cloud.
     *  \param[out] output The resultant point cloud.
     *  \param[in] value_type The parameters.
     */
    void
    applyStatisticalOutlierRemoval(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt);

    /** \brief Apply radius outlier removal filter to input cloud, using parameters specified in property tree.
     *  \param[in] cloud The input point cloud.
     *  \param[out] output The resultant point cloud.
     *  \param[in] value_type The parameters.
     */
    void
    applyRadiusOutlierRemoval(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt);

    /** \brief Apply voxel grid filter to input cloud, using parameters specified in property tree.
     *  \param[in] cloud The input point cloud.
     *  \param[out] output The resultant point cloud.
     *  \param[in] value_type The parameters.
     */
    void
    applyVoxelGrid(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt);

    /** \brief Apply grid minimum filter to input cloud, using parameters specified in property tree.
      *  \param[in] cloud The input point cloud.
      *  \param[out] output The resultant point cloud.
      *  \param[in] value_type The parameters.
      */
    void
    applyGridMinimum(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt);

    /** \brief Apply approximate progressive morphological filter to input cloud, using parameters specified in property tree.
     *  \param[in] cloud The input point cloud.
     *  \param[out] output The resultant point cloud.
     *  \param[in] value_type The parameters.
     */
    void
    applyApproximateProgressiveMorphologicalFilter(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt);

    /** \brief Apply progressive morphological filter to input cloud, using parameters specified in property tree.
     *  \param[in] cloud The input point cloud.
     *  \param[out] output The resultant point cloud.
     *  \param[in] value_type The parameters.
     */
    void
    applyProgressiveMorphologicalFilter(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt);

    /** \brief Apply normal estimation to input cloud, using parameters specified in property tree.
     *  \param[in] cloud The input point cloud.
     *  \param[out] output The resultant point cloud.
     *  \param[in] value_type The parameters.
     */
    //void
    //applyNormalEstimation(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt);

    /** \brief Apply conditional removal filter to input cloud, using parameters specified in property tree.
     *  \param[in] cloud The input point cloud.
     *  \param[out] output The resultant point cloud.
     *  \param[in] value_type The parameters.
     */
    //void
    //applyConditionalRemoval(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt);

    /** \brief Apply moving least squares to input cloud, using parameters specified in property tree.
     *  \param[in] cloud The input point cloud.
     *  \param[out] output The resultant point cloud.
     *  \param[in] value_type The parameters.
     */
    void
    applyMovingLeastSquares(PointCloudConstPtr cloud, PointCloud &output, boost::property_tree::ptree::value_type &vt);


    /** \brief Generate tile indices for input point cloud at the given resolution.
     *  \param[in] cloud The input point cloud.
     *  \param[in] resolution The tile size (resolution).
     *  \param[out] tile_indices Vector of tile indices for each point in the cloud.
     */
    void
    generateTileIndices(PointCloudConstPtr cloud, const float &resolution, std::vector<PointIndices> &tile_indices);

private:
    bool filename_set_, json_set_;

    boost::property_tree::ptree pt_;

    /** \brief The offsets to the data in the x, y, and z dimension. */
    double x_offset_, y_offset_, z_offset_;
};


} // namespace pcl


#ifdef PCL_NO_PRECOMPILE
#include <pcl/pipeline/impl/pipeline.hpp>
#endif
