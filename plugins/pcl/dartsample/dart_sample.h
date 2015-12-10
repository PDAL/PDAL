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

#pragma once

#include <pcl/filters/filter_indices.h>

namespace pcl
{
  /** \brief @b DartSample performs dart throwing for a given radius.
    * \author Bradley J Chambers
    * \ingroup filters
    */
  template<typename PointT>
  class DartSample : public FilterIndices<PointT>
  {
    using FilterIndices<PointT>::filter_name_;
    using FilterIndices<PointT>::getClassName;
    using FilterIndices<PointT>::indices_;
    using FilterIndices<PointT>::input_;
    using FilterIndices<PointT>::negative_;
    using FilterIndices<PointT>::user_filter_value_;
    using FilterIndices<PointT>::extract_removed_indices_;
    using FilterIndices<PointT>::removed_indices_;

    typedef typename FilterIndices<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:

      typedef boost::shared_ptr< DartSample<PointT> > Ptr;
      typedef boost::shared_ptr< const DartSample<PointT> > ConstPtr;

      /** \brief Empty constructor. */
      DartSample (bool extract_removed_indices = false) :
        FilterIndices<PointT> (extract_removed_indices),
        radius_ (1.0)
      {
        filter_name_ = "DartSample";
      }

      /** \brief Set minimum distance radius for adding points..
        * \param radius
        */
      inline void
      setRadius (double radius)
      {
        radius_ = radius;
      }

      /** \brief Get the value of the internal \a radius parameter.
        */
      inline double
      getRadius ()
      {
        return (radius_);
      }

    protected:

      /** \brief Minimum distance radius for adding points. */
      double radius_;

      /** \brief Sample of point indices into a separate PointCloud
        * \param output the resultant point cloud
        */
      void
      applyFilter (PointCloud &output);

      /** \brief Sample of point indices
        * \param indices the resultant point cloud indices
        */
      void
      applyFilter (std::vector<int> &indices);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include "dart_sample.hpp"
#endif
