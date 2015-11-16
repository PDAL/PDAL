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

#include "dart_sample.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <vector>

#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>

///////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::DartSample<PointT>::applyFilter (PointCloud &output)
{
  std::vector<int> indices;
  output.is_dense = true;
  applyFilter (indices);
  copyPointCloud (*input_, indices, output);
}

///////////////////////////////////////////////////////////////////////////////
template<typename PointT>
void
pcl::DartSample<PointT>::applyFilter (std::vector<int> &indices)
{
  unsigned N = static_cast<unsigned> (input_->size ());

  std::srand ( std::time (NULL));
  std::vector<int> shuffled_indices = (*indices_);
  std::random_shuffle (shuffled_indices.begin (), shuffled_indices.end ());

  // Reserve N indices/removed_indices_
  indices.resize (static_cast<size_t> (N));
  if (extract_removed_indices_)
    removed_indices_->resize (static_cast<size_t> (N));

  unsigned i = 0;
  unsigned ri = 0;

  // Create octree, seeded with the first index
  pcl::octree::OctreePointCloudSearch<PointT> tree (radius_ / std::sqrt (3));
  typename pcl::PointCloud<PointT>::Ptr cloud_t (new pcl::PointCloud<PointT>);
  tree.setInputCloud (cloud_t);
  tree.addPointToCloud (input_->points[shuffled_indices[0]], cloud_t);

  // Keep the first index
  indices[i++] = shuffled_indices[0];

  // Iterate over remaining points, keeping only those meeting the minimum
  // distance criteria.
  for (auto const& j : shuffled_indices)
  {
    std::vector<int> neighbors;
    std::vector<float> sqr_distances;
    PointT temp_pt = input_->points[j];

    int num = tree.radiusSearch (temp_pt, radius_, neighbors, sqr_distances, 1);

    if (num == 0)
    {
      indices[i++] = j;
      tree.addPointToCloud (temp_pt, cloud_t);
    }
    else if (extract_removed_indices_)
    {
      (*removed_indices_)[ri++] = j;
    }
  }

  indices.resize (static_cast<size_t> (i));
  removed_indices_->resize (static_cast<size_t> (ri));
}

#define PCL_INSTANTIATE_DartSample(T) \
    template class PCL_EXPORTS pcl::DartSample<T>;
