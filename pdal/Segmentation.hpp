/******************************************************************************
 * Copyright (c) 2016-2017, Bradley J. Chambers (brad.chambers@gmail.com)
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

#include <pdal/pdal_export.hpp>
#include <pdal/pdal_types.hpp>

#include "../filters/private/DimRange.hpp"

#include <vector>

namespace pdal
{

class PointView;

namespace Segmentation
{

/**
  Extract clusters of points from input PointView.

  For each point, find neighbors within a given tolerance (Euclidean distance).
  If a neighbor already belongs to another cluster, skip it. Otherwise, add it
  to the current cluster. Recursively visit newly added cluster points, looking
  for neighbors to add to the cluster.

  \param[in] view the input PointView.
  \param[in] min_points the minimum number of points in a cluster.
  \param[in] max_points the maximum number of points in a cluster.
  \param[in] tolerance the tolerance for adding points to a cluster.
  \returns a vector of clusters (themselves vectors of PointIds).
*/
PDAL_DLL std::vector<std::vector<PointId>> extractClusters(PointView& view,
                                                           uint64_t min_points,
                                                           uint64_t max_points,
                                                           double tolerance);

PDAL_DLL void ignoreDimRange(DimRange dr, PointViewPtr input, PointViewPtr keep,
                             PointViewPtr ignore);

PDAL_DLL void segmentLastReturns(PointViewPtr input, PointViewPtr last,
                                 PointViewPtr other);

} // namespace Segmentation
} // namespace pdal
