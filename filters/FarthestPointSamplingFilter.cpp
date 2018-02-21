/******************************************************************************
 * Copyright (c) 2019, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "FarthestPointSamplingFilter.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <algorithm>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace pdal
{

static PluginInfo const s_info{"filters.fps", "Farthest point sampling filter",
                               "http://pdal.io/stages/filters.fps.html"};

CREATE_STATIC_STAGE(FarthestPointSamplingFilter, s_info)

std::string FarthestPointSamplingFilter::getName() const
{
    return s_info.name;
}

FarthestPointSamplingFilter::FarthestPointSamplingFilter()
{
}

void FarthestPointSamplingFilter::addArgs(ProgramArgs& args)
{
    args.add("count", "Target number of points after sampling", m_count,
             point_count_t(1000));
}

PointViewSet FarthestPointSamplingFilter::run(PointViewPtr inView)
{
    // Return empty PointViewSet if the input PointView has no points.
    PointViewSet viewSet;
    if (!inView->size() || (inView->size() < m_count))
        return viewSet;

    // Otherwise, make a new output PointView.
    PointViewPtr outView = inView->makeNew();

    // Construct a KD-tree of the input view.
    KD3Index kdi(*inView);
    kdi.build();

    // Seed the output view with the first point in the current sorting.
    PointId seedId(0);
    outView->appendPoint(*inView, seedId);

    // Compute distances from seedId to all other points.
    PointIdList indices(inView->size());
    std::vector<double> sqr_dists(inView->size());
    kdi.knnSearch(seedId, inView->size(), &indices, &sqr_dists);

    // Sort distances by PointId.
    std::vector<double> min_dists(inView->size());
    for (PointId i = 0; i < inView->size(); ++i)
        min_dists[indices[i]] = sqr_dists[i];

    // Proceed until we have m_count points in the output PointView.
    for (PointId i = 1; i < m_count; ++i)
    {
        // Find the max distance in min_dists, this is the farthest point from
        // any point currently in the output PointView.
        auto it = std::max_element(min_dists.begin(), min_dists.end());

        // Record the PointId of the farthest point and add it to the output
        // PointView.
        PointId idx(it - min_dists.begin());
        outView->appendPoint(*inView, idx);

        log()->get(LogLevel::Debug)
            << "Adding PointId " << idx << " with distance "
            << std::sqrt(min_dists[idx]) << std::endl;

        // Compute distances from idx to all other points.
        kdi.knnSearch(idx, inView->size(), &indices, &sqr_dists);

        // Update distances.
        for (PointId j = 0; j < inView->size(); ++j)
        {
            if (sqr_dists[j] < min_dists[indices[j]])
                min_dists[indices[j]] = sqr_dists[j];
        }
    }

    viewSet.insert(outView);
    return viewSet;
}

} // namespace pdal
