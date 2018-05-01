/******************************************************************************
* Copyright (c) 2016, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "LOFFilter.hpp"

#include <pdal/KDIndex.hpp>

#include <string>
#include <vector>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.lof",
    "LOF Filter",
    "http://pdal.io/stages/filters.lof.html"
};

CREATE_STATIC_STAGE(LOFFilter, s_info)

std::string LOFFilter::getName() const
{
    return s_info.name;
}

void LOFFilter::addArgs(ProgramArgs& args)
{
    args.add("minpts", "Minimum number of points", m_minpts, 10);
}

void LOFFilter::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;
    m_kdist = layout->registerOrAssignDim("KDistance", Type::Double);
    m_lrd = layout->registerOrAssignDim("LocalReachabilityDistance", Type::Double);
    m_lof = layout->registerOrAssignDim("LocalOutlierFactor", Type::Double);
}

void LOFFilter::filter(PointView& view)
{
    using namespace Dimension;

    // Build the 3D KD-tree.
    KD3Index index(view);
    log()->get(LogLevel::Debug) << "Building 3D KD-tree...\n";
    index.build();

    // Increment the minimum number of points, as knnSearch will be returning
    // the neighbors along with the query point.
    m_minpts++;

    // First pass: Compute the k-distance for each point.
    // The k-distance is the Euclidean distance to k-th nearest neighbor.
    log()->get(LogLevel::Debug) << "Computing k-distances...\n";
    for (PointId i = 0; i < view.size(); ++i)
    {
        std::vector<PointId> indices(m_minpts);
        std::vector<double> sqr_dists(m_minpts);
        index.knnSearch(i, m_minpts, &indices, &sqr_dists);
        view.setField(m_kdist, i, std::sqrt(sqr_dists[m_minpts-1]));
    }

    // Second pass: Compute the local reachability distance for each point.
    // For each neighbor point, the reachability distance is the maximum value
    // of that neighbor's k-distance and the distance between the neighbor and
    // the current point. The lrd is the inverse of the mean of the reachability
    // distances.
    log()->get(LogLevel::Debug) << "Computing lrd...\n";
    for (PointId i = 0; i < view.size(); ++i)
    {
        std::vector<PointId> indices(m_minpts);
        std::vector<double> sqr_dists(m_minpts);
        index.knnSearch(i, m_minpts, &indices, &sqr_dists);
        double M1 = 0.0;
        point_count_t n = 0;
        for (PointId j = 0; j < indices.size(); ++j)
        {
            double k = view.getFieldAs<double>(m_kdist, indices[j]);
            double reachdist = (std::max)(k, std::sqrt(sqr_dists[j]));
            M1 += (reachdist - M1) / ++n;
        }
        view.setField(m_lrd, i, 1.0 / M1);
    }

    // Third pass: Compute the local outlier factor for each point.
    // The LOF is the average of the lrd's for a neighborhood of points.
    log()->get(LogLevel::Debug) << "Computing LOF...\n";
    for (PointId i = 0; i < view.size(); ++i)
    {
        double lrdp = view.getFieldAs<double>(m_lrd, i);
        std::vector<PointId> indices(m_minpts);
        std::vector<double> sqr_dists(m_minpts);
        index.knnSearch(i, m_minpts, &indices, &sqr_dists);
        double M1 = 0.0;
        point_count_t n = 0;
        for (auto const& j : indices)
        {
            M1 += (view.getFieldAs<double>(m_lrd, j) / lrdp - M1) / ++n;
        }
        view.setField(m_lof, i, M1);
    }
}

} // namespace pdal
