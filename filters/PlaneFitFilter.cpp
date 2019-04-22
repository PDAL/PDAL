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

// PDAL implementation of the plane fit criterion presented in T. Weyrich, M.
// Pauly, R. Keiser, S. Heinzle, S. Scandella, and M. Gross, “Post-processing
// of Scanned 3D Surface Data,” Proc. Eurographics Symp.  Point-Based Graph.
// 2004, pp. 85–94, 2004.

#include "PlaneFitFilter.hpp"

#include <pdal/EigenUtils.hpp>
#include <pdal/KDIndex.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <Eigen/Dense>

#include <string>
#include <thread>
#include <vector>

namespace pdal
{

using namespace Dimension;

static StaticPluginInfo const s_info
{
    "filters.planefit",
    "Plane Fit (Kutz et al., 2003)",
    "http://pdal.io/stages/filters.planefit.html"
};

CREATE_STATIC_STAGE(PlaneFitFilter, s_info)

std::string PlaneFitFilter::getName() const
{
    return s_info.name;
}

void PlaneFitFilter::addArgs(ProgramArgs& args)
{
    args.add("knn", "k-Nearest neighbors", m_knn, 8);
    args.add("threads", "Number of threads used to run this filter", m_threads,
             1);
}

void PlaneFitFilter::addDimensions(PointLayoutPtr layout)
{
    m_planefit =
        layout->registerOrAssignDim("PlaneFit", Dimension::Type::Double);
}

void PlaneFitFilter::filter(PointView& view)
{
    KD3Index& kdi = view.build3dIndex();

    point_count_t nloops = view.size();
    std::vector<std::thread> threadPool(m_threads);
    for (int t = 0; t < m_threads; t++)
    {
        threadPool[t] = std::thread(std::bind(
            [&](const PointId start, const PointId end, const PointId t) {
                for (PointId i = start; i < end; i++)
                    setPlaneFit(view, i, kdi);
            },
            t * nloops / m_threads,
            (t + 1) == m_threads ? nloops : (t + 1) * nloops / m_threads, t));
    }
    for (auto& t : threadPool)
        t.join();
}

double PlaneFitFilter::absDistance(PointView& view, const PointId& i,
                                   Eigen::Vector3d& centroid,
                                   Eigen::Vector3f& normal)
{
    double x = view.getFieldAs<double>(Dimension::Id::X, i);
    double y = view.getFieldAs<double>(Dimension::Id::Y, i);
    double z = view.getFieldAs<double>(Dimension::Id::Z, i);
    Eigen::Vector3f p;
    p << x - centroid[0], y - centroid[1], z - centroid[2];
    double d = normal.dot(p);
    return std::fabs(d);
}

void PlaneFitFilter::setPlaneFit(PointView& view, const PointId& i,
                                 const KD3Index& kdi)
{
    // Find k-nearest neighbors of i.
    std::vector<PointId> ni = kdi.neighbors(i, m_knn + 1);

    // Normal based only on neighbors, so exclude first point.
    std::vector<PointId> neighbors(ni.begin() + 1, ni.end());

    // Covariance and normal are based off demeaned coordinates, so we record
    // the centroid to properly offset the coordinates when computing point to
    // plance distance.
    auto centroid = eigen::computeCentroid(view, neighbors);

    // Compute covariance of the neighbors.
    auto B = eigen::computeCovariance(view, neighbors);

    // Perform the eigen decomposition, using the eigenvector of the smallest
    // eigenvalue as the normal.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(B);
    if (solver.info() != Eigen::Success)
        throwError("Cannot perform eigen decomposition.");
    auto eval = solver.eigenvalues();
    Eigen::Vector3f normal = solver.eigenvectors().col(0);

    // Compute point to plane distance of the query point.
    double d = absDistance(view, i, centroid, normal);

    // Compute mean point to plane distance of neighbors.
    double d_sum(0.0);
    for (auto const& j : neighbors)
    {
        d_sum += absDistance(view, j, centroid, normal);
    }
    double d_bar(d_sum / m_knn);

    // Compute and set the plane fit criterion.
    view.setField(m_planefit, i, d / (d + d_bar));
}

} // namespace pdal
