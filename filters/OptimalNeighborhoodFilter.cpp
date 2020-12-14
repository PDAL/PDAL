/******************************************************************************
 * Copyright (c) 2020, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "OptimalNeighborhoodFilter.hpp"

#include <pdal/KDIndex.hpp>

#include <Eigen/Dense>

#include <numeric>

namespace pdal
{
using namespace Dimension;
using namespace Eigen;

static StaticPluginInfo const s_info{
    "filters.optimalneighborhood", "OptimalNeighborhood Filter",
    "http://pdal.io/stages/filters.optimalneighborhood.html"};

CREATE_STATIC_STAGE(OptimalNeighborhood, s_info)

std::string OptimalNeighborhood::getName() const
{
    return s_info.name;
}

OptimalNeighborhood::OptimalNeighborhood() : Filter() {}

void OptimalNeighborhood::addArgs(ProgramArgs& args)
{
    args.add("min_k", "Minimum k-Nearest Neighbors", m_kMin, (point_count_t)10);
    args.add("max_k", "Maximum k-Nearest Neighbors", m_kMax, (point_count_t)14);
}

void OptimalNeighborhood::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Id::OptimalKNN);
    layout->registerDim(Id::OptimalRadius);
}

void OptimalNeighborhood::filter(PointView& view)
{
    // Build the 3D KD-tree.
    const KD3Index& index = view.build3dIndex();

    for (PointRef p : view)
    {
        // find the max k-nearest neighbors
        PointIdList id3(m_kMax);
        std::vector<double> dists(m_kMax);
        index.knnSearch(p, m_kMax, &id3, &dists);

        double minentropy = (std::numeric_limits<double>::max)();
        point_count_t kopt(0);
        double ropt(0.0);
        double mx(0.0);
        double my(0.0);
        double mz(0.0);
        Matrix3d B = Matrix3d::Zero(3, 3);

        // precompute covariance matrix up to k-1 neighbors
        for (point_count_t k = 0; k < m_kMin - 1; ++k)
        {
            PointRef q = view.point(id3[k]);

            double dx = q.getFieldAs<double>(Id::X) - mx;
            double dy = q.getFieldAs<double>(Id::Y) - my;
            double dz = q.getFieldAs<double>(Id::Z) - mz;
            double n = double(k + 1);
            mx += dx / n;
            my += dy / n;
            mz += dz / n;
            double s = (n - 1) / n;
            B(0, 0) = B(0, 0) + s * dx * dx;
            B(1, 1) = B(1, 1) + s * dy * dy;
            B(2, 2) = B(2, 2) + s * dz * dz;
            B(1, 0) = B(0, 1) = B(0, 1) + s * dx * dy;
            B(2, 0) = B(0, 2) = B(0, 2) + s * dx * dz;
            B(1, 2) = B(2, 1) = B(2, 1) + s * dy * dz;
        }

        // update covariance for all k in the range [kMin, kMax], compute
        // eigenentropy and update optimal values
        for (point_count_t k = m_kMin - 1; k < m_kMax; ++k)
        {
            PointRef q = view.point(id3[k]);

            double dx = q.getFieldAs<double>(Id::X) - mx;
            double dy = q.getFieldAs<double>(Id::Y) - my;
            double dz = q.getFieldAs<double>(Id::Z) - mz;
            double n = double(k + 1);
            mx += dx / n;
            my += dy / n;
            mz += dz / n;
            double s = (n - 1) / n;
            B(0, 0) = B(0, 0) + s * dx * dx;
            B(1, 1) = B(1, 1) + s * dy * dy;
            B(2, 2) = B(2, 2) + s * dz * dz;
            B(1, 0) = B(0, 1) = B(0, 1) + s * dx * dy;
            B(2, 0) = B(0, 2) = B(0, 2) + s * dx * dz;
            B(1, 2) = B(2, 1) = B(2, 1) + s * dy * dz;

            // perform the eigen decomposition
	    Eigen::SelfAdjointEigenSolver<Matrix3d> solver(B / (n - 1));
            if (solver.info() != Eigen::Success)
                throwError("Cannot perform eigen decomposition.");
            Vector3d ev = solver.eigenvalues();

            std::vector<double> lambda = {((std::max)(ev[2], 0.0)),
                                          ((std::max)(ev[1], 0.0)),
                                          ((std::max)(ev[0], 0.0))};
            double sum = std::accumulate(lambda.begin(), lambda.end(), 0.0);

            std::transform(lambda.begin(), lambda.end(), lambda.begin(),
                           [&sum](double v) -> double { return v / sum; });

            double entropy = -(lambda[2] * std::log(lambda[2]) +
                               lambda[1] * std::log(lambda[1]) +
                               lambda[0] * std::log(lambda[0]));

            if (entropy < minentropy)
            {
                minentropy = entropy;
                kopt = k + 1;
                ropt = dists[k];
            }
        }

        p.setField(Id::OptimalKNN, kopt);
        p.setField(Id::OptimalRadius, std::sqrt(ropt));
    }
}

} // namespace pdal
