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

#include "ApproximateCoplanarFilter.hpp"

#include <pdal/EigenUtils.hpp>
#include <pdal/KDIndex.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.approximatecoplanar",
    "Estimates the planarity of a neighborhood of points using eigenvalues.",
    "http://pdal.io/stages/filters.approximatecoplanar.html"
};

CREATE_STATIC_STAGE(ApproximateCoplanarFilter, s_info)

std::string ApproximateCoplanarFilter::getName() const
{
    return s_info.name;
}


void ApproximateCoplanarFilter::addArgs(ProgramArgs& args)
{
    args.add("knn", "k-Nearest Neighbors", m_knn, 8);
    args.add("thresh1", "Threshold 1", m_thresh1, 25.0);
    args.add("thresh2", "Threshold 2", m_thresh2, 6.0);
}


void ApproximateCoplanarFilter::addDimensions(PointLayoutPtr layout)
{
    m_coplanar = layout->registerOrAssignDim("Coplanar",
        Dimension::Type::Unsigned8);
}

void ApproximateCoplanarFilter::filter(PointView& view)
{
    using namespace Eigen;

    KD3Index& kdi = view.build3dIndex();

    for (PointId i = 0; i < view.size(); ++i)
    {
        // find the k-nearest neighbors
        auto ids = kdi.neighbors(i, m_knn);

        // compute covariance of the neighborhood
        auto B = computeCovariance(view, ids);

        // perform the eigen decomposition
        SelfAdjointEigenSolver<Matrix3d> solver(B);
        if (solver.info() != Success)
            throwError("Cannot perform eigen decomposition.");
        auto ev = solver.eigenvalues();

        // test eigenvalues to label points that are approximately coplanar
        if ((ev[1] > m_thresh1 * ev[0]) && (m_thresh2 * ev[1] > ev[2]))
            view.setField(m_coplanar, i, 1u);
        else
            view.setField(m_coplanar, i, 0u);
    }
}

} // namespace pdal
