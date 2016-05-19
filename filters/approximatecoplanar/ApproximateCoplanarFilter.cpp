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

#include <pdal/Eigen.hpp>
#include <pdal/KDIndex.hpp>
#include <pdal/pdal_macros.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.approximatecoplanar", "ApproximateCoplanar Filter", 
               "http://pdal.io/stages/filters.approximatecoplanar.html");

CREATE_STATIC_PLUGIN(1, 0, ApproximateCoplanarFilter, Filter, s_info)

std::string ApproximateCoplanarFilter::getName() const
{
    return s_info.name;
}

Options ApproximateCoplanarFilter::getDefaultOptions()
{
    Options options;
    options.add("knn", 8, "k-Nearest Neighbors");
    options.add("thresh1", 25, "Threshold 1");
    options.add("thresh2", 6, "Threshold 2");
    return options;
}

void ApproximateCoplanarFilter::processOptions(const Options& options)
{
    m_knn = options.getValueOrDefault<int>("knn", 8);
    m_thresh1 = options.getValueOrDefault<double>("thresh1", 25);
    m_thresh2 = options.getValueOrDefault<double>("thresh2", 6);
}

void ApproximateCoplanarFilter::addDimensions(PointLayoutPtr layout)
{
    m_coplanar = layout->registerOrAssignDim("Coplanar", Dimension::Type::Unsigned8);
}

void ApproximateCoplanarFilter::filter(PointView& view)
{
    using namespace Eigen;

    KD3Index kdi(view);
    kdi.build();

    for (PointId i = 0; i < view.size(); ++i)
    {
        // find the k-nearest neighbors
        double x = view.getFieldAs<double>(Dimension::Id::X, i);
        double y = view.getFieldAs<double>(Dimension::Id::Y, i);
        double z = view.getFieldAs<double>(Dimension::Id::Z, i);
        auto ids = kdi.neighbors(x, y, z, m_knn);

        // compute covariance of the neighborhood
        auto B = computeCovariance(view, ids);

        // perform the eigen decomposition
        SelfAdjointEigenSolver<Matrix3f> solver(B);
        if (solver.info() != Success)
            throw pdal_error("Cannot perform eigen decomposition.");
        auto ev = solver.eigenvalues();

        // test eigenvalues to label points that are approximately coplanar
        if ((ev[1] > m_thresh1 * ev[0]) && (m_thresh2 * ev[1] > ev[2]))
            view.setField(m_coplanar, i, 1);
        else
            view.setField(m_coplanar, i, 0);
    }
}

} // namespace pdal
