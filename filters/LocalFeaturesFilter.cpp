/******************************************************************************
* Copyright (c) 2019, Helix Re Inc. nicolas@helix.re
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
*     * Neither the name of Helix Re Inc. nor the
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

// This is an implementation of the local feature descriptors introduced in
// WEAKLY SUPERVISED SEGMENTATION-AIDED CLASSIFICATION OF URBANSCENES FROM 3D LIDAR POINT CLOUDS
// Stéphane Guinard, Loïc Landrieu, 2017

#include "LocalFeaturesFilter.hpp"

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
    "filters.locfeat",
    "Filter that calculates local features based on the covariance matrix",
    "http://pdal.io/stages/filters.localfeatures.html"
};

CREATE_STATIC_STAGE(LocalFeaturesFilter, s_info)

std::string LocalFeaturesFilter::getName() const
{
    return s_info.name;
}

void LocalFeaturesFilter::addArgs(ProgramArgs& args)
{
    args.add("knn", "k-Nearest neighbors", m_knn, 8);
}

void LocalFeaturesFilter::addDimensions(PointLayoutPtr layout)
{
    m_linearity = layout->registerOrAssignDim("Linearity", Dimension::Type::Float);
    m_planarity = layout->registerOrAssignDim("Planarity", Dimension::Type::Float);
    m_scattering = layout->registerOrAssignDim("Scattering", Dimension::Type::Float);
    m_verticality = layout->registerOrAssignDim("Verticality", Dimension::Type::Float);
}

void LocalFeaturesFilter::filter(PointView& view)
{
    using namespace Eigen;

    KD3Index& kdi = view.build3dIndex();

    for (PointId i = 0; i < view.size(); ++i)
    {
        // find the k-nearest neighbors
        auto ids = kdi.neighbors(i, m_knn);

        // compute covariance of the neighborhood
        auto B = eigen::computeCovariance(view, ids);

        // perform the eigen decomposition
        SelfAdjointEigenSolver<Matrix3f> solver(B);
        if (solver.info() != Success)
            throwError("Cannot perform eigen decomposition.");
        auto ev = solver.eigenvalues();

        view.setField(m_linearity, i, ev[0]);
        view.setField(m_planarity, i, ev[1]);
        view.setField(m_scattering, i, ev[2]);
        view.setField(m_verticality, i, ev[2]);
    }
}

}