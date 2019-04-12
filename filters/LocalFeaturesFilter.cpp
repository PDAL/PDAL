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
#include <cmath>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.localfeatures",
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
    args.add("knn", "k-Nearest neighbors", m_knn, 10);
    args.add("threads", "Number of threads used to run this filter", m_threads, 1);
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

    KD3Index& kdi = view.build3dIndex();

    point_count_t nloops = view.size();
    std::vector<std::thread> threadPool(m_threads);
    for(int t = 0;t<m_threads;t++)
    {
        threadPool[t] = std::thread(std::bind(
                [&](const PointId start, const PointId end, const PointId t)
                {
                    for(PointId i = start;i<end;i++)
                        setSinglePoint(view, i, kdi);
                },
                t*nloops/m_threads,(t+1)==m_threads?nloops:(t+1)*nloops/m_threads,t));
    }
    for (auto &t: threadPool)
        t.join();
}

void LocalFeaturesFilter::setSinglePoint(PointView &view, const PointId &id, const KD3Index &kdi)
{
    using namespace Eigen;

    // find the k-nearest neighbors
    auto ids = kdi.neighbors(id, m_knn + 1);

    // compute covariance of the neighborhood
    auto B = eigen::computeCovariance(view, ids);

    // perform the eigen decomposition
    SelfAdjointEigenSolver<Matrix3f> solver(B);
    if (solver.info() != Success)
        throwError("Cannot perform eigen decomposition.");

    // Extract eigenvalues and eigenvectors in decreasing order (largest eigenvalue first)
    auto ev = solver.eigenvalues();
    std::vector<float> lambda = {(std::max(ev[2],0.f)),
                                 (std::max(ev[1],0.f)),
                                 (std::max(ev[0],0.f))};

    auto eigenVectors = solver.eigenvectors();
    std::vector<float> v1(3), v2(3), v3(3);
    for (int i=0; i < 3; i++)
    {
        v1[i] = eigenVectors.col(2)(i);
        v2[i] = eigenVectors.col(1)(i);
        v3[i] = eigenVectors.col(0)(i);
    }

    float linearity  = (std::sqrtf(lambda[0]) - std::sqrtf(lambda[1])) / std::sqrtf(lambda[0]);
    float planarity  = (std::sqrtf(lambda[1]) - std::sqrtf(lambda[2])) / std::sqrtf(lambda[0]);
    float scattering =  std::sqrtf(lambda[2]) / std::sqrtf(lambda[0]);
    view.setField(m_linearity, id, linearity);
    view.setField(m_planarity, id, planarity);
    view.setField(m_scattering, id, scattering);

    std::vector<float> unary_vector(3);
    float norm = 0;
    for (int i=0; i <3 ; i++)
    {
        unary_vector[i] = lambda[0] * std::fabsf(v1[i]) + lambda[1] * std::fabsf(v2[i]) + lambda[2] * std::fabsf(v3[i]);
        norm += unary_vector[i] * unary_vector[i];
    }
    norm = std::sqrt(norm);
    view.setField(m_verticality, id, unary_vector[2] / norm);
}

}