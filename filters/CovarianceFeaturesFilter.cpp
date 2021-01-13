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
// WEAKLY SUPERVISED SEGMENTATION-AIDED CLASSIFICATION OF URBANSCENES FROM 3D
// LIDAR POINT CLOUDS Stéphane Guinard, Loïc Landrieu, 2017

#include "CovarianceFeaturesFilter.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/private/MathUtils.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <numeric>
#include <string>
#include <vector>

namespace pdal
{
using namespace Dimension;

static StaticPluginInfo const s_info
{
    "filters.covariancefeatures",
    "Filter that calculates local features based on the covariance matrix of a "
    "point's neighborhood.",
    "http://pdal.io/stages/filters.covariancefeatures.html"
};

CREATE_STATIC_STAGE(CovarianceFeaturesFilter, s_info)

std::string CovarianceFeaturesFilter::getName() const
{
    return s_info.name;
}

std::istream& operator>>(std::istream& in, CovarianceFeaturesFilter::Mode& mode)
{
    std::string s;
    in >> s;

    s = Utils::tolower(s);
    if (s == "raw")
        mode = CovarianceFeaturesFilter::Mode::Raw;
    else if (s == "sqrt")
        mode = CovarianceFeaturesFilter::Mode::SQRT;
    else if (s == "normalized")
        mode = CovarianceFeaturesFilter::Mode::Normalized;
    else
        in.setstate(std::ios_base::failbit);
    return in;
}

std::ostream& operator<<(std::ostream& out, const CovarianceFeaturesFilter::Mode& mode)
{
    switch (mode)
    {
    case CovarianceFeaturesFilter::Mode::Raw:
        out << "raw";
    case CovarianceFeaturesFilter::Mode::SQRT:
        out << "sqrt";
    case CovarianceFeaturesFilter::Mode::Normalized:
        out << "normalized";
    }
    return out;
}

void CovarianceFeaturesFilter::addArgs(ProgramArgs& args)
{
    args.add("knn", "k-Nearest neighbors", m_knn, 10);
    args.add("threads", "Number of threads used to run this filter", m_threads, 1);
    args.add("feature_set", "Set of features to be computed", m_featureSetString,
             {"dimensionality"});
    args.add("stride", "Compute features on strided neighbors", m_stride, size_t(1));
    m_radiusArg = &args.add("radius", "Radius for nearest neighbor search", m_radius);
    args.add("min_k", "Minimum number of neighbors in radius", m_minK, 3);
    args.add("mode", "Raw, normalized, or sqrt of eigenvalues", m_mode, Mode::SQRT);
    args.add("optimized", "Use OptimalKNN or OptimalRadius?", m_optimal, false);
}

void CovarianceFeaturesFilter::addDimensions(PointLayoutPtr layout)
{
    for (auto& feat : m_featureSetString)
    {
        std::string featureSet = Utils::tolower(feat);
        Utils::trim(featureSet);
        if (featureSet == "dimensionality")
            m_extraDims.insert(m_extraDims.end(),
                               {Id::Linearity, Id::Planarity, Id::Scattering,
                                Id::Verticality});
        else if (featureSet == "all")
            m_extraDims.insert(m_extraDims.end(),
                               {Id::Linearity, Id::Planarity, Id::Scattering,
                                Id::Verticality, Id::Omnivariance,
                                Id::Anisotropy, Id::Eigenentropy,
                                Id::EigenvalueSum, Id::SurfaceVariation,
                                Id::DemantkeVerticality, Id::Density});
        else if (featureSet == "linearity")
            m_extraDims.push_back(Id::Linearity);
        else if (featureSet == "planarity")
            m_extraDims.push_back(Id::Planarity);
        else if (featureSet == "scattering")
            m_extraDims.push_back(Id::Scattering);
        else if (featureSet == "verticality")
            m_extraDims.push_back(Id::Verticality);
        else if (featureSet == "omnivariance")
            m_extraDims.push_back(Id::Omnivariance);
        else if (featureSet == "anisotropy")
            m_extraDims.push_back(Id::Anisotropy);
        else if (featureSet == "eigenentropy")
            m_extraDims.push_back(Id::Eigenentropy);
        else if (featureSet == "eigenvaluesum")
            m_extraDims.push_back(Id::EigenvalueSum);
        else if (featureSet == "surfacevariation")
            m_extraDims.push_back(Id::SurfaceVariation);
        else if (featureSet == "demantkeverticality")
            m_extraDims.push_back(Id::DemantkeVerticality);
        else if (featureSet == "density")
            m_extraDims.push_back(Id::Density);
    }

    layout->registerDims(m_extraDims);
}

void CovarianceFeaturesFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    if (std::count(m_extraDims.begin(), m_extraDims.end(), Id::Density))
    {
        if (!(layout->hasDim(Id::OptimalKNN) && layout->hasDim(Id::OptimalRadius)))
            throwError("Missing OptimalKNN and OptimalRadius dimensions in input PointView.");
    }
    if (m_optimal)
    {
        if (!layout->hasDim(Id::OptimalKNN))
            throwError("Missing OptimalKNN dimension in input PointView.");
    }
}

void CovarianceFeaturesFilter::filter(PointView& view)
{
    KD3Index& kdi = view.build3dIndex();

    point_count_t nloops = view.size();
    std::vector<std::thread> threadList(m_threads);
    for(int t = 0;t<m_threads;t++)
    {
        threadList[t] = std::thread(std::bind(
                [&](const PointId start, const PointId end)
                {
                    for(PointId i = start;i<end;i++)
                        setDimensionality(view, i, kdi);
                },
                t*nloops/m_threads,(t+1)==m_threads?nloops:(t+1)*nloops/m_threads));
    }
    for (auto &t: threadList)
        t.join();
}

void CovarianceFeaturesFilter::setDimensionality(PointView &view, const PointId &id, const KD3Index &kdi)
{
    using namespace Eigen;
    
    PointRef p = view.point(id);

    // find neighbors, either by radius or k nearest neighbors
    PointIdList ids;
    if (m_optimal)
    {
        ids = kdi.neighbors(p, p.getFieldAs<uint64_t>(Id::OptimalKNN), 1);
    }
    else if (m_radiusArg->set())
    {
        ids = kdi.radius(p, m_radius);
        if (ids.size() < (size_t)m_minK)
            return;
    }
    else
    {
        ids = kdi.neighbors(p, m_knn + 1, m_stride);
    }

    // compute covariance of the neighborhood
    auto B = math::computeCovariance(view, ids);

    // perform the eigen decomposition
    SelfAdjointEigenSolver<Matrix3d> solver(B);
    if (solver.info() != Success)
        throwError("Cannot perform eigen decomposition.");

    // Extract eigenvalues and eigenvectors in decreasing order (largest eigenvalue first)
    auto ev = solver.eigenvalues();
    std::vector<double> lambda = {((std::max)(ev[2],0.0)),
                                  ((std::max)(ev[1],0.0)),
                                  ((std::max)(ev[0],0.0))};
    double sum = std::accumulate(lambda.begin(), lambda.end(), 0.0);

    if (lambda[0] == 0)
        throwError("Eigenvalues are all 0. Can't compute local features.");

    if (m_mode == Mode::SQRT)
    {
	// Gressin, Adrien, Clément Mallet, and N. David. "Improving 3d lidar
	// point cloud registration using optimal neighborhood knowledge."
	// Proceedings of ISPRS Annals of the Photogrammetry, Remote Sensing
	// and Spatial Information Sciences, Melbourne, Australia 5.111-116
	// (2012): 2.
        std::transform(lambda.begin(), lambda.end(), lambda.begin(),
                       [](double v) -> double { return std::sqrt(v); });
    }
    else if (m_mode == Mode::Normalized)
    {
        std::transform(lambda.begin(), lambda.end(), lambda.begin(),
                       [&sum](double v) -> double { return v / sum; });
    }

    auto eigenVectors = solver.eigenvectors();
    std::vector<double> v1(3), v2(3), v3(3);
    for (int i=0; i < 3; i++)
    {
        v1[i] = eigenVectors.col(2)(i);
        v2[i] = eigenVectors.col(1)(i);
        v3[i] = eigenVectors.col(0)(i);
    }

    for (auto const& dim : m_extraDims)
    {
        if (dim == Id::Linearity)
        {
            double linearity = (lambda[0] - lambda[1]) / lambda[0];
            p.setField(Id::Linearity, linearity);
        }

        if (dim == Id::Planarity)
        {
            double planarity = (lambda[1] - lambda[2]) / lambda[0];
            p.setField(Id::Planarity, planarity);
        }

        if (dim == Id::Scattering)
        {
            double scattering = lambda[2] / lambda[0];
            p.setField(Id::Scattering, scattering);
        }

        if (dim == Id::Verticality)
        {
            std::vector<double> unary_vector(3);
            double norm = 0;
            for (int i = 0; i < 3; i++)
            {
                unary_vector[i] = lambda[0] * fabs(v1[i]) +
                                  lambda[1] * fabs(v2[i]) +
                                  lambda[2] * fabs(v3[i]);
                norm += unary_vector[i] * unary_vector[i];
            }
            norm = sqrt(norm);
            p.setField(Id::Verticality, unary_vector[2] / norm);
        }

        if (dim == Id::Omnivariance)
        {
            double omnivariance = std::cbrt(lambda[2] * lambda[1] * lambda[0]);
            p.setField(Id::Omnivariance, omnivariance);
        }

        if (dim == Id::EigenvalueSum)
        {
            p.setField(Id::EigenvalueSum, sum);
        }

        if (dim == Id::Eigenentropy)
        {
            double eigenentropy = -(lambda[2] * std::log(lambda[2]) +
                                    lambda[1] * std::log(lambda[1]) +
                                    lambda[0] * std::log(lambda[0]));
            p.setField(Id::Eigenentropy, eigenentropy);
        }

        if (dim == Id::Anisotropy)
        {
            double anisotropy = (lambda[0] - lambda[2]) / lambda[0];
            p.setField(Id::Anisotropy, anisotropy);
        }

        if (dim == Id::SurfaceVariation)
        {
            double surfaceVariation = lambda[2] / sum;
            p.setField(Id::SurfaceVariation, surfaceVariation);
        }

        if (dim == Id::DemantkeVerticality)
        {
            auto e3 = solver.eigenvectors().col(0);
            double verticality = 1 - std::fabs(e3[2]);
            p.setField(Id::DemantkeVerticality, verticality);
        }

        if (dim == Id::Density)
        {
            double kopt = p.getFieldAs<double>(Id::OptimalKNN);
            double ropt = p.getFieldAs<double>(Id::OptimalRadius);
    	    double pi = 3.14159265;
            p.setField(Id::Density,
                (kopt + 1.0) / ((4.0 / 3.0) * pi * std::pow(ropt, 3)));
        }
    }
}
} // namespace pdal
