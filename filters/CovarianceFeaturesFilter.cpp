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

#include "CovarianceFeaturesFilter.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/private/MathUtils.hpp>

#include <Eigen/Dense>

#define _USE_MATH_DEFINES  // for M_PI
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
    "Filter that calculates local features based on the covariance matrix of a point's neighborhood.",
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

std::istream& operator>>(std::istream& in, CovarianceFeaturesFilter::FeatureSet& featureset)
{
    std::string s;
    in >> s;

    s = Utils::tolower(s);
    if (s == "dimensionality")
        featureset = CovarianceFeaturesFilter::FeatureSet::Dimensionality;
    else
        in.setstate(std::ios_base::failbit);
    return in;
}

std::ostream& operator<<(std::ostream& out, const CovarianceFeaturesFilter::FeatureSet& featureset)
{
    switch (featureset)
    {
    case CovarianceFeaturesFilter::FeatureSet::Dimensionality:
        out << "dimensionality";
    }
    return out;
}


void CovarianceFeaturesFilter::addArgs(ProgramArgs& args)
{
    args.add("knn", "k-Nearest neighbors", m_knn, 10);
    args.add("threads", "Number of threads used to run this filter", m_threads, 1);
    m_featureSetArg = &args.add("feature_set", "Set of features to be computed", m_featureSet);
    args.add("stride", "Compute features on strided neighbors", m_stride, size_t(1));
    m_radiusArg = &args.add("radius", "Radius for nearest neighbor search", m_radius);
    args.add("min_k", "Minimum number of neighbors in radius", m_minK, 3);
    args.add("features", "List of features to be computed", m_features, {"all"});
    args.add("mode", "Raw, normalized, or sqrt of eigenvalues", m_mode, Mode::SQRT);
    args.add("optimized", "Use OptimalKNN or OptimalRadius?", m_optimal, false);
}

void CovarianceFeaturesFilter::addDimensions(PointLayoutPtr layout)
{
    if (m_featureSetArg->set() && (m_featureSet == FeatureSet::Dimensionality))
    {
        m_mode = Mode::SQRT;
        layout->registerDims(
            {Id::Linearity, Id::Planarity, Id::Scattering, Id::Verticality});
    }
    else
    {
        log()->get(LogLevel::Info)
            << "Feature list provided. Ignoring feature_set " << m_featureSet
            << ".\n";
        if (m_featureTypes & FeatureType::Linearity)
            layout->registerDim(Id::Linearity);
        if (m_featureTypes & FeatureType::Planarity)
            layout->registerDim(Id::Planarity);
        if (m_featureTypes & FeatureType::Scattering)
            layout->registerDim(Id::Scattering);
        if (m_featureTypes & FeatureType::Verticality)
            layout->registerDim(Id::Verticality);
        if (m_featureTypes & FeatureType::Omnivariance)
            layout->registerDim(Id::Omnivariance);
        if (m_featureTypes & FeatureType::Anisotropy)
            layout->registerDim(Id::Anisotropy);
        if (m_featureTypes & FeatureType::Eigenentropy)
            layout->registerDim(Id::Eigenentropy);
        if (m_featureTypes & FeatureType::EigenvalueSum)
            layout->registerDim(Id::EigenvalueSum);
        if (m_featureTypes & FeatureType::SurfaceVariation)
            layout->registerDim(Id::SurfaceVariation);
        if (m_featureTypes & FeatureType::DemantkeVerticality)
            layout->registerDim(Id::DemantkeVerticality);
        if (m_featureTypes & FeatureType::Density)
            layout->registerDim(Id::Density);
    }
}

void CovarianceFeaturesFilter::initialize()
{
    for (auto& feat : m_features)
    {
        std::string f = Utils::tolower(feat);
        Utils::trim(f);
        if (f == "all")
        {
            m_featureTypes = ~0;
            break;
        }
        if (f == "linearity")
            m_featureTypes |= FeatureType::Linearity;
        else if (f == "planarity")
            m_featureTypes |= FeatureType::Planarity;
        else if (f == "scattering")
            m_featureTypes |= FeatureType::Scattering;
        else if (f == "verticality")
            m_featureTypes |= FeatureType::Verticality;
        else if (f == "omnivariane")
            m_featureTypes |= FeatureType::Omnivariance;
        else if (f == "anisotropy")
            m_featureTypes |= FeatureType::Anisotropy;
        else if (f == "eigenentropy")
            m_featureTypes |= FeatureType::Eigenentropy;
        else if (f == "sum")
            m_featureTypes |= FeatureType::EigenvalueSum;
        else if (f == "surfacevariation")
            m_featureTypes |= FeatureType::SurfaceVariation;
        else if (f == "demantkeverticality")
            m_featureTypes |= FeatureType::DemantkeVerticality;
        else if (f == "density")
            m_featureTypes |= FeatureType::Density;
        else
            throwError("Invalid feature type: '" + f + "'.");
    }
}

void CovarianceFeaturesFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    if (m_optimal)
    {
        if (!layout->hasDim(Id::OptimalKNN))
            throwError("Missing OptimalKNN dimension in input PointView.");
        if (!layout->hasDim(Id::OptimalRadius))
            throwError("Missing OptimalRadius dimension in input PointView.");
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

    if (m_featureTypes & FeatureType::Linearity)
    {
        double linearity  = (lambda[0] - lambda[1]) / lambda[0];
        p.setField(Id::Linearity, linearity);
    }
    
    if (m_featureTypes & FeatureType::Planarity)
    {
        double planarity  = (lambda[1] - lambda[2]) / lambda[0];
        p.setField(Id::Planarity, planarity);
    }

    if (m_featureTypes & FeatureType::Scattering)
    {
        double scattering =  lambda[2] / lambda[0];
        p.setField(Id::Scattering, scattering);
    }

    if (m_featureTypes & FeatureType::Verticality)
    {
        std::vector<double> unary_vector(3);
        double norm = 0;
        for (int i=0; i <3 ; i++)
        {
            unary_vector[i] = lambda[0] * fabs(v1[i]) + lambda[1] * fabs(v2[i]) + lambda[2] * fabs(v3[i]);
            norm += unary_vector[i] * unary_vector[i];
        }
        norm = sqrt(norm);
        p.setField(Id::Verticality, unary_vector[2] / norm);
    }

    if (m_featureTypes & FeatureType::Omnivariance)
    {
        double omnivariance = std::cbrt(lambda[2] * lambda[1] * lambda[0]);
        p.setField(Id::Omnivariance, omnivariance);
    }

    if (m_featureTypes & FeatureType::EigenvalueSum)
    {
        p.setField(Id::EigenvalueSum, sum);
    }

    if (m_featureTypes & FeatureType::Eigenentropy)
    {
        double eigenentropy = -(lambda[2] * std::log(lambda[2]) +
                                lambda[1] * std::log(lambda[1]) +
                                lambda[0] * std::log(lambda[0]));
        p.setField(Id::Eigenentropy, eigenentropy);
    }

    if (m_featureTypes & FeatureType::Anisotropy)
    {
        double anisotropy = (lambda[0] - lambda[2]) / lambda[0];
        p.setField(Id::Anisotropy, anisotropy);
    }

    if (m_featureTypes & FeatureType::SurfaceVariation)
    {
        double surfaceVariation = lambda[2] / sum;
        p.setField(Id::SurfaceVariation, surfaceVariation);
    }

    if (m_featureTypes & FeatureType::DemantkeVerticality)
    {
        auto e3 = solver.eigenvectors().col(0);
        double verticality = 1 - std::fabs(e3[2]);
        p.setField(Id::DemantkeVerticality, verticality);
    }

    if (m_optimal && (m_featureTypes & FeatureType::Density))
    {
        double kopt = p.getFieldAs<uint64_t>(Id::OptimalKNN);
        double ropt = p.getFieldAs<double>(Id::OptimalRadius);
        p.setField(Id::Density,
                   (kopt + 1) / ((4 / 3) * M_PI * std::pow(ropt, 3)));
    }
}
} // namespace pdal
