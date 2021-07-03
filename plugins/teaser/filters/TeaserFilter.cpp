/******************************************************************************
 * Copyright (c) 2020 Bradley J Chambers (brad.chambers@gmail.com)
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

// PDAL wrapper of https://github.com/MIT-SPARK/TEASER-plusplus, based on H.
// Yang, J. Shi, and L. Carlone, "TEASER: Fast and Certifiable Point Cloud
// Registraton," arXiv preprint, arXiv:2001.07715, 2020.

#include "TeaserFilter.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/private/MathUtils.hpp>

#include <Eigen/Dense>

#include <teaser/matcher.h>
#include <teaser/registration.h>

#include <numeric>

#define NOISE_BOUND 0.05

namespace pdal
{
using namespace Dimension;
using namespace Eigen;

static PluginInfo const s_info{"filters.teaser", "TEASER++ registration.",
                               "http://pdal.io/stages/filters.teaser.html"};

CREATE_SHARED_STAGE(TeaserFilter, s_info)

std::string TeaserFilter::getName() const
{
    return s_info.name;
}

void TeaserFilter::addArgs(ProgramArgs& args)
{
    args.add("nr", "normal radius", m_nr, 0.02);
    args.add("fr", "feature radius", m_fr, 0.04);
    args.add("fpfh", "Use FPFH to find correspondences", m_fpfh, true);
}

PointViewSet TeaserFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;
    if (m_fixed)
    {
        log()->get(LogLevel::Debug2) << "Calculating TEASER++\n";
        m_moving = view;
        teaser();
        viewSet.insert(m_moving);
        log()->get(LogLevel::Debug2) << "TEASER++ complete\n";
        m_complete = true;
    }
    else
    {
        log()->get(LogLevel::Debug2) << "Adding fixed points\n";
        m_fixed = view;
    }
    return viewSet;
}

void TeaserFilter::done(PointTableRef _)
{
    if (!m_complete)
    {
        throw pdal_error(
            "filters.teaser must have two point view inputs, no more, no less");
    }
}

Affine3d TeaserFilter::fpfh()
{
    // We probably have a lot to learn about how to drive TEASER, and some of
    // these could later be exposed as filter params, but we'll stick with the
    // values used in the majority of the published examples.
    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = NOISE_BOUND;
    params.cbar2 = 1;
    params.estimate_scaling = false;
    params.rotation_max_iterations = 100;
    params.rotation_gnc_factor = 1.4;
    params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::
        ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold = 0.005;

    // A short lambda to facilitate offsetting and scaling PointView
    // coordinates to fit within the unit cube, while creating the required
    // TEASER PointCloud.
    auto pv2pc = [&bounds = m_bounds, &scale = m_scale](PointViewPtr view) {
        teaser::PointCloud pc;
        pc.reserve(view->size());
        for (PointRef p : *view)
        {
            double x = p.getFieldAs<double>(Id::X);
            double y = p.getFieldAs<double>(Id::Y);
            double z = p.getFieldAs<double>(Id::Z);
            float fx = static_cast<float>((x - bounds.minx) / scale);
            float fy = static_cast<float>((y - bounds.miny) / scale);
            float fz = static_cast<float>((z - bounds.minz) / scale);
            pc.push_back({fx, fy, fz});
        }
        return pc;
    };

    // Convert both the fixed and moving PointViews using our lambda.
    teaser::PointCloud dst = pv2pc(m_fixed);
    teaser::PointCloud src = pv2pc(m_moving);

    // Use FPFH to generate descriptors for both the dst (fixed) and src
    // (moving) PointClouds.
    teaser::FPFHEstimation fpfh;
    auto src_desc = fpfh.computeFPFHFeatures(src, m_nr, m_fr);
    auto dst_desc = fpfh.computeFPFHFeatures(dst, m_nr, m_fr);

    // Given the FPFH descriptors and corresponding PointClouds, find
    // correspondences for TEASER.
    teaser::Matcher matcher;
    auto correspondences = matcher.calculateCorrespondences(
        src, dst, *src_desc, *dst_desc, false, true, false, 0.95);

    // Create the solver, solve, and get the solution!
    teaser::RobustRegistrationSolver solver(params);
    solver.solve(src, dst, correspondences);
    auto solution = solver.getSolution();

    // We will work with Eigen's Affine3d elsewhere, so go ahead and create
    // that based on the rotation and translation obtained from the solver.
    Affine3d T;
    T.matrix().block<3, 3>(0, 0) = solution.rotation;
    T.matrix().block<3, 1>(0, 3) = m_scale * solution.translation;
    return T;
}

Affine3d TeaserFilter::nofpfh()
{
    // TEASER without correspondences expects a 3xN Eigen matrix of doubles, so
    // we will go ahead and define Matrix3N for this.
    using Matrix3N = Matrix<double, 3, Dynamic>;

    // We probably have a lot to learn about how to drive TEASER, and some of
    // these could later be exposed as filter params, but we'll stick with the
    // values used in the majority of the published examples.
    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = NOISE_BOUND;
    params.cbar2 = 1;
    params.estimate_scaling = false;
    params.rotation_max_iterations = 100;
    params.rotation_gnc_factor = 1.4;
    params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::
        ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold = 0.005;

    // A short lambda to facilitate offsetting and scaling PointView
    // coordinates to fit within the unit cube, while creating the required
    // Matrix3N.
    auto pv2mat = [&bounds = m_bounds, &scale = m_scale](PointViewPtr view) {
        Matrix3N mat(3, view->size());
        for (PointRef p : *view)
        {
            double x = p.getFieldAs<double>(Id::X) - bounds.minx;
            double y = p.getFieldAs<double>(Id::Y) - bounds.miny;
            double z = p.getFieldAs<double>(Id::Z) - bounds.minz;
            mat.col(p.pointId()) << x, y, z;
        }
        return mat /= scale;
    };

    // Convert both the fixed and moving PointViews using our lambda.
    Matrix3N dst = pv2mat(m_fixed);
    Matrix3N src = pv2mat(m_moving);

    // Create the solver, solve, and get the solution!
    teaser::RobustRegistrationSolver solver(params);
    solver.solve(src, dst);
    auto solution = solver.getSolution();

    // We will work with Eigen's Affine3d elsewhere, so go ahead and create
    // that based on the rotation and translation obtained from the solver.
    Affine3d T;
    T.matrix().block<3, 3>(0, 0) = solution.rotation;
    T.matrix().block<3, 1>(0, 3) = m_scale * solution.translation;
    return T;
}

void TeaserFilter::teaser()
{
    using namespace std::chrono; // steady_clock, duration_cast, microseconds

    // Compute centroid of fixed PointView such that both the fixed an moving
    // PointViews can be centered.
    PointIdList ids(m_fixed->size());
    std::iota(ids.begin(), ids.end(), 0);
    Vector3d centroid = math::computeCentroid(*m_fixed, ids);

    // Compute bounds of fixed PointView. All coordinates will subsequently be
    // shifted by minimum values.
    BOX3D fixedBounds, movingBounds;
    m_fixed->calculateBounds(fixedBounds);
    m_moving->calculateBounds(movingBounds);

    m_bounds.clear();
    m_bounds.minx = std::min(fixedBounds.minx, movingBounds.minx);
    m_bounds.miny = std::min(fixedBounds.miny, movingBounds.miny);
    m_bounds.minz = std::min(fixedBounds.minz, movingBounds.minz);
    m_bounds.maxx = std::max(fixedBounds.maxx, movingBounds.maxx);
    m_bounds.maxy = std::max(fixedBounds.maxy, movingBounds.maxy);
    m_bounds.maxz = std::max(fixedBounds.maxz, movingBounds.maxz);

    // To fit our data to the unit cube, we must also determine the appropriate
    // scale by finding the max range in XYZ.
    double xrange = m_bounds.maxx - m_bounds.minx;
    double yrange = m_bounds.maxy - m_bounds.miny;
    double zrange = m_bounds.maxz - m_bounds.minz;
    m_scale = std::max(xrange, std::max(yrange, zrange));

    // hack alert
    m_bounds.minx = centroid.x();
    m_bounds.miny = centroid.y();
    m_bounds.minz = centroid.z();
    m_scale /= 2;

    // T will contain the final affine transformation.
    Affine3d T = Affine3d::Identity();

    // Choose one of two registration approaches depending on whether FPFH
    // correspondences are to be used or if TEASER will run correspondence
    // free.
    steady_clock::time_point begin = steady_clock::now();
    if (m_fpfh)
        T = fpfh();
    else
        T = nofpfh();
    steady_clock::time_point end = steady_clock::now();

    // Log results
    log()->get(LogLevel::Debug)
        << "=====================================" << std::endl;
    log()->get(LogLevel::Debug)
        << "          TEASER++ Results           " << std::endl;
    log()->get(LogLevel::Debug)
        << "=====================================" << std::endl;
    log()->get(LogLevel::Debug) << "Estimated rotation: " << std::endl;
    log()->get(LogLevel::Debug) << T.rotation() << std::endl;
    log()->get(LogLevel::Debug) << std::endl;
    log()->get(LogLevel::Debug) << "Estimated translation: " << std::endl;
    log()->get(LogLevel::Debug) << T.translation() << std::endl;
    log()->get(LogLevel::Debug)
        << "Time taken (s): "
        << duration_cast<microseconds>(end - begin).count() / 1000000.0
        << std::endl;

    // Apply the transformation to the moving PointView.
    Vector3d offsets(m_bounds.minx, m_bounds.miny, m_bounds.minz);
    for (PointRef p : *m_moving)
    {
        Vector3d coord(p.getFieldAs<double>(Id::X), p.getFieldAs<double>(Id::Y),
                       p.getFieldAs<double>(Id::Z));
        coord -= offsets;
        Vector3d q = T * coord + offsets;
        p.setField(Id::X, q.x());
        p.setField(Id::Y, q.y());
        p.setField(Id::Z, q.z());
    }

    // Compute the MSE using the fixed PointView and the transformed, moving
    // PointView.
    double mse(0.0);
    KD3Index& kdi = m_fixed->build3dIndex();
    for (PointRef p : *m_moving)
    {
        PointIdList indices(1);
        std::vector<double> sqr_dists(1);
        kdi.knnSearch(p, 1, &indices, &sqr_dists);
        double delta = std::sqrt(sqr_dists[0]) - mse;
        mse += (delta / (p.pointId() + 1));
    }
    log()->get(LogLevel::Debug2) << "MSE: " << mse << std::endl;

    // Transformation to shift coords
    Translation3d pretrans(-offsets);

    // Transformation to return to global coords
    Translation3d posttrans(offsets);

    // The composed transformation is built from right to left in order of
    // operations.
    Affine3d composed_transformation = posttrans * T * pretrans;

    Eigen::IOFormat MetadataFmt(Eigen::FullPrecision, Eigen::DontAlignCols, " ",
                                "\n", "", "", "", "");
    MetadataNode root = getMetadata();
    std::stringstream ss;
    ss << T.matrix().format(MetadataFmt);
    root.add("transform", ss.str());
    ss.str("");
    ss << composed_transformation.matrix().format(MetadataFmt);
    root.add("composed", ss.str());
    ss.str("");
    ss << offsets.format(MetadataFmt);
    root.add("centroid", ss.str());
    root.add("fitness", mse);
}

} // namespace pdal
