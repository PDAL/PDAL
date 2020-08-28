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

#include "IterativeClosestPoint.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/private/MathUtils.hpp>
#include <pdal/util/Utils.hpp>

#include <Eigen/Dense>

#include <numeric>

namespace pdal
{

using namespace Dimension;
using namespace Eigen;

static StaticPluginInfo const s_info
{
    "filters.icp",
    "Iterative Closest Point (ICP) registration.",
    "http://pdal.io/stages/filters.icp.html"
};

CREATE_STATIC_STAGE(IterativeClosestPoint, s_info)

std::string IterativeClosestPoint::getName() const
{
    return s_info.name;
}

void IterativeClosestPoint::addArgs(ProgramArgs& args)
{
    args.add("max_iter", "Maximum number of iterations", m_max_iters, 100);
    args.add("rt", "Rotation threshold", m_rotation_threshold,
             0.99999); // 0.256 degrees
    args.add("tt", "Translation threshold", m_translation_threshold,
             3e-4 * 3e-4); // 0.0003 meters
    args.add("mse_abs", "Absolute threshold for MSE", m_mse_abs, 1e-12);
    args.add("max_similar",
             "Max number of similar transforms to consider converged",
             m_max_similar, 0);
    m_maxdistArg =
        &args.add("max_dist", "Maximum correspondence distance", m_maxdist);
    m_matrixArg =
        &args.add("init", "Initial transformation matrix", m_matrixStr);
}

void IterativeClosestPoint::prepared(PointTableRef table)
{
    if (m_matrixArg->set())
    {
        std::stringstream matrix;
        matrix.str(m_matrixStr);
        matrix.seekg(0);
        double val;
        while (matrix >> val)
            m_vec.push_back(val);
        if (m_vec.size() != 16)
            throwError("Expecting exactly 16 values in 'init' got " +
                std::to_string(m_vec.size()));
    }
}

PointViewSet IterativeClosestPoint::run(PointViewPtr view)
{
    PointViewSet viewSet;
    if (this->m_fixed)
    {
        log()->get(LogLevel::Debug2) << "Calculating ICP\n";
        PointViewPtr result = this->icp(this->m_fixed, view);
        viewSet.insert(result);
        log()->get(LogLevel::Debug2) << "ICP complete\n";
        this->m_complete = true;
    }
    else
    {
        log()->get(LogLevel::Debug2) << "Adding fixed points\n";
        this->m_fixed = view;
    }
    return viewSet;
}

void IterativeClosestPoint::done(PointTableRef _)
{
    if (!this->m_complete)
    {
        throw pdal_error(
            "filters.icp must have two point view inputs, no more, no less");
    }
}

PointViewPtr IterativeClosestPoint::icp(PointViewPtr fixed,
                                        PointViewPtr moving) const
{
    // Compute centroid of fixed PointView such that both the fixed an moving
    // PointViews can be centered.
    PointIdList ids(fixed->size());
    std::iota(ids.begin(), ids.end(), 0);
    auto centroid = math::computeCentroid(*fixed, ids);

    // Demean the fixed and moving PointViews.
    PointViewPtr tempFixed = math::demeanPointView(*fixed, centroid.data());
    PointViewPtr tempMoving = math::demeanPointView(*moving, centroid.data());

    // Initialize the final_transformation to identity. In the future, it would
    // be reasonable to alternately accept an initial guess.
    Matrix4d final_transformation;
    if (m_matrixArg->set())
        final_transformation = Eigen::Map<const Matrix4d>(m_vec.data());
    else
        final_transformation = Matrix4d::Identity();

    // Construct 3D KD-tree of the centered, fixed PointView to facilitate
    // nearest neighbor searches in each iteration.
    KD3Index& kd_fixed = tempFixed->build3dIndex();

    // Iterate to the max number of iterations or until converged.
    bool converged(false);
    double prev_mse(0.0);
    int num_similar(0);
    for (int iter = 0; iter < m_max_iters; ++iter)
    {
        // At the beginning of each iteration, transform our centered, moving
        // PointView by the current final_transformation.
        PointViewPtr tempMovingTransformed =
            math::transform(*tempMoving, final_transformation.data());

        // Create empty lists to hold point correspondences, and initialize MSE
        // to zero.
        PointIdList fixed_idx, moving_idx;
        fixed_idx.reserve(tempMovingTransformed->size());
        moving_idx.reserve(tempMovingTransformed->size());
        double mse(0.0);
        double sqr_maxdist = m_maxdist * m_maxdist;

        // For every point in the centered, moving PointView, find the nearest
        // neighbor in the centered fixed PointView. Record the indices of each
        // and update the MSE.
        for (PointRef p : *tempMovingTransformed)
        {
            // Find the index of the nearest neighbor, and the square distance
            // between each point.
            PointIdList indices(1);
            std::vector<double> sqr_dists(1);
            kd_fixed.knnSearch(p, 1, &indices, &sqr_dists);

            // In the PCL code, there would've been a check that the square
            // distance did not exceed a threshold value.
            if (m_maxdistArg->set())
            {
                if (sqr_dists[0] > sqr_maxdist)
                    continue;
            }

            // Store the indices of the correspondence and update the MSE.
            moving_idx.push_back(p.pointId());
            fixed_idx.push_back(indices[0]);
            mse += std::sqrt(sqr_dists[0]);
        }

        // Finalize and log the MSE.
        mse /= moving_idx.size();
        log()->get(LogLevel::Debug2) << "MSE: " << mse << std::endl;

        // Estimate rigid transformation using Umeyama method, logging the
        // current translation in X and Y.
        auto A = math::pointViewToEigen(*tempFixed, fixed_idx);
        auto B = math::pointViewToEigen(*tempMovingTransformed, moving_idx);
        auto T = Eigen::umeyama(B.transpose(), A.transpose(), false);
        log()->get(LogLevel::Debug2) << "Current dx: " << T.coeff(0, 3) << ", "
                                     << "dy: " << T.coeff(1, 3) << std::endl;

        // Update the final_transformation and log the X and Y translations.
        final_transformation = final_transformation * T;
        log()->get(LogLevel::Debug2)
            << "Cumulative dx: " << final_transformation.coeff(0, 3) << ", "
            << "dy: " << final_transformation.coeff(1, 3) << std::endl;

        bool is_similar = false;

        // Compute and log the rotation and translation of the current
        // transformation (not cumulative).
        double cos_angle =
            0.5 * (T.coeff(0, 0) + T.coeff(1, 1) + T.coeff(2, 2) - 1);
        double translation_sqr = T.coeff(0, 3) * T.coeff(0, 3) +
                                 T.coeff(1, 3) * T.coeff(1, 3) +
                                 T.coeff(2, 3) * T.coeff(2, 3);
        log()->get(LogLevel::Debug2) << "Rotation: " << cos_angle << std::endl;
        log()->get(LogLevel::Debug2)
            << "Translation: " << translation_sqr << std::endl;

        // Check for change in MSE.
        if (std::fabs(mse - prev_mse) < m_mse_abs)
        {
            if (num_similar >= m_max_similar)
            {
                converged = true;
                log()->get(LogLevel::Debug2) << "converged via absolute MSE\n";
                break;
            }
            is_similar = true;
        }

        // If the rotation and translation satisfy the specified thresholds,
        // mark as converged, and exit the for loop.
        if ((cos_angle >= m_rotation_threshold) &&
            (translation_sqr <= m_translation_threshold))
        {
            if (num_similar >= m_max_similar)
            {
                converged = true;
                log()->get(LogLevel::Debug2)
                    << "converged via rotation/translation thresholds\n";
                break;
            }
            is_similar = true;
        }

        if (is_similar)
            ++num_similar;
        else
            num_similar = 0;

        prev_mse = mse;
    }

    // Apply the final_transformation to the moving PointView.
    for (PointRef p : *moving)
    {
        double x = p.getFieldAs<double>(Id::X) - centroid.x();
        double y = p.getFieldAs<double>(Id::Y) - centroid.y();
        double z = p.getFieldAs<double>(Id::Z) - centroid.z();
        p.setField(Id::X, x * final_transformation.coeff(0, 0) +
                              y * final_transformation.coeff(0, 1) +
                              z * final_transformation.coeff(0, 2) +
                              final_transformation.coeff(0, 3) + centroid.x());
        p.setField(Id::Y, x * final_transformation.coeff(1, 0) +
                              y * final_transformation.coeff(1, 1) +
                              z * final_transformation.coeff(1, 2) +
                              final_transformation.coeff(1, 3) + centroid.y());
        p.setField(Id::Z, x * final_transformation.coeff(2, 0) +
                              y * final_transformation.coeff(2, 1) +
                              z * final_transformation.coeff(2, 2) +
                              final_transformation.coeff(2, 3) + centroid.z());
    }

    // Compute the MSE one last time, using the unaltered, fixed PointView and
    // the transformed, moving PointView.
    double mse(0.0);
    KD3Index& kd_fixed_orig = fixed->build3dIndex();
    for (PointRef p : *moving)
    {
        PointIdList indices(1);
        std::vector<double> sqr_dists(1);
        kd_fixed_orig.knnSearch(p, 1, &indices, &sqr_dists);
        mse += std::sqrt(sqr_dists[0]);
    }
    mse /= moving->size();
    log()->get(LogLevel::Debug2) << "MSE: " << mse << std::endl;

    // Transformation to demean coords
    Matrix4d pretrans = Matrix4d::Identity();
    pretrans.block<3, 1>(0, 3) = -centroid;

    // Transformation to return to global coords
    Matrix4d posttrans = Matrix4d::Identity();
    posttrans.block<3, 1>(0, 3) = centroid;

    // The composed transformation is built from right to left in order of
    // operations.
    Matrix4d composed_transformation =
        posttrans * final_transformation * pretrans;

    // Populate metadata nodes to capture the final transformation, convergence
    // status, and MSE.
    Eigen::IOFormat MetadataFmt(Eigen::FullPrecision, Eigen::DontAlignCols, " ",
                                "\n", "", "", "", "");
    MetadataNode root = getMetadata();
    std::stringstream ss;
    ss << final_transformation.format(MetadataFmt);
    root.add("transform", ss.str());
    ss.str("");
    ss << composed_transformation.format(MetadataFmt);
    root.add("composed", ss.str());
    ss.str("");
    ss << centroid.format(MetadataFmt);
    root.add("centroid", ss.str());
    root.add("converged", converged);
    root.add("fitness", mse);

    return moving;
}

} // namespace pdal
