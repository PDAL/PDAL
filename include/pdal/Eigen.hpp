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

#pragma once

#include <pdal/pdal_internal.hpp>
#include <pdal/util/Bounds.hpp>

#include <Eigen/Dense>

#include <vector>

namespace pdal
{
class PointView;

/**
 * \brief Compute the centroid of a collection of points.
 *
 * Computes the 3D centroid of a collection of points (specified by PointId)
 * sampled from the input PointView.
 *
 * \code
 * // build 3D kd-tree
 * KD3Index kdi(view);
 * kdi.build();
 *
 * // find the k-nearest neighbors of the first point (k=8)
 * double x = view.getFieldAs<double>(Dimension::Id::X, 0);
 * double y = view.getFieldAs<double>(Dimension::Id::Y, 0);
 * double z = view.getFieldAs<double>(Dimension::Id::Z, 0);
 * auto ids = kdi.neighbors(x, y, z, 8);
 *
 * // compute the centroid
 * auto centroid = computeCentroid(view, ids);
 * \endcode
 *
 * \param view the source PointView.
 * \param ids a vector of PointIds specifying a subset of points.
 * \return the 3D centroid of the XYZ dimensions.
 */
PDAL_DLL Eigen::Vector3f computeCentroid(PointView& view, std::vector<PointId> ids);

/**
 * \brief Compute the covariance matrix of a collection of points.
 *
 * Computes the covariance matrix of a collection of points (specified by
 * PointId) sampled from the input PointView.
 *
 * \code
 * // build 3D kd-tree
 * KD3Index kdi(view);
 * kdi.build();
 *
 * // find the k-nearest neighbors of the first point (k=8)
 * double x = view.getFieldAs<double>(Dimension::Id::X, 0);
 * double y = view.getFieldAs<double>(Dimension::Id::Y, 0);
 * double z = view.getFieldAs<double>(Dimension::Id::Z, 0);
 * auto ids = kdi.neighbors(x, y, z, 8);
 *
 * // compute the covariance
 * auto cov = computeCovariance(view, ids);
 * \endcode
 *
 * \param view the source PointView.
 * \param ids a vector of PointIds specifying a subset of points.
 * \return the covariance matrix of the XYZ dimensions.
 */
PDAL_DLL Eigen::Matrix3f computeCovariance(PointView& view, std::vector<PointId> ids);

/**
 * \brief Compute the rank of a collection of points.
 *
 * Computes the rank of a collection of points (specified by PointId) sampled
 * from the input PointView. This method uses Eigen's JacobiSVD class to solve
 * the singular value decomposition and to estimate the rank using the given
 * threshold. A singular value will be considered nonzero if its absolute value
 * is greater than the product of the user-supplied threshold and the absolute
 * value of the maximum singular value.
 *
 * More on JacobiSVD can be found at
 * https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html.
 *
 * \code
 * // build 3D kd-tree
 * KD3Index kdi(view);
 * kdi.build();
 *
 * // find the k-nearest neighbors of the first point (k=8)
 * double x = view.getFieldAs<double>(Dimension::Id::X, 0);
 * double y = view.getFieldAs<double>(Dimension::Id::Y, 0);
 * double z = view.getFieldAs<double>(Dimension::Id::Z, 0);
 * auto ids = kdi.neighbors(x, y, z, 8);
 *
 * // compute the rank using threshold of 0.01
 * auto rank = computeRank(view, ids, 0.01);
 * \endcode
 *
 * \param view the source PointView.
 * \param ids a vector of PointIds specifying a subset of points.
 * \return the estimated rank.
 */
PDAL_DLL uint8_t computeRank(PointView& view, std::vector<PointId> ids, double threshold);

// createDSM returns a matrix with minimum Z values from the provided
// PointView.
PDAL_DLL Eigen::MatrixXd createDSM(PointView& view, int rows, int cols,
                                   double cell_size, BOX2D bounds);
} // namespace pdal
