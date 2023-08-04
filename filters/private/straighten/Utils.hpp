/******************************************************************************
 * Copyright (c) 2023, Guilhem Villemin (guilhem.villemin@altametris.com)
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

#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace pdal
{
namespace straighten
{
namespace Utils
{
double sqrDistToLine(double ptX, double ptY, double x1, double y1, double x2,
                     double y2, double& minDistX, double& minDistY,
                     double epsilon = 4 *
                                      std::numeric_limits<double>::epsilon());

inline bool doubleNear(double a, double b,
                       double epsilon = 4 *
                                        std::numeric_limits<double>::epsilon())
{
    const bool aIsNan = std::isnan(a);
    const bool bIsNan = std::isnan(b);
    if (aIsNan || bIsNan)
        return aIsNan && bIsNan;

    const double diff = a - b;
    return diff > -epsilon && diff <= epsilon;
}
double azimuth(double x1, double y1, double x2, double y2);
double angularRatio(double v1, double v2, double ratio);

/** \brief Create a transformation from the given translation and Euler angles
 * (XYZ-convention) \param[in] x the input x translation \param[in] y the input
 * y translation \param[in] z the input z translation \param[in] roll the input
 * roll angle \param[in] pitch the input pitch angle \param[in] yaw the input
 * yaw angle \param[out] t the resulting transformation matrix \ingroup common
 */
template <typename Scalar>
void getTransformation(Scalar x, Scalar y, Scalar z, Scalar roll, Scalar pitch,
                       Scalar yaw,
                       Eigen::Transform<Scalar, 3, Eigen::Affine>& t)
{
    Scalar A = std::cos(yaw), B = sin(yaw), C = std::cos(pitch), D = sin(pitch),
           E = std::cos(roll), F = sin(roll), DE = D * E, DF = D * F;

    t(0, 0) = A * C;
    t(0, 1) = A * DF - B * E;
    t(0, 2) = B * F + A * DE;
    t(0, 3) = x;
    t(1, 0) = B * C;
    t(1, 1) = A * E + B * DF;
    t(1, 2) = B * DE - A * F;
    t(1, 3) = y;
    t(2, 0) = -D;
    t(2, 1) = C * F;
    t(2, 2) = C * E;
    t(2, 3) = z;
    t(3, 0) = 0;
    t(3, 1) = 0;
    t(3, 2) = 0;
    t(3, 3) = 1;
}

} // namespace Utils
} // namespace straighten
} // namespace pdal
