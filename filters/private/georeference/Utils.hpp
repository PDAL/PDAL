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
 *     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include <Eigen/Dense>
#include <cmath>
#include <numeric>
#include <pdal/PluginManager.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <sstream>
#include <string>

namespace pdal
{
namespace georeference
{
namespace Utils
{
template <typename Scalar = double> inline Scalar rad2deg(Scalar radians)
{
    return radians * (180.0 / M_PI);
}

template <typename Scalar = double> inline Scalar deg2rad(Scalar degrees)
{
    return degrees * (M_PI / 180.0);
}

template <typename Scalar = double>
inline Scalar getValue(pdal::PointViewPtr view, pdal::Dimension::Id id,
                       uint64_t idx, double frac)
{
    return (view->getFieldAs<Scalar>(id, idx) * frac +
            view->getFieldAs<Scalar>(id, idx - 1) * (1 - frac));
}

template <typename Scalar = double>
inline Scalar getAngle(pdal::PointViewPtr view, pdal::Dimension::Id id,
                       uint64_t idx, double frac)
{
    const Scalar a2(view->getFieldAs<Scalar>(id, idx)),
        a1(view->getFieldAs<Scalar>(id, idx - 1));
    return std::atan2(frac * std::sin(a2) + (1 - frac) * std::sin(a1),
                      frac * std::cos(a2) + (1 - frac) * std::cos(a1));
}

template <typename Scalar = double>
inline Scalar getValue(pdal::PointRef p1, pdal::PointRef p2,
                       pdal::Dimension::Id id, double frac)
{
    return (p1.getFieldAs<Scalar>(id) * frac +
            p2.getFieldAs<Scalar>(id) * (1 - frac));
}

template <typename Scalar = double>
inline Scalar getValue(Scalar p1, Scalar p2, Scalar frac)
{
    return p1 * frac + p2 * (1 - frac);
}

template <typename Scalar = double>
inline Scalar getAngle(pdal::PointRef p1, pdal::PointRef p2,
                       pdal::Dimension::Id id, double frac)
{
    const Scalar a2(p2.getFieldAs<Scalar>(id)), a1(p1.getFieldAs<Scalar>(id));
    return std::atan2(frac * std::sin(a2) + (1 - frac) * std::sin(a1),
                      frac * std::cos(a2) + (1 - frac) * std::cos(a1));
}

template <typename Scalar = double>
inline Scalar getAngle(const Scalar& a1, const Scalar& a2, double frac)
{
    return std::atan2(frac * std::sin(a2) + (1 - frac) * std::sin(a1),
                      frac * std::cos(a2) + (1 - frac) * std::cos(a1));
}

template <typename Scalar = double>
inline Eigen::Transform<Scalar, 3, Eigen::Affine>
getTransformation(Scalar x, Scalar y, Scalar z, Scalar roll, Scalar pitch,
                  Scalar yaw)
{
    Eigen::Transform<Scalar, 3, Eigen::Affine> t;
    Scalar A = std::cos(yaw), B = std::sin(yaw), C = std::cos(pitch),
           D = std::sin(pitch), E = std::cos(roll), F = std::sin(roll),
           DE = D * E, DF = D * F;

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
    return t;
}
}; // namespace Utils
}; // namespace georeference
}; // namespace pdal