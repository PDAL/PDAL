#pragma once

#include "rxp2las_export.h"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cpl_conv.h>
#include <cpl_csv.h>
#include <cpl_error.h>
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <numeric>
#include <pdal/PluginManager.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <sstream>
#include <string>

namespace pdal
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
}; // namespace pdal