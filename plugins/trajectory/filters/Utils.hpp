//
// (c) 2022 SRI International
//
#pragma once

#include <Eigen/Dense>

namespace pdal
{
namespace trajectory
{

inline double degreesToRadians(double deg)
{
    return deg / 180.0 * M_PI;
}

inline double radiansToDegrees(double rad)
{
    return rad * 180.0 / M_PI;
}

inline Eigen::Vector2d radiansToDegrees(const Eigen::Vector2d& v)
{
    return v * 180 / M_PI;
}

// Normalize from (-M_PI, M_PI]
// std::remainder calculates a remainder in the range [-N / 2, N / 2] where N is the divisor.
inline double normalizeRadians(double r)
{
    return std::remainder(r, 2 * M_PI);
}

inline Eigen::Matrix3d PerpProjector(const Eigen::Vector3d& v, double d)
{
      // Assume v is a unit vector
      double v2 = v(0) * v(0) + v(1) * v(1);
      Eigen::Matrix3d M;
      // See proj.mac for derivation
      M(0,0) = (v(0)*v(0) * v(2) + v(1)*v(1)) / v2;
      M(1,1) = (v(1)*v(1) * v(2) + v(0)*v(0)) / v2;
      M(0,1) = -(1 - v(2)) * v(0) * v(1) / v2;
      M(1,0) = M(0,1);
      M(0,2) = -v(0);
      M(1,2) = -v(1);
      M.row(2) = v.transpose();
      // Scale first two rows to get projection error at v
      M.row(0) *= d;
      M.row(1) *= d;
      return M;
}

} // namespace trajectory
} // namespace pdal
