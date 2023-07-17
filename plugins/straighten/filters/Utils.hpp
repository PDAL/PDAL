#include <cmath>
#include <Eigen/Dense>
#include <limits>

namespace pdal {
namespace Utils {
double sqrDistToLine(double ptX, double ptY, double x1, double y1, double x2,
                     double y2, double &minDistX, double &minDistY,
                     double epsilon = 4 *
                                      std::numeric_limits<double>::epsilon());

inline bool
doubleNear(double a, double b,
           double epsilon = 4 * std::numeric_limits<double>::epsilon()) {
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
                       Eigen::Transform<Scalar, 3, Eigen::Affine> &t) {
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
} // namespace pdal
