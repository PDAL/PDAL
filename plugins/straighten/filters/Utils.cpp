#include "Utils.hpp"

namespace pdal {
namespace Utils {
double sqrDistToLine(double ptX, double ptY, double x1, double y1, double x2,
                     double y2, double &minDistX, double &minDistY,
                     double epsilon) {
  minDistX = x1;
  minDistY = y1;

  double dx = x2 - x1;
  double dy = y2 - y1;

  if (!doubleNear(dx, 0.0) || !doubleNear(dy, 0.0)) {
    const double t = ((ptX - x1) * dx + (ptY - y1) * dy) / (dx * dx + dy * dy);
    if (t > 1) {
      minDistX = x2;
      minDistY = y2;
    } else if (t > 0) {
      minDistX += dx * t;
      minDistY += dy * t;
    }
  }

  dx = ptX - minDistX;
  dy = ptY - minDistY;

  const double dist = dx * dx + dy * dy;

  // prevent rounding errors if the point is directly on the segment
  if (doubleNear(dist, 0.0, epsilon)) {
    minDistX = ptX;
    minDistY = ptY;
    return 0.0;
  }

  return dist;
}

double azimuth(double x1, double y1, double x2, double y2) {
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  return std::atan2(dx, dy);
}

double angularRatio(double v1, double v2, double ratio) {
  const double sin = std::sin(v2) * ratio + std::sin(v1) * (1 - ratio);
  const double cos = std::cos(v2) * ratio + std::cos(v1) * (1 - ratio);
  return std::atan2(sin, cos);
}

} // namespace Utils
} // namespace pdal