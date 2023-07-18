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