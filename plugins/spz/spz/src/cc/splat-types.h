#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <array>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <vector>
#include <algorithm>
#include "splat-c-types.h"


namespace spz {

inline SpzFloatBuffer copyFloatBuffer(const std::vector<float> &vector) {
  SpzFloatBuffer buffer = {0, nullptr};
  if (!vector.empty()) {
    buffer.count = vector.size();
    buffer.data = new float[buffer.count];
    std::memcpy(buffer.data, vector.data(), buffer.count * sizeof(float));
  }
  return buffer;
}

// A point cloud composed of Gaussians. Each gaussian is represented by:
//   - xyz position
//   - xyz scales (on log scale, compute exp(x) to get scale factor)
//   - xyzw quaternion
//   - alpha (before sigmoid activation, compute sigmoid(a) to get alpha value between 0 and 1)
//   - rgb color (as SH DC component, compute 0.5 + 0.282095 * x to get color value between 0 and 1)
//   - 0 to 45 spherical harmonics coefficients (see comment below)
struct GaussianCloud {
  // Total number of points (gaussians) in this splat.
  int numPoints = 0;

  // Degree of spherical harmonics for this splat.
  int shDegree = 0;

  // Whether the gaussians should be rendered in antialiased mode (mip splatting)
  bool antialiased = false;

  // See block comment above for details
  std::vector<float> positions;
  std::vector<float> scales;
  std::vector<float> rotations;
  std::vector<float> alphas;
  std::vector<float> colors;

  // Spherical harmonics coefficients. The number of coefficients per point depends on shDegree:
  //   0 -> 0
  //   1 -> 9   (3 coeffs x 3 channels)
  //   2 -> 24  (8 coeffs x 3 channels)
  //   3 -> 45  (15 coeffs x 3 channels)
  // The color channel is the inner (fastest varying) axis, and the coefficient is the outer
  // (slower varying) axis, i.e. for degree 1, the order of the 9 values is:
  //   sh1n1_r, sh1n1_g, sh1n1_b, sh10_r, sh10_g, sh10_b, sh1p1_r, sh1p1_g, sh1p1_b
  std::vector<float> sh;

  // The caller is responsible for freeing the pointers in the returned GaussianCloudData
  GaussianCloudData data() const {
    GaussianCloudData data;
    data.numPoints = numPoints;
    data.shDegree = shDegree;
    data.antialiased = antialiased;
    data.positions = copyFloatBuffer(positions);
    data.scales = copyFloatBuffer(scales);
    data.rotations = copyFloatBuffer(rotations);
    data.alphas = copyFloatBuffer(alphas);
    data.colors = copyFloatBuffer(colors);
    data.sh = copyFloatBuffer(sh);
    return data;
  }

  // Rotates the GaussianCloud by 180 degrees about the x axis (converts from RUB to RDF coordinates
  // and vice versa. This is performed in-place.
  void rotate180DegAboutX() {
    for (size_t i = 0; i < positions.size(); i += 3) {
      positions[i+1] = -positions[i+1];
      positions[i+2] = -positions[i+2];
    }
    for (size_t i = 0; i < rotations.size(); i += 4) {
      const float x = rotations[i], y = rotations[i + 1], z = rotations[i + 2], w = rotations[i + 3];
      const float s = x < 0.0f ? -1.0f : 1.0f;
      rotations[i] = -s * w;
      rotations[i + 1] = s * z;
      rotations[i + 2] = -s * y;
      rotations[i + 3] = s * x;
    }
    // Rotate spherical harmonics by inverting coefficients that reference the y and z axes, for
    // each RGB channel. See spherical_harmonics_kernel_impl.h for spherical harmonics formulas.
    constexpr std::array<size_t, 8> coeffsToInvert = {0, 1, 3, 6, 8, 10, 11, 13};
    const size_t numCoeffs = sh.size() / 3;
    const size_t numCoeffsPerPoint = numCoeffs / numPoints;
    for (size_t i = 0; i < numCoeffs; i += numCoeffsPerPoint) {
      for (size_t j : coeffsToInvert) {
        if (j >= numCoeffsPerPoint) break;
        const size_t idx = (i + j) * 3;
        sh[idx + 0] = -sh[idx + 0];
        sh[idx + 1] = -sh[idx + 1];
        sh[idx + 2] = -sh[idx + 2];
      }
    }
  }

  float medianVolume() const {
    if (numPoints == 0) {
      return 0.01f;
    }
    // The volume of an ellipsoid is 4/3 * pi * x * y * z, where x, y, and z are the radii on each
    // axis. Scales are stored on a log scale, and exp(x) * exp(y) * exp(z) = exp(x + y + z). So we
    // can sort by value = (x + y + z) and compute volume = 4/3 * pi * exp(value) later.
    std::vector<float> scaleSums;
    for(int i = 0; i < (int)scales.size(); i += 3) {
      float sum = scales[i] + scales[i + 1] + scales[i + 2];
      scaleSums.push_back(sum);
    }
    std::sort(scaleSums.begin(), scaleSums.end());
    float median = scaleSums[(int) (scaleSums.size() / 2)];
    return (M_PI * 4 / 3) * exp(median);
  }
};

// SPZ Splat math helpers, lightweight implementations of vector and quaternion math.
using Vec3f = std::array<float, 3>;  // x, y, z
using Quat4f = std::array<float, 4>;  // w, x, y, z
using Half = uint16_t;

// Half-precision helpers.
float halfToFloat(Half h);
Half floatToHalf(float f);

// Vector helpers.
Vec3f normalized(const Vec3f &v);
float norm(const Vec3f &a);

// Quaternion helpers.
float norm(const Quat4f &q);
Quat4f normalized(const Quat4f &v);
Quat4f axisAngleQuat(const Vec3f &scaledAxis);

// Constexpr helpers.
constexpr Vec3f vec3f(const float *data) { return {data[0], data[1], data[2]}; }

constexpr float dot(const Vec3f &a, const Vec3f &b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

constexpr float squaredNorm(const Vec3f &v) { return dot(v, v); }

constexpr Quat4f quat4f(const float *data) { return {data[0], data[1], data[2], data[3]}; }

constexpr Vec3f times(const Quat4f &q, const Vec3f &p) {
  auto [w, x, y, z] = q;
  auto [vx, vy, vz] = p;
  auto x2 = x + x;
  auto y2 = y + y;
  auto z2 = z + z;
  auto wx2 = w * x2;
  auto wy2 = w * y2;
  auto wz2 = w * z2;
  auto xx2 = x * x2;
  auto xy2 = x * y2;
  auto xz2 = x * z2;
  auto yy2 = y * y2;
  auto yz2 = y * z2;
  auto zz2 = z * z2;
  return {
    vx * (1.0f - (yy2 + zz2)) + vy * (xy2 - wz2) + vz * (xz2 + wy2),
    vx * (xy2 + wz2) + vy * (1.0f - (xx2 + zz2)) + vz * (yz2 - wx2),
    vx * (xz2 - wy2) + vy * (yz2 + wx2) + vz * (1.0f - (xx2 + yy2))};
}

inline Quat4f times(const Quat4f &a, const Quat4f &b) {
  auto [w, x, y, z] = a;
  auto [qw, qx, qy, qz] = b;
  return normalized(std::array<float, 4>{
    w * qw - x * qx - y * qy - z * qz,
    w * qx + x * qw + y * qz - z * qy,
    w * qy - x * qz + y * qw + z * qx,
    w * qz + x * qy - y * qx + z * qw});
}

constexpr Quat4f times(const Quat4f &a, float s) {
  return {a[0] * s, a[1] * s, a[2] * s, a[3] * s};
}

constexpr Quat4f plus(const Quat4f &a, const Quat4f &b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]};
}

constexpr Vec3f times(const Vec3f &v, float s) { return {v[0] * s, v[1] * s, v[2] * s}; }

constexpr Vec3f plus(const Vec3f &a, const Vec3f &b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

}  // namespace spz
