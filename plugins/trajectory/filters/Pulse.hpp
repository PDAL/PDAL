#pragma once

#include <cmath>
#include <limits>
#include <Eigen/Dense>

#include "Utils.hpp"
#include "SplineFit.hpp"

namespace pdal
{
namespace trajectory
{

class Pulse
{
    // Data on one pulse of the lidar.  If multiple returns, gives the
    // midpoint, r, of the first/last returns and the unit vector, n, from last
    // to first.  d is the distance from the midpoint to the first return.  If
    // d = 0 (i.e., it's a single return, use n to give sweep vector for a
    // scan-angle pulse).
  public:
    double t;
    Eigen::Vector3d r;
    Eigen::Vector3d n;
    double d;
    double ang;

    Pulse()
      : t(0)
      , r(Eigen::Vector3d::Zero())
      , n(Eigen::Vector3d::Zero())
      , d(0)
      , ang(0)
    {}

    Pulse(double _t, const Eigen::Vector3d& _r, double _ang = 0) :
        t(_t), r(_r), n(Eigen::Vector3d::Zero()), d(0), ang(degreesToRadians(_ang))
    {}
    Pulse(double _t, const Eigen::Vector3d& _f, const Eigen::Vector3d& _l, double _ang = 0) :
        t(_t), r((_f + _l) / 2), ang(degreesToRadians(_ang))
    {
        const Eigen::Vector3d& diff = _f - _l;
        n = diff.normalized();
        d = diff.norm() / 2;
    }

    bool operator<(const Pulse& p) const
    { return t < p.t; }

    bool MultiReturn() const
    { return d > 0; }

};

inline std::ostream& operator<<(std::ostream& out, const Pulse& p)
{
    out << p.d << "/" << p.t << "/" << p.r(0) << "," << p.r(1) << "," << p.r(2) << "/" <<
        p.n(0) << "," << p.n(1) << "," << p.n(2) << "/" << p.d << "/" << p.ang;
    return out;
}

class FirstLastError
{
public:
    FirstLastError(const Pulse& p, double t)
      : _t(t)                     // fractional time in block
      , _r(p.r)
      , _M(PerpProjector(p.n, p.d))
    {}

private:
    double _t;
    const Eigen::Vector3d _r;
    Eigen::Matrix3d _M;

public:
    template <typename T>
    bool operator()(const T* const rm, // 3 vec for pos at beg
                    const T* const vm, // 3 vec for vel at beg
                    const T* const rp, // 3 vec for pos at end
                    const T* const vp, // 3 vec for vel at end
                    // 2 residuals
                    T* residuals) const
    {
        const T t = T(_t);
        const T r[3] = { T(_r(0)), T(_r(1)), T(_r(2)) };
        T p[3];                              // Platform relative to return

        for (int i = 0; i < 3; ++i)
            p[i] = SplineFitScalar::EndPointCubic(rm[i], vm[i], rp[i], vp[i], t) - r[i];

      // Transform
      T xt = T(_M(0,0)) * p[0] + T(_M(0,1)) * p[1] + T(_M(0,2)) * p[2];
      T yt = T(_M(1,0)) * p[0] + T(_M(1,1)) * p[1] + T(_M(1,2)) * p[2];
      T zt = T(_M(2,0)) * p[0] + T(_M(2,1)) * p[1] + T(_M(2,2)) * p[2];

      // Project
      residuals[0] = xt / zt;
      residuals[1] = yt / zt;
      return true;
    }
};

//  Attitude error.
class ScanAngleError
{
private:
    double _t, _ang, _pitch, _pitchweight;
    const Eigen::Vector3d _r;
    bool _fixedpitch;

public:
    ScanAngleError(const Pulse& p, double t, double pitchweight = 1,
        double pitch = std::numeric_limits<double>::quiet_NaN()) :
      _t(t)                   // fractional time in block
      , _ang(p.ang)
      , _pitch(pitch)
      , _pitchweight(pitchweight)
      , _r(p.r)
      , _fixedpitch(!std::isnan(_pitch))  //true only if pitch is not a NaN
    {}

    template <typename T>
    bool operator()(const T* const rm, // 3 vec for pos at beg
                    const T* const vm, // 3 vec for vel at beg
                    const T* const rp, // 3 vec for pos at end
                    const T* const vp, // 3 vec for vel at end
                    const T* const am, // 2 vec for attitude  at beg
                    const T* const bm, // 2 vec for attitude' at beg
                    const T* const ap, // 2 vec for attitude  at end
                    const T* const bp, // 2 vec for attitude' at end
                    // 2 residuals
                    T* residuals) const
    {
        using std::sin; using std::cos;

        // Only use two components (yaw, pitch) of attitude roll is effectively
        // zero (definition of scan angle includes the roll of the platform).
        const T t = T(_t);
        const T ang = T(_ang);
        const T pitch = T(_pitch);
        const T pitchweight = T(_pitchweight);

        T d0[3];                  // Platform direction
        T m[3] { T(_r(0)), T(_r(1)), T(_r(2)) };  //midpoint of the return

        //d0: three vector looking from the plane down to the ground,
        // directed upward, in world coordinate
        for (int i = 0; i < 3; ++i)
            d0[i] = SplineFitScalar::EndPointCubic(rm[i], vm[i], rp[i], vp[i], t) - m[i];

        // Don't normalize; instead do a projection at the end
        T a[2];                   // Platform attitude

        a[0] = SplineFitScalar::EndPointCubic(am[0], bm[0], ap[0], bp[0], t);
        if (!_fixedpitch)
            a[1] = SplineFitScalar::EndPointCubic(am[1], bm[1], ap[1], bp[1], t);
        else
            a[1] = pitch;

        // Apply rotz(yaw)
        T sz = sin(a[0]);
        T cz = cos(a[0]);
        T d1[3] { cz * d0[0] - sz * d0[1], sz * d0[0] + cz * d0[1], d0[2] };

        // Apply rotx(-pitch)
        T sx = -sin(a[1]);
        T cx = cos(a[1]);
        T d2[3] = { d1[0], cx * d1[1] - sx * d1[2], sx * d1[1] + cx * d1[2] };

        // Apply roty(scan)
        // N.B. in some datasets scan sign is wrong, this needs to be fixed when
        // reading the dataset.
        T sy = sin(ang);
        T cy = cos(ang);
        T d3[3] { sy * d2[2] + cy * d2[0], d2[1], cy * d2[2] - sy * d2[0] };
        residuals[0] = d3[0] / d3[2];
        //pitchweight needs to be tuned. keep it 1 for now
        residuals[1] = pitchweight * d3[1] / d3[2];
        return true;
    }
};

} // namespace trajectory
} // namespace pdal
