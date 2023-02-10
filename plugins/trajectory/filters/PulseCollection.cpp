//
// (c) 2022 SRI International
//
#include <cmath>
#include <ceres/ceres.h>

#include <pdal/pdal_types.hpp>

#include "PulseCollection.hpp"

namespace pdal
{
namespace trajectory
{

PulseCollection::PulseCollection(const Args& args) : m_args(args), m_first(true), m_lowhigh(0)
{}

PulseCollection::~PulseCollection()
{}

void PulseCollection::Add(double time, const Eigen::Vector3d& r, int nr, int rn, double angle)
{
    // Let's skip return unless num and ret are sensible
    if (rn < 1 || rn > nr)
        return;

    angle *= m_args.flipScanAngle;

    if (m_first)
    {
        // Somewhat arbitrary choice of origin (to ensure internal calculations
        // are well conditioned.
        m_timeOrigin = std::floor(time / 10) * 10;  // Round left to lowest 10 value.
        m_rOrigin = (Eigen::floor(r.array() / 100) * 100).matrix(); // Round to nearest 100
        m_timeMin = time;
        m_first = false;
    }
    double adjTime = time - m_timeOrigin;
    if (adjTime < m_lastAdjTime)
        throw pdal_error("PulseCollection: returns are not sorted in time");

    if (nr == 1)
    {
        addPoint(adjTime, r - m_rOrigin, r - m_rOrigin, angle);
        m_lowhigh = 0;     // In case we didn't get a good multi-return pulse.
    }
    else
    {
        if (rn == 1)
        {
            m_rlow = r;
            m_lowhigh++;
        }
        else if (nr == rn)
        {
            m_rhigh = r;
            m_lowhigh++;
        }
        if (m_lowhigh == 2)
        {
            addPoint(adjTime, m_rlow - m_rOrigin, m_rhigh - m_rOrigin, angle);
            m_lowhigh = 0;
        }
    }
    m_lastAdjTime = adjTime;
    m_timeMax = time;
}

bool PulseCollection::usingMulti() const
{
    return std::isfinite(m_args.dtr);
}

bool PulseCollection::usingSingle() const
{
    return std::isfinite(m_args.dts);
}

void PulseCollection::addPoint(double time, const Eigen::Vector3d& startPos,
    const Eigen::Vector3d& endPos, double angle)
{
    auto shouldRegister = [](const std::vector<Pulse>& buf, double curTime, double interval) -> bool
    {
        if (buf.empty())
            return false;
        return std::floor(buf.front().t / interval) != std::floor(curTime / interval);
    };

    if (usingMulti())
    {
        if (shouldRegister(m_multiBuf, time, m_args.dtr))
        {
            registerMulti(m_multiBuf);
            m_multiBuf.clear();
        }
        m_multiBuf.push_back(Pulse(time, startPos, endPos, angle));
    }
    if (usingSingle())
    {
        if (shouldRegister(m_singleBuf, time, m_args.dts))
        {
            registerSingle(m_singleBuf);
            m_singleBuf.clear();
        }
        m_singleBuf.push_back(Pulse(time, startPos, angle));
    }
}

void PulseCollection::registerMulti(const std::vector<Pulse>& buf)
{
    assert(buf.size());

    auto it = std::max_element(buf.begin(), buf.end(),
        [](const Pulse& p1, const Pulse& p2){ return p1.d < p2.d; });
    const Pulse& p = *it;

    // Only include pulses if separation > minsep
    if (p.d * 2 > m_args.minsep)
        pulses.push_back(p);
}

void PulseCollection::registerSingle(const std::vector<Pulse>& buf)
{
    assert(buf.size());

    // Look for midpoint of a run of pulses with the same angle (on the
    // theory that this will have the smallest quantization error).
    if (buf.size() > (std::numeric_limits<int>::max)())
        throw pdal_error("Attempting to register an oversized vector.");

    int mid = buf.size() / 2;
    int low = mid - 1;
    int high = mid + 1;
    double angle = buf[mid].ang;
    while (low >= 0 && buf[low].ang == angle)
        --low;
    while (high < (int)buf.size() && buf[high].ang == angle)
        ++high;

    // low can end up -1 and high can end up == buf.size(), but the because integer division
    // rounds to 0, this will always yield a value in range.
    mid = (low + high) / 2;
    pulses.push_back(buf[mid]);

    // If change in scan angle consistent
    if (low >= 0 && high < (int)buf.size() &&
        (buf[mid].ang - buf[low].ang) * (buf[high].ang - buf[mid].ang) > 0)
        pulses.back().n = (buf[mid].ang - buf[low].ang > 0 ? 1 : -1) * (buf[high].r - buf[low].r);
}

bool
PulseCollection::EstimatedPositionVelocity(double t, Eigen::Vector3d& r, Eigen::Vector3d& v) const
{
    std::vector<Pulse> psub;

    // Extract pulses in a window around t. Adjust pulse times to be relative to t.
    for (const Pulse& p: pulses)
    {
      if (p.t >= t - m_args.tblock && p.t <= t + m_args.tblock) {
        psub.push_back(p);
        psub.back().t -= t;
      }
    }

    int k = int(psub.size());

    // Accumulate scan vector for scan-angle pulses
    int kscan = 0;
    double sx, sy;
    bool skipscan = false;

    {
        Eigen::Vector3d scandir(Eigen::Vector3d::Zero());
        for (int l = 0; l < k; ++l) {
            if (!psub[l].MultiReturn()) {
                scandir += psub[l].n;
                ++kscan;
            }
        }
        sx = -scandir(0); sy = -scandir(1);
        double h = std::hypot(sx, sy);
        if (h > 0)
        { sx /= h; sy /= h; }
        else
            skipscan = true;
    }

    int m = skipscan ? k - kscan : k;
    if (m  < m_args.estn)
        return false;

    // For multi-return pulses
    // equation for pulse starting at r in direction n, distance = s
    //
    //   x + vx*t = rx + nx * s
    //   y + vy*t = ry + ny * s
    //   z + vz*t = rz + nz * s
    //
    // replace s by z as parameterization, s = (z + t*vz - rz)/nz
    //   nz*x      - nx*z + nz*t*vx           - nx*t*vz = nz*rx - nx*rz
    //        nz*y - ny*z           + nz*t*vy - ny*t*vz = nz*ry - ny*rz
    // or
    //
    //    A . [x,y,z,vx,vy,vz]' = B
    //
    // where
    //
    //   A = [nz,  0, -nx, nz*t,    0, -nx*t]
    //       [ 0, nz, -ny,    0, nz*t, -ny*t]
    //       ... for all pulses
    //   B = [nz * rx - nx * rz]
    //       [nz * ry - ny * rz]
    //       ... for all pulses
    // finally, we'll weight use pulse contribution by d

    // For scan angle pulses, we can substitute
    //
    //   nx = sin(ang) * sx, ny = sin(ang) * sy, nz = cos(ang)
    //
    // Thus assumes pitch = 0.  It would be simple to use fixedpitch here.
    Eigen::MatrixXd A(2 * m, 6);
    Eigen::VectorXd B(2 * m);
    for (int l = 0, h = 0; l < k; ++l)
    {
        const Pulse& p = psub[l];
        double nx, ny, nz, w;
        if (p.MultiReturn())
        {
            nx = p.n(0); ny = p.n(1); nz = p.n(2);
            w = p.d;
        }
        else
        {
            if (skipscan)
                continue;
            double sang = sin(p.ang);
            double cang = cos(p.ang);
            nx = sang * sx;
            ny = sang * sy;
            nz = cang;
            w = m_args.scanweightest;
        }
        A(2*h+0, 0) = nz; A(2*h+0, 2) = -nx;
        A(2*h+1, 1) = nz; A(2*h+1, 2) = -ny;
        A(2*h+0, 3) = nz*p.t; A(2*h+0, 5) = -nx*p.t;
        A(2*h+1, 4) = nz*p.t; A(2*h+1, 5) = -ny*p.t;
        A(2*h+0, 1) = A(2*h+0, 4) = A(2*h+1, 0) = A(2*h+1, 3) = 0;
        B(2*h+0) = nz * p.r(0) - nx * p.r(2);
        B(2*h+1) = nz * p.r(1) - ny * p.r(2);
        A.row(2*h+0) *= w; B(2*h+0) *= w;
        A.row(2*h+1) *= w; B(2*h+1) *= w;
        ++h;
    }
    Eigen::Matrix<double, 1, Eigen::Dynamic> rv(A.jacobiSvd(Eigen::ComputeThinU |
                Eigen::ComputeThinV).solve(B));
    r = rv.head<3>();
    v = rv.tail<3>();
    return true;
}

void PulseCollection::InitializeTrajectory()
{
    if (pulses.empty())
        throw pdal_error("PulseCollection: no pulses for Solve");

    double tstart = std::floor((m_timeMin - m_timeOrigin) / m_args.tblock);
    int num = int(std::ceil((m_timeMax - m_timeOrigin) / m_args.tblock) - tstart) - 1;
    if (num < 1)
        throw pdal_error("PulseCollection: no time interval for Solve");
    tstart *= m_args.tblock;

    traj = SplineFit3(num, m_args.tblock, tstart);
    attitude = SplineFit2(num, m_args.tblock, tstart);

    for (int i = 0; i <= num; ++i)
    {
        traj.missing[i] = !EstimatedPositionVelocity(tstart + i * m_args.tblock,
            traj.r[i], traj.v[i]);
        traj.v[i] *= m_args.tblock;
    }

    if (!traj.fillmissing(true))
        throw pdal_error("PulseCollection: too few pulses for initial estimate of trajectory");

    for (int i = 0; i <= num; ++i)
    {
        int im = (std::max)(0, i - 1);
        int ip = (std::min)(num, i + 1);

        // atan(dx, dy) to give clockwise from north convention
        attitude.r[i] = Eigen::Vector2d(std::atan2(traj.r[ip](0) - traj.r[im](0),
            traj.r[ip](1) - traj.r[im](1)),
            std::isnan(m_args.fixedpitch) ? 0.0 : degreesToRadians(m_args.fixedpitch));
        attitude.v[i] = Eigen::Vector2d::Zero();
    }
    // Make sure heading doesn't jump around
    double ang0 = attitude.r[0](0);
    for (int i = 1; i <= num; ++i)
    {
        double ang1 = attitude.r[i](0);
        attitude.r[i](0) = ang0 + normalizeRadians(ang1 - ang0);
    }
}

void PulseCollection::Solve()
{
    if (usingMulti() && m_multiBuf.size())
        registerMulti(m_multiBuf);
    if (usingSingle() && m_singleBuf.size())
        registerSingle(m_singleBuf);

    InitializeTrajectory();
    int num = traj.num;

    ceres::Problem problem;

    // for debugging
    int numMultiReturnResidualBlocks = 0;
    int numScanAngleResidualBlocks = 0;

    // Set up residual blocks for pulses
    for (const Pulse& p : pulses)
    {
      auto tconv = traj.tconvert(p.t);
      int i = tconv.first;
      double t = tconv.second;
      if (p.MultiReturn())
      {
          ceres::CostFunction* cost_function =
              new ceres::AutoDiffCostFunction<FirstLastError,
                  2,       // number of residuals
                  3,3,3,3> // data for cubic fit
                      (new FirstLastError(p, t));
          ceres::LossFunction* loss_function =
              new ceres::ScaledLoss(new ceres::CauchyLoss(m_args.dr),
                      m_args.multiweight,
                      ceres::TAKE_OWNERSHIP);
          problem.AddResidualBlock(cost_function, loss_function,
                  traj.r[i  ].data(),
                  traj.v[i  ].data(),
                  traj.r[i+1].data(),
                  traj.v[i+1].data());
          numMultiReturnResidualBlocks++;
      }
      else
      {
        ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ScanAngleError,
                                          2,       // number of residuals
                                          3,3,3,3,
                                          2,2,2,2> // data for cubic fit
          (new ScanAngleError(p, t, m_args.pitchweight, degreesToRadians(m_args.fixedpitch)));
        ceres::LossFunction* loss_function = new ceres::ScaledLoss
          (new ceres::CauchyLoss(degreesToRadians(m_args.dang)), m_args.scanweight,
          ceres::TAKE_OWNERSHIP);
        problem.AddResidualBlock(cost_function, loss_function,
                                 traj.r[i  ].data(),
                                 traj.v[i  ].data(),
                                 traj.r[i+1].data(),
                                 traj.v[i+1].data(),
                                 attitude.r[i  ].data(),
                                 attitude.v[i  ].data(),
                                 attitude.r[i+1].data(),
                                 attitude.v[i+1].data());
        numScanAngleResidualBlocks++;
      }
    }

    // The acceleration constraints for traj and attitude
    if (m_args.accelweight > 0) {
      for (int i = 1; i < num; ++i) {
        ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<AccelJumpConstraint<3>,
                                          3,       // number of residuals
                                          3,3,3,3,3> // data for cubic fit
          (new AccelJumpConstraint<3>(m_args.tblock));
        ceres::LossFunction* loss_function =
          (ceres::LossFunction*)
          (new ceres::ScaledLoss(nullptr, m_args.accelweight,
                                 ceres::TAKE_OWNERSHIP));
        problem.AddResidualBlock(cost_function, loss_function,
                                 traj.r[i-1].data(),
                                 traj.v[i-1].data(),
                                 traj.v[i  ].data(),
                                 traj.r[i+1].data(),
                                 traj.v[i+1].data());
      }
    }
    if (m_args.attaccelweight > 0) {
      for (int i = 1; i < num; ++i) {
        ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<AccelJumpConstraint<2>,
                                          2,       // number of residuals
                                          2,2,2,2,2> // data for cubic fit
          (new AccelJumpConstraint<2>(m_args.tblock));
        ceres::LossFunction* loss_function =
          (ceres::LossFunction*)
          (new ceres::ScaledLoss(nullptr, m_args.attaccelweight, ceres::TAKE_OWNERSHIP));
        problem.AddResidualBlock(cost_function, loss_function,
                                 attitude.r[i-1].data(),
                                 attitude.v[i-1].data(),
                                 attitude.v[i  ].data(),
                                 attitude.r[i+1].data(),
                                 attitude.v[i+1].data());
      }
    }

    // Also allow "clamping" of at the nodes with constrains neighboring cubic
    // polynomials to be close to one another.  In general clampweight should
    // be "small".  straddleweight is a larger weight to enforce clamping where
    // there's a sparsity of data.
    if (m_args.clampweight > 0 || m_args.straddleweight > 0)
    {
        for (int i = 1; i < num; ++i)
        {
            double w = traj.missing[i] ? m_args.straddleweight : m_args.clampweight;
            if (w <= 0)
                continue;
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<ClampConstraint<3>,
                    3,       // number of residuals
                    3,3,3,3,3> // data for cubic fit
                        (new ClampConstraint<3>(m_args.tblock));
            ceres::LossFunction* loss_function =
                (ceres::LossFunction*)
                (new ceres::ScaledLoss(nullptr, w,
                                       ceres::TAKE_OWNERSHIP));
            problem.AddResidualBlock(cost_function, loss_function,
                    traj.r[i-1].data(),
                    traj.v[i-1].data(),
                    traj.r[i  ].data(),
                    traj.r[i+1].data(),
                    traj.v[i+1].data());
        }
    }
    // The estimate of the pitch can sometimes oscillate too much.
    // extrapitchclamp is a way to suppress this.
    if (m_args.attclampweight > 0)
    {
        Eigen::Vector2d mult(1.0, m_args.extrapitchclamp);
        for (int i = 1; i < num; ++i)
        {
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<ClampConstraint<2>,
                    2,       // number of residuals
                    2,2,2,2,2> // data for cubic fit
                        (new ClampConstraint<2>(m_args.tblock, mult));
            ceres::LossFunction* loss_function =
                (ceres::LossFunction*)
                (new ceres::ScaledLoss(nullptr, m_args.attclampweight,
                                       ceres::TAKE_OWNERSHIP));
            problem.AddResidualBlock(cost_function, loss_function,
                    attitude.r[i-1].data(),
                    attitude.v[i-1].data(),
                    attitude.r[i  ].data(),
                    attitude.r[i+1].data(),
                    attitude.v[i+1].data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.logging_type = ceres::SILENT;
    options.max_linear_solver_iterations = m_args.niter;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}

Eigen::Vector3d
PulseCollection::Trajectory(double t, Eigen::Vector3d& v, Eigen::Vector3d& a) const
{
    return traj.position(t - m_timeOrigin, v, a) + m_rOrigin;
}

Eigen::Vector2d PulseCollection::Attitude(double t, Eigen::Vector2d& v) const
{
    Eigen::Vector2d p = radiansToDegrees(attitude.position(t - m_timeOrigin, v));
    v = radiansToDegrees(v);
    return p;
}

} // namespace trajectory
} // namespace pdal
