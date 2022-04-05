
#include "Trajectory.hpp"

#include <iostream>
#include <limits>
#include <cmath>
#include <Eigen/Dense>

#include "PulseCollection.hpp"

namespace pdal
{

static PluginInfo const s_info
{
    "filters.sritrajectory",
    "SRI Trajectory calculator",
    "http://link/to/documentation"
};

struct Trajectory::PrivateArgs
{
    StringList m_params;
};

CREATE_SHARED_STAGE(Trajectory, s_info)

Trajectory::Trajectory() : m_args(new PrivateArgs)
{}

Trajectory::~Trajectory()
{}

std::string Trajectory::getName() const
{ return s_info.name; }

void Trajectory::addArgs(ProgramArgs& args)
{
    args.add("params", "Set params on backend engine using "
             "\"key1 = val1, key2 = val2, ...\".  Useful parameters "
             "(and defaults) are: dt (0.0005 s) the sampling interval; "
             "tblock (1 s) the spline block size; "
             "tout (0.005 s) the time resolustion for the output trajectory; "
             "dr (0.01 m) the resolution of the lidar positions.",
             m_args->m_params, StringList(0));
}

void Trajectory::addDimensions(PointLayoutPtr layout)
{
    using D = Dimension::Id;

    layout->registerDims({ D::XVelocity, D::YVelocity, D::ZVelocity,
        D::XBodyAccel, D::YBodyAccel, D::ZBodyAccel, D::Azimuth, D::Pitch });

    layout->registerDim(D::XBodyAngRate); // dPitch/dt
    // YBodyAngRate - not set.
    layout->registerDim(D::ZBodyAngRate); // dAzimuth/dt
}

PointViewSet Trajectory::run(PointViewPtr inView)
{
    if (!inView->size())
        throwError("No returns to process.");

    PointViewPtr outView = inView->makeNew();

    LidarTrajectory::PulseCollection coll;

    Eigen::Vector3d r;
    for (PointRef point : *inView)
    {
        double t = point.getFieldAs<double>(Dimension::Id::GpsTime);
        r(0) = point.getFieldAs<double>(Dimension::Id::X);
        r(1) = point.getFieldAs<double>(Dimension::Id::Y);
        r(2) = point.getFieldAs<double>(Dimension::Id::Z);
        double ang = point.getFieldAs<double>(Dimension::Id::ScanAngleRank);
        int num = point.getFieldAs<int>(Dimension::Id::NumberOfReturns);
        int ret = point.getFieldAs<int>(Dimension::Id::ReturnNumber);
        coll.Add(t, r, num, ret, ang);
    }

    coll.Solve();

//    double tout = coll.params.lookup<double>("tout", 0.005);
    double tout = .005;
    double tmin = inView->getFieldAs<double>(Dimension::Id::GpsTime, 0);
    double tmax = inView->getFieldAs<double>(Dimension::Id::GpsTime, inView->size() - 1);

    tmin = std::floor(tmin / tout);
    int nt = int(std::ceil(tmax / tout) - tmin);
    tmin *= tout;

    Eigen::Vector3d pos, vel, accel;
    Eigen::Vector2d att, attvel;
    for (int it = 0; it <= nt; ++it)
    {
        double t = tmin + it * tout;
        pos = coll.Trajectory(t, vel, accel);
        att = coll.Attitude(t, attvel);
  
        PointId idx = outView->size();
        outView->setField(Dimension::Id::GpsTime,      idx, t);
        outView->setField(Dimension::Id::X,            idx, pos(0));
        outView->setField(Dimension::Id::Y,            idx, pos(1));
        outView->setField(Dimension::Id::Z,            idx, pos(2));
        outView->setField(Dimension::Id::XVelocity,    idx, vel(0));
        outView->setField(Dimension::Id::YVelocity,    idx, vel(1));
        outView->setField(Dimension::Id::ZVelocity,    idx, vel(2));
        outView->setField(Dimension::Id::XBodyAccel,   idx, accel(0));
        outView->setField(Dimension::Id::YBodyAccel,   idx, accel(1));
        outView->setField(Dimension::Id::ZBodyAccel,   idx, accel(2));
        outView->setField(Dimension::Id::Azimuth,      idx, att(0));
        outView->setField(Dimension::Id::Pitch,        idx, att(1));
        outView->setField(Dimension::Id::ZBodyAngRate, idx, attvel(0));
        outView->setField(Dimension::Id::XBodyAngRate, idx, attvel(1));
    }

    return PointViewSet { outView };
  }

} // namespace pdal
