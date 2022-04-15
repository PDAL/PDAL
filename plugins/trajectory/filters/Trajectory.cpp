//
// (c) 2022 SRI International
//
#include "Trajectory.hpp"

#include <limits>
#include <cmath>
#include <Eigen/Dense>

#include "Args.hpp"
#include "PulseCollection.hpp"

namespace pdal
{

static PluginInfo const s_info
{
    "filters.trajectory",
    "SRI Trajectory calculator",
    "http://link/to/documentation"
};

CREATE_SHARED_STAGE(Trajectory, s_info)

Trajectory::Trajectory() : m_args(new trajectory::Args)
{}

Trajectory::~Trajectory()
{}

std::string Trajectory::getName() const
{ return s_info.name; }

void Trajectory::addArgs(ProgramArgs& args)
{
    args.add("dtr", "Multi-return sampling interval (secs) (inf = don't sample)",
        m_args->dtr, .001);
    args.add("dts", "Single return sampling interval (secs) (inf = don't sample)",
        m_args->dts, .001);
    args.add("minsep", "Minimum separation (meters) of returns considered",
        m_args->minsep, .01);
    args.add("tblock", "Block size for cubic spline (secs)",
        m_args->tblock, 1.0);
    args.add("tout", "Output data interval (secs)", m_args->tout, .01);
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

void Trajectory::prepared(PointTableRef table)
{
    using D = Dimension::Id;

    std::vector<D> dims { D::GpsTime, D::ScanAngleRank, D::NumberOfReturns, D::ReturnNumber };

    for (const D& dim : dims)
        if (!table.layout()->hasDim(dim))
        {
            Dimension::name(dim);  // These are all well-defined dimensions.
            throwError("Can't compute trajectory when data is missing dimension '" +
                Dimension::name(dim) + "'.");
        }
}

PointViewSet Trajectory::run(PointViewPtr inView)
{
    if (!inView->size())
        throwError("No returns to process.");

    PointViewSet out;
    try
    {
        out = runAlgorithm(inView);
    }
    catch (const std::exception& e)
    {
        throwError(e.what());
    }
    return out;
}

PointViewSet Trajectory::runAlgorithm(PointViewPtr inView)
{
    PointViewPtr outView = inView->makeNew();

    trajectory::PulseCollection coll(*m_args);

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

    double tmin = inView->getFieldAs<double>(Dimension::Id::GpsTime, 0);
    double tmax = inView->getFieldAs<double>(Dimension::Id::GpsTime, inView->size() - 1);

    tmin = std::floor(tmin / m_args->tout);
    int nt = int(std::ceil(tmax / m_args->tout) - tmin);
    tmin *= m_args->tout;

    for (int it = 0; it <= nt; ++it)
    {
        double t = tmin + it * m_args->tout;
        Eigen::Vector3d vel, accel;
        Eigen::Vector3d pos = coll.Trajectory(t, vel, accel);
        Eigen::Vector2d attvel;
        Eigen::Vector2d att = coll.Attitude(t, attvel);
  
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
