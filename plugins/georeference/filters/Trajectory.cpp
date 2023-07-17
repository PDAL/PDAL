#include <cmath>
#include <pdal/pdal_types.hpp>
#include "private/Trajectory.hpp"
#include "private/Utils.hpp"
#include <filesystem>
#include <algorithm>

#include <pdal/io/SbetReader.hpp>

namespace pdal
{
using DimId = Dimension::Id;

Trajectory::Trajectory(const std::filesystem::path& trajFile)
{
  SbetReader reader;

  Options readerOptions;
  readerOptions.add("filename", trajFile.string());
  readerOptions.add("angles_as_degrees", false);

  reader.addOptions(readerOptions);
  reader.prepare(m_table);
  m_set = reader.execute(m_table);

  m_pointView = *(m_set.begin());
}

bool Trajectory::getTrajPoint(double time, TrajPoint& output) const
{
  PointViewIter upper =
      std::lower_bound(m_pointView->begin(), m_pointView->end(), time,
                       [](const PointRef pt, double time) { return pt.getFieldAs<double>(DimId::GpsTime) < time; });
  if (upper != m_pointView->begin() && upper != m_pointView->end())
  {
    PointViewIter lower = upper - 1;
    PointRef p1 = *(upper - 1);
    PointRef p2 = *upper;
    const double t1 = p1.getFieldAs<double>(DimId::GpsTime);
    const double t2 = p2.getFieldAs<double>(DimId::GpsTime);
    const double frac = (time - t1) / (t2 - t1);

    output.roll = Utils::getAngle(p1.getFieldAs<double>(DimId::Roll), p2.getFieldAs<double>(DimId::Roll), frac);
    output.pitch = Utils::getAngle(p1.getFieldAs<double>(DimId::Pitch), p2.getFieldAs<double>(DimId::Pitch), frac);
    output.azimuth =
        Utils::getAngle(p1.getFieldAs<double>(DimId::Azimuth), p2.getFieldAs<double>(DimId::Azimuth), frac);
    output.wanderAngle =
        Utils::getAngle(p1.getFieldAs<double>(DimId::WanderAngle), p2.getFieldAs<double>(DimId::WanderAngle), frac);
    output.x = Utils::getAngle(p1.getFieldAs<double>(DimId::X), p2.getFieldAs<double>(DimId::X), frac);
    output.y = Utils::getAngle(p1.getFieldAs<double>(DimId::Y), p2.getFieldAs<double>(DimId::Y), frac);
    output.z = Utils::getValue(p1.getFieldAs<double>(DimId::Z), p2.getFieldAs<double>(DimId::Z), frac);
    output.time = Utils::getValue(p1.getFieldAs<double>(DimId::GpsTime), p2.getFieldAs<double>(DimId::GpsTime), frac);
    return true;
  }
  return false;
}

}  // namespace pdal