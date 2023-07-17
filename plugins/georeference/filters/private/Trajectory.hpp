#pragma once
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/pdal_types.hpp>

#include <filesystem>
#include <list>
#include <map>
#include <string>
#include <utility>

namespace pdal
{
struct TrajPoint;
class Trajectory
{
  pdal::PointViewPtr m_pointView;
  pdal::PointTable m_table;
  pdal::PointViewSet m_set;

public:
  Trajectory(const std::filesystem::path& trajFile);

  bool getTrajPoint(double time, TrajPoint& trajPoint) const;
};

struct TrajPoint
{
  double roll, pitch, azimuth, wanderAngle, x, y, z, time;
};

};  // namespace pdal
