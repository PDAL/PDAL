#pragma once

#include <memory>
#include <pdal/Geometry.hpp>
#include <pdal/Log.hpp>
#include <pdal/PointRef.hpp>
#include <pdal/SpatialReference.hpp>

namespace pdal {

class KD2Index;
class RowPointTable;
class PointView;

// TODO : add a kdTree to look for segment more easily (using a pdal KDIndex ?)
class Polyline : public Geometry {
public:
  Polyline();
  virtual ~Polyline();

  Polyline(const std::string &wkt_or_json,
           SpatialReference ref = SpatialReference());
  Polyline(OGRGeometryH g);
  Polyline(OGRGeometryH g, const SpatialReference &srs);
  Polyline(const Polyline &poly);
  Polyline &operator=(const Polyline &src);

  // give the closest point in segment on the polyline
  double closestSegment(const PointRef &point, double &x, double &y, double &z,
                        double &m, double &azimuth, double &offset);

  // returns the given polyline point for a given PK as X point dimension
  void interpolate(const PointRef &point, double &x, double &y, double &z,
                   double &m, double &azimuth, double &offset);

  virtual void modified() override;
  virtual void clear() override;

private:
  void init();

  std::unique_ptr<KD2Index> m_index;
  std::unique_ptr<RowPointTable> m_table;
  std::unique_ptr<PointView> m_view;
};

} // namespace pdal