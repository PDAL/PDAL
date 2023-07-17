// GeoreferenceFilter.hpp

#pragma once
#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/pdal_internal.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace GeographicLib
{
class LocalCartesian;
}

namespace pdal
{
class TransformationFilter;

class PDAL_DLL GeoreferenceFilter : public Filter, public Streamable
{
public:
  GeoreferenceFilter();

  GeoreferenceFilter& operator=(const GeoreferenceFilter&) = delete;
  GeoreferenceFilter(const GeoreferenceFilter&) = delete;

  std::string getName() const;

private:
  virtual void addArgs(ProgramArgs& args) override;
  virtual void initialize() override;
  virtual bool processOne(PointRef& point) override;
  virtual void filter(PointView& view) override;
  virtual void addDimensions(PointLayoutPtr layout);
  virtual void prepared(PointTableRef table);

  struct Config;
  std::unique_ptr<TransformationFilter::Transform> m_matrix;
  std::unique_ptr<Config> m_config;
  std::unique_ptr<GeographicLib::LocalCartesian> m_localCartesian;
  std::string m_trajectory;
  std::string m_scan2imu;
  double m_timeOffset;
  bool m_reverse;
};

}  // namespace pdal