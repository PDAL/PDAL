// GeoreferenceFilter.cpp
#include <pdal/filters/TransformationFilter.hpp>
#include <GeoreferenceFilter.hpp>
#include "private/Trajectory.hpp"
#include "private/Utils.hpp"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Constants.hpp>

#include <pdal/pdal_internal.hpp>

namespace pdal
{
static PluginInfo const s_info{ "filters.georeference", "Georeferencing filter", "http://link/to/documentation" };

CREATE_SHARED_STAGE(GeoreferenceFilter, s_info)

using DimId = Dimension::Id;

struct GeoreferenceFilter::Config
{
public:
  Trajectory m_trajectory;
  Eigen::Affine3d m_scan2imu;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Config(const std::string& trajFile, const TransformationFilter::Transform& matrix) : m_trajectory(trajFile)
  {
    Eigen::Matrix4d m;
    m << matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5], matrix[6], matrix[7], matrix[8], matrix[9],
        matrix[10], matrix[11], matrix[12], matrix[13], matrix[14], matrix[15];
    m_scan2imu.matrix() = m;
  }
};

GeoreferenceFilter::GeoreferenceFilter()
  : Filter()
  , Streamable()
  , m_config(nullptr)
  , m_reverse(false)
  , m_matrix(new TransformationFilter::Transform)
  , m_timeOffset(0.0)
  , m_localCartesian(new GeographicLib::LocalCartesian(0.0, 0.0, 0.0, GeographicLib::Geocentric::WGS84()))
{
}

std::string GeoreferenceFilter::getName() const
{
  return s_info.name;
}

void GeoreferenceFilter::addArgs(ProgramArgs& args)
{
  args.add("trajectory", "Path to trajectory file", m_trajectory).setPositional();
  args.add("scan2imu", "Transformation from scanner to imu", *m_matrix).setPositional();
  args.add("reverse", "reverse georeferencing", m_reverse, m_reverse);
  args.add("time_offset", "time offset between trajectory and scanner timestamps", m_timeOffset, m_timeOffset);
}

void GeoreferenceFilter::initialize()
{
  m_config.reset(new Config(m_trajectory, *m_matrix));
}

void GeoreferenceFilter::addDimensions(PointLayoutPtr layout)
{
  layout->registerDim(DimId::ScanAngleRank);
}

void GeoreferenceFilter::prepared(PointTableRef table)
{
}

bool GeoreferenceFilter::processOne(PointRef& point)
{
  TrajPoint barycenter;
  if (!m_config->m_trajectory.getTrajPoint(point.getFieldAs<double>(Dimension::Id::GpsTime) + m_timeOffset, barycenter))
    return false;

  const Eigen::Vector3d scanAngle(point.getFieldAs<double>(Dimension::Id::BeamDirectionX),
                                  point.getFieldAs<double>(Dimension::Id::BeamDirectionY),
                                  point.getFieldAs<double>(Dimension::Id::BeamDirectionZ));

  point.setField(Dimension::Id::ScanAngleRank, Utils::rad2deg(std::atan2(scanAngle.y(), scanAngle.x())));

  const Eigen::Affine3d transform(Utils::getTransformation(0.0, 0.0, 0.0, barycenter.roll, barycenter.pitch,
                                                           barycenter.azimuth - barycenter.wanderAngle) *
                                  m_config->m_scan2imu);

  m_localCartesian->Reset(Utils::rad2deg(barycenter.y), Utils::rad2deg(barycenter.x), barycenter.z);
  if (m_reverse)
  {
    Eigen::Vector3d ned;
    m_localCartesian->Forward(point.getFieldAs<double>(DimId::Y), point.getFieldAs<double>(DimId::X),
                              point.getFieldAs<double>(DimId::Z), ned.y(), ned.x(), ned.z());
    ned.z() = -ned.z();
    const Eigen::Vector3d scan(transform.inverse() * ned);
    point.setField(DimId::X, scan.x());
    point.setField(DimId::Y, scan.y());
    point.setField(DimId::Z, scan.z());
  }
  else
  {
    const Eigen::Vector3d ned(transform * Eigen::Vector3d(point.getFieldAs<double>(DimId::X),
                                                          point.getFieldAs<double>(DimId::Y),
                                                          point.getFieldAs<double>(DimId::Z)));

    double lat(0.0), lon(0.0), h(0.0);
    m_localCartesian->Reverse(ned.y(), ned.x(), -ned.z(), lat, lon, h);
    point.setField(DimId::X, lon);
    point.setField(DimId::Y, lat);
    point.setField(DimId::Z, h);
  }
  return true;
}

void GeoreferenceFilter::filter(PointView& view)
{
  PointRef point(view, 0);
  for (PointId idx = 0; idx < view.size(); ++idx)
  {
    point.setPointId(idx);
    processOne(point);
  }
  view.invalidateProducts();
}

}  // namespace pdal