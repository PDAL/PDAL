// StraightenFilter.cpp
#include <StraightenFilter.hpp>
#include "Polyline.hpp"
#include "Utils.hpp"

#include <pdal/pdal_internal.hpp>

namespace pdal {

static PluginInfo const s_info{"filters.straighten", "Straighten filter",
                               "http://link/to/documentation"};

CREATE_SHARED_STAGE(StraightenFilter, s_info)

struct StraightenFilter::Args {
public:
  Polyline m_polyline;
  bool m_unstraighten;
  double m_offset;
};

StraightenFilter::StraightenFilter()
    : Filter(), Streamable(), m_args(new StraightenFilter::Args) {}

std::string StraightenFilter::getName() const { return s_info.name; }

void StraightenFilter::addArgs(ProgramArgs &args) {
  args.add("polyline",
           "Track polyline to straigthen, LineStringZM, with m value is roll "
           "in radians",
           m_args->m_polyline)
      .setErrorText("Invalid polyline specification. "
                    "Must be valid GeoJSON/WKT");
  args.add("reverse", "Set to true if you the to unstraighten.",
           m_args->m_unstraighten, false);
  args.add("offset",
           "Use a global offset, so that straighten X starts with that value",
           m_args->m_offset, 0.0);
}

void StraightenFilter::initialize() {
  if (!m_args->m_polyline.valid())
    throwError("Geometrically invalid polygon in option 'polyline'.");
}

bool StraightenFilter::processOne(PointRef &point) {

  double segmentX, segmentY, segmentZ, segmentM, segmentAzimuth, segmentOffset;
  if (m_args->m_unstraighten) {
    m_args->m_polyline.interpolate(point, segmentX, segmentY, segmentZ,
                                   segmentM, segmentAzimuth, segmentOffset);

    const Eigen::Vector3d straight(0.0,
                                   point.getFieldAs<double>(Dimension::Id::Y),
                                   point.getFieldAs<double>(Dimension::Id::Z));

    Eigen::Affine3d t;
    Utils::getTransformation(segmentX, segmentY, segmentZ, segmentM, 0.0,
                             M_PI_2 - segmentAzimuth, t);

    const Eigen::Vector3d world = t * straight;

    point.setField(Dimension::Id::X, world.x());
    point.setField(Dimension::Id::Y, world.y());
    point.setField(Dimension::Id::Z, world.z());
    return true;
  } else {

    if (m_args->m_polyline.closestSegment(point, segmentX, segmentY, segmentZ,
                                          segmentM, segmentAzimuth,
                                          segmentOffset) >= 0.0) {
      const Eigen::Vector3d world(point.getFieldAs<double>(Dimension::Id::X),
                                  point.getFieldAs<double>(Dimension::Id::Y),
                                  point.getFieldAs<double>(Dimension::Id::Z));

      Eigen::Affine3d t;
      Utils::getTransformation(segmentX, segmentY, segmentZ, segmentM, 0.0,
                               M_PI_2 - segmentAzimuth, t);

      const Eigen::Vector3d straight = t.inverse() * world;

      point.setField(Dimension::Id::X,
                     straight.x() + segmentOffset + m_args->m_offset);
      point.setField(Dimension::Id::Y, straight.y());
      point.setField(Dimension::Id::Z, straight.z());
      return true;
    }
  }
  return false;
}

void StraightenFilter::filter(PointView &view) {
  PointRef point(view, 0);
  for (PointId idx = 0; idx < view.size(); ++idx) {
    point.setPointId(idx);
    processOne(point);
  }
  view.invalidateProducts();
}

} // namespace pdal