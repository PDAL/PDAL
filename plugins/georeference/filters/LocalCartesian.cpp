#include "private/LocalCartesian.hpp"
#include <ogr_spatialref.h>

namespace pdal
{
LocalCartesian::LocalCartesian(double lat0, double lon0, double h0)
{
    reset(lat0, lon0, h0);
}

LocalCartesian::~LocalCartesian() {}

void LocalCartesian::reset(double lat0, double lon0, double h0)
{
    OGRCoordinateTransformationOptions coordTransfoOptions;
    std::string coordOperation =
        "+proj=pipeline +step +proj=cart +ellps=WGS84 +step +proj=topocentric "
        "+ellps=WGS84 +lon_0=" +
        std::to_string(lon0) + " +lat_0=" + std::to_string(lat0) +
        " +h_0=" + std::to_string(h0);
    coordTransfoOptions.SetCoordinateOperation(coordOperation.c_str(), false);
    OGRSpatialReference nullSrs("");
    m_forwardTransfo.reset(OGRCreateCoordinateTransformation(
        &nullSrs, &nullSrs, coordTransfoOptions));
    coordTransfoOptions.SetCoordinateOperation(coordOperation.c_str(), true);
    m_reverseTransfo.reset(OGRCreateCoordinateTransformation(
        &nullSrs, &nullSrs, coordTransfoOptions));
}

bool LocalCartesian::forward(PointRef& point)
{
    double x(point.getFieldAs<double>(Dimension::Id::X));
    double y(point.getFieldAs<double>(Dimension::Id::Y));
    double z(point.getFieldAs<double>(Dimension::Id::Z));

    bool ok = m_forwardTransfo && m_forwardTransfo->Transform(1, &x, &y, &z);
    if (ok)
    {
        point.setField(Dimension::Id::X, x);
        point.setField(Dimension::Id::Y, y);
        point.setField(Dimension::Id::Z, z);
    }
    return ok;
}

bool LocalCartesian::reverse(PointRef& point)
{
    double x(point.getFieldAs<double>(Dimension::Id::X));
    double y(point.getFieldAs<double>(Dimension::Id::Y));
    double z(point.getFieldAs<double>(Dimension::Id::Z));

    bool ok = m_reverseTransfo && m_reverseTransfo->Transform(1, &x, &y, &z);
    if (ok)
    {
        point.setField(Dimension::Id::X, x);
        point.setField(Dimension::Id::Y, y);
        point.setField(Dimension::Id::Z, z);
    }
    return ok;
}

} // namespace pdal