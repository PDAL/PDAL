#pragma once

#include <pdal/PointRef.hpp>

#include <memory>

class OGRCoordinateTransformation;

namespace pdal
{
class LocalCartesian
{
    double m_lat0, m_lon0, m_h0;
    std::unique_ptr<OGRCoordinateTransformation> m_forwardTransfo,
        m_reverseTransfo;
    ;

public:
    LocalCartesian(double lat0, double lon0, double h = 0.0);
    ~LocalCartesian();

    void reset(double lat0, double lon0, double h0 = 0.0);
    bool forward(PointRef& point);
    bool reverse(PointRef& point);
};
} // namespace pdal