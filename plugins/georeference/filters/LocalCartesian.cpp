/******************************************************************************
* Copyright (c) 2023, Guilhem Villemin (guilhem.villemin@altametris.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

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