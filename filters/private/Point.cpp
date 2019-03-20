/******************************************************************************
* Copyright (c) 2016, Howard Butler (howard@hobu.co)
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
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
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

#include "Point.hpp"

namespace pdal
{

namespace filter
{

namespace
{

const double LOWEST = (std::numeric_limits<double>::lowest)();

}

Point::Point()
{
    m_geom.reset(new OGRPoint());
    clear();
}


Point::Point(const std::string& wkt_or_json, SpatialReference ref)
    : Geometry(wkt_or_json, ref)
{}


void Point::update(const std::string& wkt_or_json)
{
    Geometry::update(wkt_or_json);

    OGRwkbGeometryType t = m_geom->getGeometryType();
    if (t != wkbPoint && t != wkbPoint25D)
    {
        m_geom.reset(new OGRPoint());
        throw pdal_error("Can't set Point from string.  String doesn't "
            "represent a point");
    }

    // We use a sentinel for 3D that's different from what GDAL uses.
    if (m_geom->getCoordinateDimension() == 2)
        z(LOWEST);
}


void Point::clear()
{
    x(LOWEST);
    y(LOWEST);
    z(LOWEST);
}


bool Point::empty() const
{
    return (x() == LOWEST && y() == LOWEST && z() == LOWEST);
}


bool Point::is3d() const
{
    return (z() != LOWEST);
}


double Point::x() const
{
    return static_cast<OGRPoint *>(m_geom.get())->getX();
}


double Point::y() const
{
    return static_cast<OGRPoint *>(m_geom.get())->getY();
}


double Point::z() const
{
    return static_cast<OGRPoint *>(m_geom.get())->getZ();
}


void Point::x(double xval)
{
    static_cast<OGRPoint *>(m_geom.get())->setX(xval);
}


void Point::y(double yval)
{
    static_cast<OGRPoint *>(m_geom.get())->setY(yval);
}


void Point::z(double zval)
{
    static_cast<OGRPoint *>(m_geom.get())->setZ(zval);
}

} // namespace filter

} // namespace pdal
