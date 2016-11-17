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

namespace
{

const double LOWEST = (std::numeric_limits<double>::lowest)();
const double HIGHEST = (std::numeric_limits<double>::max)();

}

namespace cropfilter
{

Point::Point()
    : Geometry()
    , x(LOWEST)
    , y(LOWEST)
    , z(LOWEST)
{

};

Point::Point(const std::string& wkt_or_json, SpatialReference ref)
    : Geometry(wkt_or_json, ref)
{

}

void Point::update(const std::string& wkt_or_json, SpatialReference ref)
{

    Geometry::update(wkt_or_json, ref);

    int t = GEOSGeomTypeId_r(m_geoserr.ctx(), m_geom.get());
    if (t == -1)
        throw pdal_error("Unable to fetch geometry point type");
    if (t > 0)
        throw pdal_error("Geometry type is not point!");

    int nGeometries = GEOSGetNumGeometries_r(m_geoserr.ctx(), m_geom.get());
    if (nGeometries > 1)
        throw pdal_error("Geometry count is > 1!");

    const GEOSGeometry* g = GEOSGetGeometryN_r(m_geoserr.ctx(), m_geom.get(), 0);

    GEOSCoordSequence const* coords = GEOSGeom_getCoordSeq_r(m_geoserr.ctx(),  g);

    uint32_t numInputDims;
    GEOSCoordSeq_getDimensions_r(m_geoserr.ctx(), coords, &numInputDims);

    uint32_t count(0);
    GEOSCoordSeq_getSize_r(m_geoserr.ctx(), coords, &count);
    if (count == 0)
        throw pdal_error("No coordinates in geometry!");

    for (unsigned i = 0; i < count; ++i)
    {
        GEOSCoordSeq_getOrdinate_r(m_geoserr.ctx(), coords, i, 0, &x);
        GEOSCoordSeq_getOrdinate_r(m_geoserr.ctx(), coords, i, 1, &y);
        if (numInputDims > 2)
            GEOSCoordSeq_getOrdinate_r(m_geoserr.ctx(), coords, i, 2, &z);
    }

}


void Point::clear()
{
    x = LOWEST; y = LOWEST; z = LOWEST;
}


bool Point::empty() const
{
    return  x == LOWEST && y == LOWEST && z == LOWEST;
}


bool Point::is3d() const
{
    return (z != LOWEST );
}

} //namespace cropfilter

} //namespace pdal

