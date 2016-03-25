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
*     * Neither the name of Hobu, Inc. nor the
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
#pragma once

#include <pdal/pdal_types.hpp>
#include <pdal/Log.hpp>
#include <pdal/PointRef.hpp>
#include <pdal/GDALUtils.hpp>
#include <pdal/GEOSUtils.hpp>
#include <pdal/util/Bounds.hpp>

#include <pdal/SpatialReference.hpp>
#include <pdal/GlobalEnvironment.hpp>

namespace pdal
{

namespace geos { class ErrorHandler; }

class PDAL_DLL Polygon
{
public:
    Polygon();
    Polygon(const std::string& wkt_or_json,
           SpatialReference ref = SpatialReference(),
           geos::ErrorHandler& ctx = pdal::GlobalEnvironment::get().geos());
    Polygon(const BOX2D&);
    Polygon(const BOX3D&);
    Polygon(const Polygon&);
    Polygon(GEOSGeometry* g, const SpatialReference& srs,
        geos::ErrorHandler& ctx);
    Polygon(OGRGeometryH g, const SpatialReference& srs,
        geos::ErrorHandler& ctx);
    Polygon& operator=(const Polygon&);
private:
    Polygon(const std::string& wkt_or_json, SpatialReference ref,
        GEOSContextHandle_t ctx);
    Polygon(GEOSGeometry* g, const SpatialReference& srs,
        GEOSContextHandle_t ctx);

public:
    ~Polygon();
    void update(const std::string& wkt_or_json,
        SpatialReference ref = SpatialReference());

    void setSpatialReference( const SpatialReference& ref)
        { m_srs = ref; }

    const SpatialReference& getSpatialReference() const
        { return m_srs; }

    Polygon transform(const SpatialReference& ref) const;

    bool equals(const Polygon& other, double tolerance=0.0001) const;
    bool operator==(const Polygon& other) const;
    bool operator!=(const Polygon& other) const;
    bool operator<(const Polygon& other) const
        { return wkt() < other.wkt(); }

    Polygon simplify(double distance_tolerance, double area_tolerance) const;
    double area() const;

    bool covers(PointRef& ref) const;
    bool equal(const Polygon& p) const;

    bool valid() const;
    std::string validReason() const;

    std::string wkt(double precision=8, bool bOutputZ=false) const;
    std::string json(double precision=8) const;

    BOX3D bounds() const;

    operator bool () const
        { return m_geom != NULL; }

private:
    void initializeFromBounds(const BOX3D& b);
    GEOSGeometry *m_geom;
    const GEOSPreparedGeometry *m_prepGeom;

    SpatialReference m_srs;
    GEOSContextHandle_t m_ctx;

    void prepare();

    friend PDAL_DLL std::ostream& operator<<(std::ostream& ostr,
        const Polygon& p);
    friend PDAL_DLL std::istream& operator>>(std::istream& istr,
        Polygon& p);
};


PDAL_DLL std::ostream& operator<<(std::ostream& ostr,
    const Polygon& p);
PDAL_DLL std::istream& operator>>(std::istream& istr, Polygon& p);

} // namespace pdal

