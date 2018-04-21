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

#include <pdal/Geometry.hpp>
#include "cpl_string.h"

#include <ogr_geometry.h>


namespace pdal
{

Geometry::Geometry()
    : m_prepGeom(0)
    , m_geoserr(geos::ErrorHandler::get())
{
    geos::GeometryDeleter geom_del(m_geoserr);
    GEOSGeomPtr p(GEOSGeom_createEmptyPolygon_r(m_geoserr.ctx()), geom_del);
    m_geom.swap(p);
}


Geometry::Geometry(const std::string& wkt_or_json, SpatialReference ref)
    : m_prepGeom(0)
    , m_srs(ref)
    , m_geoserr(geos::ErrorHandler::get())
{
    update(wkt_or_json);
}


Geometry::Geometry(const Geometry& input)
    : m_srs(input.m_srs)
    , m_geoserr(input.m_geoserr)
{
    assert(input.m_geom.get() != 0);
    geos::GeometryDeleter geom_del(m_geoserr);
    GEOSGeomPtr p(GEOSGeom_clone_r(m_geoserr.ctx(),  input.m_geom.get()),
        geom_del);
    m_geom.swap(p);
    assert(m_geom.get() != 0);
    m_prepGeom = 0;
    prepare();
}


Geometry::Geometry(Geometry&& input) :
    m_geom(std::move(input.m_geom)), m_prepGeom(input.m_prepGeom),
    m_srs(input.m_srs), m_geoserr(input.m_geoserr)
{}


Geometry::~Geometry()
{
    m_geom.reset();
    if (m_prepGeom)
        GEOSPreparedGeom_destroy_r(m_geoserr.ctx(), m_prepGeom);
    m_prepGeom = 0;
}


void Geometry::update(const std::string& wkt_or_json)
{
    bool isJson = wkt_or_json.find("{") != wkt_or_json.npos ||
                  wkt_or_json.find("}") != wkt_or_json.npos;

    GEOSWKTReader* geosreader = GEOSWKTReader_create_r(m_geoserr.ctx());

    if (!isJson)
    {
        geos::GeometryDeleter geom_del(m_geoserr);
        GEOSGeomPtr p(GEOSWKTReader_read_r(m_geoserr.ctx(), geosreader,
            wkt_or_json.c_str()), geom_del);
        m_geom.swap(p);
    }
    else
    {
        // Assume it is GeoJSON and try constructing from that
        OGRGeometryH json = OGR_G_CreateGeometryFromJson(wkt_or_json.c_str());

        if (!json)
            throw pdal_error("Unable to create geometry from "
                "input GeoJSON");

        char* gdal_wkt(0);
        OGRErr err = OGR_G_ExportToWkt(json, &gdal_wkt);

        geos::GeometryDeleter geom_del(m_geoserr);
        GEOSGeomPtr p(GEOSWKTReader_read_r(m_geoserr.ctx(), geosreader,
            gdal_wkt), geom_del);
        m_geom.swap(p);

        OGRFree(gdal_wkt);
        OGR_G_DestroyGeometry(json);
    }
    prepare();

    GEOSWKTReader_destroy_r(m_geoserr.ctx(), geosreader);
}


void Geometry::prepare()
{
    if (m_geom.get())
    {
        m_prepGeom = GEOSPrepare_r(m_geoserr.ctx(), m_geom.get());
        if (!m_prepGeom)
            throw pdal_error("unable to prepare geometry for "
                "index-accelerated access");
    }
}


Geometry& Geometry::operator=(const Geometry& input)
{
    if (&input != this)
    {
        m_geoserr = input.m_geoserr;
        m_srs = input.m_srs;
        geos::GeometryDeleter geom_del(m_geoserr);
        GEOSGeomPtr p(GEOSGeom_clone_r(m_geoserr.ctx(),  input.m_geom.get()),
            geom_del);
        m_geom.swap(p);
        prepare();
    }
    return *this;
}


Geometry::Geometry(GEOSGeometry* g, const SpatialReference& srs)
    : m_srs(srs) , m_geoserr(geos::ErrorHandler::get())
{
    geos::GeometryDeleter geom_del(m_geoserr);
    GEOSGeomPtr p(GEOSGeom_clone_r(m_geoserr.ctx(),  g), geom_del);
    m_geom.swap(p);
    prepare();
}


Geometry::Geometry(OGRGeometryH g, const SpatialReference& srs)
    : m_srs(srs)
    , m_geoserr(geos::ErrorHandler::get())
{

    OGRGeometry *ogr_g = (OGRGeometry*)g;

    // Convert the the GDAL geom to WKB in order to avoid the version
    // context and DLL boundary issues with exporting directoly to GEOS
    // from GDAL
    OGRwkbByteOrder bo =
        GEOS_getWKBByteOrder() == GEOS_WKB_XDR ? wkbXDR : wkbNDR;
    int wkbSize = ogr_g->WkbSize();
    std::vector<unsigned char> wkb(wkbSize);

    ogr_g->exportToWkb(bo, wkb.data());

    GEOSWKBReader* reader = GEOSWKBReader_create_r(m_geoserr.ctx());

    geos::GeometryDeleter geom_del(m_geoserr);
    GEOSGeomPtr p(GEOSWKBReader_read_r(m_geoserr.ctx(),  reader, wkb.data(),
        wkbSize), geom_del);
    m_geom.swap(p);
    prepare();

    GEOSWKBReader_destroy_r(m_geoserr.ctx(), reader);
}


Geometry Geometry::transform(const SpatialReference& ref) const
{
    if (m_srs.empty())
        throw pdal_error("Geometry::transform failed.  "
            "Source missing spatial reference.");
    if (ref.empty())
        throw pdal_error("Geometry::transform failed.  "
            "Invalid destination spatial reference.");
    if (ref == m_srs)
        return *this;

    gdal::SpatialRef fromRef(m_srs.getWKT());
    gdal::SpatialRef toRef(ref.getWKT());
    gdal::Geometry geom(wkt(12, true), fromRef);
    geom.transform(toRef);
    return Geometry(geom.wkt(), ref);
}


BOX3D Geometry::bounds() const
{
    uint32_t numInputDims;
    BOX3D output;

    GEOSGeometry* boundary = GEOSGeom_clone_r(m_geoserr.ctx(), m_geom.get());

    // Smash out multi*
    if (GEOSGeomTypeId_r(m_geoserr.ctx(), m_geom.get()) > 3)
        boundary = GEOSEnvelope_r(m_geoserr.ctx(), m_geom.get());

    GEOSGeometry const* ring = GEOSGetExteriorRing_r(m_geoserr.ctx(), boundary);
    GEOSCoordSequence const* coords = GEOSGeom_getCoordSeq_r(m_geoserr.ctx(), ring);

    GEOSCoordSeq_getDimensions_r(m_geoserr.ctx(), coords, &numInputDims);

    uint32_t count(0);
    GEOSCoordSeq_getSize_r(m_geoserr.ctx(), coords, &count);

    double x(0.0);
    double y(0.0);
    double z(0.0);
    for (unsigned i = 0; i < count; ++i)
    {
        GEOSCoordSeq_getOrdinate_r(m_geoserr.ctx(), coords, i, 0, &x);
        GEOSCoordSeq_getOrdinate_r(m_geoserr.ctx(), coords, i, 1, &y);
        if (numInputDims > 2)
            GEOSCoordSeq_getOrdinate_r(m_geoserr.ctx(), coords, i, 2, &z);
        output.grow(x, y, z);
    }
    GEOSGeom_destroy_r(m_geoserr.ctx(), boundary);

    return output;
}


bool Geometry::equals(const Geometry& p, double tolerance) const
{
    return (bool) GEOSEqualsExact_r(m_geoserr.ctx(), m_geom.get(),
        p.m_geom.get(), tolerance);
}


bool Geometry::operator==(const Geometry& input) const
{
    return this->equals(input);
}


bool Geometry::operator!=(const Geometry& input) const
{
    return !(this->equals(input));
}


bool Geometry::valid() const
{
    int gtype = GEOSGeomTypeId_r(m_geoserr.ctx(), m_geom.get());
    if (gtype != GEOS_POLYGON && gtype != GEOS_MULTIPOLYGON)
        return false;

    return (bool)GEOSisValid_r(m_geoserr.ctx(), m_geom.get());
}


std::string Geometry::validReason() const
{
    char *reason = GEOSisValidReason_r(m_geoserr.ctx(), m_geom.get());
    std::string output(reason);
    GEOSFree_r(m_geoserr.ctx(), reason);
    return output;
}


std::string Geometry::wkt(double precision, bool bOutputZ) const
{
    GEOSWKTWriter *writer = GEOSWKTWriter_create_r(m_geoserr.ctx());
    GEOSWKTWriter_setRoundingPrecision_r(m_geoserr.ctx(), writer,
        (int)precision);
    if (bOutputZ)
        GEOSWKTWriter_setOutputDimension_r(m_geoserr.ctx(), writer, 3);

    char *smoothWkt = GEOSWKTWriter_write_r(m_geoserr.ctx(), writer,
        m_geom.get());
    std::string output(smoothWkt);
    GEOSFree_r(m_geoserr.ctx(), smoothWkt);
    GEOSWKTWriter_destroy_r(m_geoserr.ctx(), writer);
    return output;
}


std::string Geometry::json(double precision) const
{
    std::ostringstream prec;
    prec << precision;
    char **papszOptions = NULL;
    papszOptions = CSLSetNameValue(papszOptions, "COORDINATE_PRECISION",
        prec.str().c_str() );

    gdal::SpatialRef srs(m_srs.getWKT());
    gdal::Geometry g(wkt(), srs);

    char* json = OGR_G_ExportToJsonEx(g.get(), papszOptions);
    std::string output(json);
    OGRFree(json);
    return output;
}


std::ostream& operator<<(std::ostream& ostr, const Geometry& p)
{
    ostr << p.wkt();
    return ostr;
}


std::istream& operator>>(std::istream& istr, Geometry& p)
{
    std::ostringstream oss;
    oss << istr.rdbuf();

    try
    {
        p.update(oss.str());
    }
    catch (pdal_error& )
    {
        istr.setstate(std::ios::failbit);
    }
    return istr;
}

} // namespace geos
