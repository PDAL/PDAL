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

#include <pdal/Polygon.hpp>
#include "cpl_string.h"

namespace pdal
{

Polygon::Polygon()
    : m_geom(0)
    , m_ctx(pdal::GlobalEnvironment::get().geos())
{

}

Polygon::Polygon(const std::string& wkt_or_json,
                   SpatialReference ref,
                   geos::ErrorHandler& ctx)
    : m_geom(0)
    , m_srs(ref)
    , m_ctx(ctx)
{
    update(wkt_or_json, ref);
}

void Polygon::update(const std::string& wkt_or_json,
                   SpatialReference ref)
{
    bool isJson = wkt_or_json.find("{") != wkt_or_json.npos ||
                  wkt_or_json.find("}") != wkt_or_json.npos;

    if (!isJson)
    {
        m_geom = GEOSGeomFromWKT_r(m_ctx.ctx, wkt_or_json.c_str());
        if (!m_geom)
        {
            throw pdal_error("Unable to create geometry from input WKT");
        }
    }
    else
    {
        // Assume it is GeoJSON and try constructing from that
        OGRGeometryH json = OGR_G_CreateGeometryFromJson(wkt_or_json.c_str());

        char* gdal_wkt(0);
        if (!json)
        {
            throw pdal_error("Unable to create geometry from input GeoJSON/WKT");
        }

        OGRErr err = OGR_G_ExportToWkt(json, &gdal_wkt);
        m_geom = GEOSGeomFromWKT_r(m_ctx.ctx, gdal_wkt);
        if (!m_geom)
        {
            throw pdal_error("Unable to create GEOS geometry from OGR WKT!");
        }
        OGRFree(gdal_wkt);
    }

    m_prepGeom = GEOSPrepare_r(m_ctx.ctx, m_geom);
    if (!m_prepGeom)
        throw pdal_error("unable to prepare geometry for index-accelerated "
            "intersection");

}

Polygon::Polygon(const Polygon& input)
    : m_geom(GEOSGeom_clone_r(input.m_ctx.ctx, input.m_geom))
    , m_srs(input.m_srs)
    , m_ctx(input.m_ctx)
{

}

Polygon::Polygon(GEOSGeometry* g, const SpatialReference& srs, geos::ErrorHandler& ctx)
    : m_geom(GEOSGeom_clone_r(ctx.ctx, g))
    , m_srs(srs)
    , m_ctx(ctx)
{
}

Polygon Polygon::simplify(double distance_tolerance, double area_tolerance) const
{
    GEOSGeometry *smoothed = GEOSTopologyPreserveSimplify_r(m_ctx.ctx, m_geom, distance_tolerance);
    if (!smoothed)
        throw pdal_error("Unable to simplify input geometry!");

    std::vector<GEOSGeometry*> geometries;

    int numGeom = GEOSGetNumGeometries_r(m_ctx.ctx, smoothed);
    for (int n = 0; n < numGeom; ++n)
    {
        const GEOSGeometry* m = GEOSGetGeometryN_r(m_ctx.ctx, smoothed, n);
        if (!m)
            throw pdal::pdal_error("Unable to Get GeometryN");

        const GEOSGeometry* ering = GEOSGetExteriorRing_r(m_ctx.ctx, m);
        if (!ering)
            throw pdal::pdal_error("Unable to Get Exterior Ring");

        GEOSGeometry* exterior = GEOSGeom_clone_r(m_ctx.ctx, GEOSGetExteriorRing_r(m_ctx.ctx, m));
        if (!exterior)
            throw pdal::pdal_error("Unable to clone exterior ring!");

        std::vector<GEOSGeometry*> keep_rings;
        int numRings = GEOSGetNumInteriorRings_r(m_ctx.ctx, m);
        for (int i = 0; i < numRings; ++i)
        {
            double area(0.0);

            const GEOSGeometry* iring = GEOSGetInteriorRingN_r(m_ctx.ctx, m, i);
            if (!iring)
                throw pdal::pdal_error("Unable to Get Interior Ring");

            GEOSGeometry* cring = GEOSGeom_clone_r(m_ctx.ctx, iring);
            if (!cring)
                throw pdal::pdal_error("Unable to clone interior ring!");
            GEOSGeometry* aring = GEOSGeom_createPolygon_r(m_ctx.ctx, cring, NULL, 0);

            int errored = GEOSArea_r(m_ctx.ctx, aring, &area);
            if (errored == 0)
                throw pdal::pdal_error("Unable to get area of ring!");
            if (area > area_tolerance)
            {
                keep_rings.push_back(cring);
            }
        }

        GEOSGeometry* p = GEOSGeom_createPolygon_r(m_ctx.ctx,exterior,
                                                   keep_rings.data(),
                                                   keep_rings.size());
        if (p == NULL) throw
            pdal::pdal_error("smooth polygon could not be created!" );
        geometries.push_back(p);
    }

    GEOSGeometry* o = GEOSGeom_createCollection_r(m_ctx.ctx,
                                                  GEOS_MULTIPOLYGON,
                                                  geometries.data(),
                                                  geometries.size());
    Polygon p(o, m_srs, m_ctx);
    GEOSGeom_destroy_r(m_ctx.ctx, smoothed);
    GEOSGeom_destroy_r(m_ctx.ctx, o);

    return p;

//     GEOSWKTWriter *writer = GEOSWKTWriter_create_r(m_ctx.ctx);
//     GEOSWKTWriter_setRoundingPrecision_r(m_ctx.ctx, writer, precision);
//
//     char *smoothWkt = GEOSWKTWriter_write_r(m_ctx.ctx, writer, o);
//
//     std::string output(smoothWkt);
//     GEOSFree_r(m_ctx.ctx, smoothWkt);
//     GEOSWKTWriter_destroy_r(m_ctx.ctx, writer);
//     GEOSGeom_destroy_r(m_ctx.ctx, smoothed);


}

double Polygon::area() const
{
    double output(0.0);
    int errored = GEOSArea_r(m_ctx.ctx, m_geom, &output);
    if (errored == 0)
        throw pdal::pdal_error("Unable to get area of ring!");
    return output;
}

bool Polygon::covers(PointRef& ref) const
{
    GEOSCoordSequence* coords = GEOSCoordSeq_create_r(m_ctx.ctx, 1, 3);
    if (!coords)
        throw pdal_error("Unable to allocate coordinate sequence");

    const double x = ref.getFieldAs<double>(Dimension::Id::X);
    const double y = ref.getFieldAs<double>(Dimension::Id::Y);
    const double z = ref.getFieldAs<double>(Dimension::Id::Z);

    if (!GEOSCoordSeq_setX_r(m_ctx.ctx, coords, 0, x))
        throw pdal_error("unable to set x for coordinate sequence");
    if (!GEOSCoordSeq_setY_r(m_ctx.ctx, coords, 0, y))
        throw pdal_error("unable to set y for coordinate sequence");
    if (!GEOSCoordSeq_setZ_r(m_ctx.ctx, coords, 0, z))
        throw pdal_error("unable to set z for coordinate sequence");
    GEOSGeometry* p = GEOSGeom_createPoint_r(m_ctx.ctx, coords);
    if (!p)
        throw pdal_error("unable to allocate candidate test point");

    bool covers = (bool)(GEOSPreparedCovers_r(m_ctx.ctx, m_prepGeom, p));
    GEOSGeom_destroy_r(m_ctx.ctx, p);

    return covers;

}

BOX3D Polygon::bounds() const
{

    uint32_t numInputDims;
    BOX3D output;


    GEOSGeometry const* ring = GEOSGetExteriorRing_r(m_ctx.ctx,
        m_geom);
    GEOSCoordSequence const* coords = GEOSGeom_getCoordSeq_r(m_ctx.ctx,
        ring);

    GEOSCoordSeq_getDimensions_r(m_ctx.ctx, coords, &numInputDims);

    uint32_t count(0);
    GEOSCoordSeq_getSize_r(m_ctx.ctx, coords, &count);

    double x(0.0);
    double y(0.0);
    double z(0.0);
    for (unsigned i = 0; i < count; ++i)
    {
        GEOSCoordSeq_getOrdinate_r(m_ctx.ctx, coords, i, 0, &x);
        GEOSCoordSeq_getOrdinate_r(m_ctx.ctx, coords, i, 1, &y);
        if (numInputDims > 2)
            GEOSCoordSeq_getOrdinate_r(m_ctx.ctx, coords, i, 2, &z);
        output.grow(x, y, z);
    }
    return output;


}

bool Polygon::equals(const Polygon& p, double tolerance) const
{

    char c = GEOSEqualsExact_r(m_ctx.ctx, m_geom, p.m_geom, tolerance);
    return (bool) c;
}

bool Polygon::operator==(const Polygon& input) const
{
    return this->equals(input);
}


bool Polygon::operator!=(const Polygon& input) const
{
    return !(this->equals(input));
}

bool Polygon::valid() const
{
    int gtype = GEOSGeomTypeId_r(m_ctx.ctx, m_geom);
    if (gtype != GEOS_POLYGON && gtype != GEOS_MULTIPOLYGON)
        return false;

    return (bool)GEOSisValid_r(m_ctx.ctx, m_geom);
}

std::string Polygon::validReason() const
{
    int gtype = GEOSGeomTypeId_r(m_ctx.ctx, m_geom);
    if (gtype != GEOS_POLYGON && gtype != GEOS_MULTIPOLYGON)
        return std::string("Geometry is not Polygon or MultiPolygon");

    char *reason = GEOSisValidReason_r(m_ctx.ctx, m_geom);
    std::string output(reason);
    GEOSFree_r(m_ctx.ctx, reason);
    return output;
}

std::string Polygon::wkt(double precision) const
{

    GEOSWKTWriter *writer = GEOSWKTWriter_create_r(m_ctx.ctx);
    GEOSWKTWriter_setRoundingPrecision_r(m_ctx.ctx, writer, precision);
    char *smoothWkt = GEOSWKTWriter_write_r(m_ctx.ctx, writer, m_geom);
    std::string output(smoothWkt);
    GEOSFree_r(m_ctx.ctx, smoothWkt);
    GEOSWKTWriter_destroy_r(m_ctx.ctx, writer);
    return output;

}

std::string Polygon::json(double precision) const
{
    std::ostringstream prec;
    prec << precision;
    char **papszOptions = NULL;
    papszOptions = CSLSetNameValue( papszOptions, "COORDINATE_PRECISION", prec.str().c_str() );

    std::string w(wkt());

    pdal::gdal::SpatialRef srs(m_srs.getWKT(pdal::SpatialReference::eCompoundOK));
    pdal::gdal::Geometry g(w, srs);

    char* json = OGR_G_ExportToJsonEx(g.get(), papszOptions);

    std::string output(json);
    OGRFree(json);
    return output;
}

Polygon::~Polygon()
{
    GEOSGeom_destroy_r(m_ctx.ctx, m_geom);
}

std::ostream& operator<<(std::ostream& ostr, const Polygon& p)
{
    ostr << p.wkt();
    return ostr;
}


std::istream& operator>>(std::istream& istr, Polygon& p)
{

    std::ostringstream oss;
    oss << istr.rdbuf();

    std::string wkt = oss.str();

    p.update(wkt);
    return istr;
}

} // namespace geos
