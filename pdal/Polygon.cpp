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

#include <ogr_geometry.h>

namespace pdal
{

Polygon::Polygon()
    : Geometry ()
{
}


Polygon::Polygon(const std::string& wkt_or_json, SpatialReference ref)
    : Geometry(wkt_or_json, ref)
{
}


Polygon::~Polygon()
{
}


Polygon::Polygon(const Polygon& input)
    : Geometry(input)
{
}

Polygon::Polygon(const Geometry& input)
    : Geometry(input)
{
}


Polygon& Polygon::operator=(const Polygon& input)
{

    if (&input!= this)
    {
        m_geoserr = input.m_geoserr;
        m_srs = input.m_srs;
        geos::GeometryDeleter geom_del(m_geoserr);
        GEOSGeomPtr p(GEOSGeom_clone_r(m_geoserr.ctx(), input.m_geom.get()), geom_del);
        m_geom.swap(p);

        prepare();
    }
    return *this;
}


Polygon::Polygon(GEOSGeometry* g, const SpatialReference& srs)
    : Geometry(g, srs)
{
}


Polygon::Polygon(OGRGeometryH g, const SpatialReference& srs) : Geometry(g, srs)
{
    OGRwkbGeometryType t = OGR_G_GetGeometryType(g);

    if (!(t == wkbPolygon ||
        t == wkbMultiPolygon ||
        t == wkbPolygon25D ||
        t == wkbMultiPolygon25D))
    {
        std::ostringstream oss;
        oss << "pdal::Polygon cannot construct geometry because "
            "OGR geometry is not Polygon or MultiPolygon!";
        throw pdal::pdal_error(oss.str());
    }

    OGRGeometry *ogr_g = (OGRGeometry*)g;
    //
    // Convert the the GDAL geom to WKB in order to avoid the version
    // context issues with exporting directoly to GEOS.
    OGRwkbByteOrder bo =
        GEOS_getWKBByteOrder() == GEOS_WKB_XDR ? wkbXDR : wkbNDR;
    int wkbSize = ogr_g->WkbSize();
    std::vector<unsigned char> wkb(wkbSize);

    ogr_g->exportToWkb(bo, wkb.data());
    geos::GeometryDeleter geom_del(m_geoserr);
    GEOSGeomPtr p(GEOSGeomFromWKB_buf_r(m_geoserr.ctx(), wkb.data(), wkbSize), geom_del);
    m_geom.swap(p);
    prepare();

}

Polygon::Polygon(const BOX2D& box) : Geometry ()
{
    BOX3D box3(box.minx, box.miny, 0.0,
               box.maxx, box.maxy, 0.0);
    initializeFromBounds(box3);
}


Polygon::Polygon(const BOX3D& box) : Geometry ()

{
    initializeFromBounds(box);
}


void Polygon::initializeFromBounds(const BOX3D& box)
{
    GEOSCoordSequence* coords = GEOSCoordSeq_create_r(m_geoserr.ctx(), 5, 3);
    auto set_coordinate = [coords, this](int pt_num, const double&x,
        const double& y, const double& z)
    {
        if (!GEOSCoordSeq_setX_r(m_geoserr.ctx(), coords, pt_num, x))
            throw pdal_error("unable to set x for coordinate sequence");
        if (!GEOSCoordSeq_setY_r(m_geoserr.ctx(), coords, pt_num, y))
            throw pdal_error("unable to set y for coordinate sequence");
        if (!GEOSCoordSeq_setZ_r(m_geoserr.ctx(), coords, pt_num, z))
            throw pdal_error("unable to set z for coordinate sequence");
    };

    set_coordinate(0, box.minx, box.miny, box.minz);
    set_coordinate(1, box.minx, box.maxy, box.minz);
    set_coordinate(2, box.maxx, box.maxy, box.maxz);
    set_coordinate(3, box.maxx, box.miny, box.maxz);
    set_coordinate(4, box.minx, box.miny, box.minz);


    GEOSGeometry* ring = GEOSGeom_createLinearRing_r(m_geoserr.ctx(), coords);
    if (!ring)
        throw pdal_error("unable to create linear ring from BOX2D");


    geos::GeometryDeleter geom_del(m_geoserr);
    GEOSGeomPtr p(GEOSGeom_createPolygon_r(m_geoserr.ctx(), ring, 0, 0), geom_del);
    m_geom.swap(p);
    if (!m_geom.get())
        throw pdal_error("unable to create polygon from linear ring in "
            "BOX2D constructor");
    prepare();
}


Polygon Polygon::transform(const SpatialReference& ref) const
{
    if (m_srs.empty())
        throw pdal_error("Polygon::transform failed due to m_srs being empty");
    if (ref.empty())
        throw pdal_error("Polygon::transform failed due to ref being empty");

    gdal::SpatialRef fromRef(m_srs.getWKT());
    gdal::SpatialRef toRef(ref.getWKT());
    gdal::Geometry geom(wkt(12, true), fromRef);
    geom.transform(toRef);
    return Geometry(geom.wkt(), ref);
}


Polygon Polygon::simplify(double distance_tolerance,
    double area_tolerance) const
{
    GEOSGeometry *smoothed =
        GEOSTopologyPreserveSimplify_r(m_geoserr.ctx(), m_geom.get(), distance_tolerance);
    if (!smoothed)
        throw pdal_error("Unable to simplify input geometry!");

    std::vector<GEOSGeometry*> geometries;

    int numGeom = GEOSGetNumGeometries_r(m_geoserr.ctx(), smoothed);
    for (int n = 0; n < numGeom; ++n)
    {
        const GEOSGeometry* m = GEOSGetGeometryN_r(m_geoserr.ctx(), smoothed, n);
        if (!m)
            throw pdal::pdal_error("Unable to Get GeometryN");

        const GEOSGeometry* ering = GEOSGetExteriorRing_r(m_geoserr.ctx(), m);
        if (!ering)
            throw pdal::pdal_error("Unable to Get Exterior Ring");

        GEOSGeometry* exterior = GEOSGeom_clone_r(m_geoserr.ctx(),
            GEOSGetExteriorRing_r(m_geoserr.ctx(), m));
        if (!exterior)
            throw pdal::pdal_error("Unable to clone exterior ring!");

        std::vector<GEOSGeometry*> keep_rings;
        int numRings = GEOSGetNumInteriorRings_r(m_geoserr.ctx(), m);
        for (int i = 0; i < numRings; ++i)
        {
            double area(0.0);

            const GEOSGeometry* iring =
                GEOSGetInteriorRingN_r(m_geoserr.ctx(), m, i);
            if (!iring)
                throw pdal::pdal_error("Unable to Get Interior Ring");

            GEOSGeometry* cring = GEOSGeom_clone_r(m_geoserr.ctx(), iring);
            if (!cring)
                throw pdal::pdal_error("Unable to clone interior ring!");
            GEOSGeometry* aring = GEOSGeom_createPolygon_r(m_geoserr.ctx(), cring,
                NULL, 0);

            int errored = GEOSArea_r(m_geoserr.ctx(), aring, &area);
            if (errored == 0)
                throw pdal::pdal_error("Unable to get area of ring!");
            if (area > area_tolerance)
            {
                keep_rings.push_back(cring);
            }
        }

        GEOSGeometry* p = GEOSGeom_createPolygon_r(m_geoserr.ctx(), exterior,
            keep_rings.data(), keep_rings.size());
        if (p == NULL)
            throw pdal_error("smooth polygon could not be created!" );
        geometries.push_back(p);
    }

    GEOSGeometry* o = GEOSGeom_createCollection_r(m_geoserr.ctx(), GEOS_MULTIPOLYGON,
        geometries.data(), geometries.size());
    Geometry p(o, m_srs);
    GEOSGeom_destroy_r(m_geoserr.ctx(), smoothed);
    GEOSGeom_destroy_r(m_geoserr.ctx(), o);

    return p;
}

double Polygon::area() const
{
    double output(0.0);
    int errored = GEOSArea_r(m_geoserr.ctx(), m_geom.get(), &output);
    if (errored == 0)
        throw pdal::pdal_error("Unable to get area of ring!");
    return output;
}


bool Polygon::covers(PointRef& ref) const
{
    GEOSCoordSequence* coords = GEOSCoordSeq_create_r(m_geoserr.ctx(), 1, 3);
    if (!coords)
        throw pdal_error("Unable to allocate coordinate sequence");

    const double x = ref.getFieldAs<double>(Dimension::Id::X);
    const double y = ref.getFieldAs<double>(Dimension::Id::Y);
    const double z = ref.getFieldAs<double>(Dimension::Id::Z);

    if (!GEOSCoordSeq_setX_r(m_geoserr.ctx(), coords, 0, x))
        throw pdal_error("unable to set x for coordinate sequence");
    if (!GEOSCoordSeq_setY_r(m_geoserr.ctx(), coords, 0, y))
        throw pdal_error("unable to set y for coordinate sequence");
    if (!GEOSCoordSeq_setZ_r(m_geoserr.ctx(), coords, 0, z))
        throw pdal_error("unable to set z for coordinate sequence");
    GEOSGeometry* p = GEOSGeom_createPoint_r(m_geoserr.ctx(), coords);
    if (!p)
        throw pdal_error("unable to allocate candidate test point");

    bool covers = (bool)(GEOSPreparedCovers_r(m_geoserr.ctx(), m_prepGeom, p));
    GEOSGeom_destroy_r(m_geoserr.ctx(), p);

    return covers;
}

bool Polygon::covers(const Polygon& p) const
{
    return (bool) GEOSCovers_r(m_geoserr.ctx(), m_geom.get(), p.m_geom.get());
}

bool Polygon::overlaps(const Polygon& p) const
{
    return (bool) GEOSOverlaps_r(m_geoserr.ctx(), m_geom.get(), p.m_geom.get());
}

bool Polygon::contains(const Polygon& p) const
{
    return (bool) GEOSContains_r(m_geoserr.ctx(), m_geom.get(), p.m_geom.get());
}

bool Polygon::touches(const Polygon& p) const
{
    return (bool) GEOSTouches_r(m_geoserr.ctx(), m_geom.get(), p.m_geom.get());
}

bool Polygon::within(const Polygon& p) const
{
    return (bool) GEOSWithin_r(m_geoserr.ctx(), m_geom.get(), p.m_geom.get());
}

bool Polygon::crosses(const Polygon& p) const
{
    return (bool) GEOSCrosses_r(m_geoserr.ctx(), m_geom.get(), p.m_geom.get());
}

} // namespace geos
