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

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

#include <ogr_api.h>
#include <ogr_geometry.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <pdal/Geometry.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>

#include "private/SrsTransform.hpp"

namespace pdal
{

void Geometry::throwNoGeos()
{
    if (!OGRGeometryFactory::haveGEOS())
        throw pdal_error("PDAL must be using a version of GDAL built with "
            "GEOS support to use this function.");
}


Geometry::Geometry(const std::string& wkt_or_json, SpatialReference ref)
{
    update(wkt_or_json);
    if (ref.valid())
        setSpatialReference(ref);
}


Geometry::Geometry(const Geometry& input)
{
    if (input.m_geom)
        m_geom.reset(input.m_geom->clone());
}


Geometry::Geometry(Geometry&& input) : m_geom(std::move(input.m_geom))
{}


Geometry::Geometry(double x, double y, double z, SpatialReference ref)
{
    OGRGeometry* geom(nullptr);
    OGRPoint point(x, y, z);
    geom = reinterpret_cast<OGRGeometry *>(&point);

    if (geom)
        m_geom.reset(geom->clone());

    setSpatialReference(ref);
}

void Geometry::OGRGeometryDeleter::operator()(OGRGeometry *geom)
{
    delete geom;
}

void Geometry::construct(void *g)
{
    OGRGeometry* geom(nullptr);
    geom = reinterpret_cast<OGRGeometry *>(g);

    if (geom)
        m_geom.reset(geom->clone());
}


void Geometry::construct(void *g, const SpatialReference& srs)
{
    OGRGeometry* geom(nullptr);
    geom = reinterpret_cast<OGRGeometry *>(g);

    if (geom)
        m_geom.reset(geom->clone());

    setSpatialReference(srs);
}


void Geometry::modified()
{}


void Geometry::update(const std::string& wkt_or_json)
{
    bool isJson = (wkt_or_json.find("{") != wkt_or_json.npos) ||
                  (wkt_or_json.find("}") != wkt_or_json.npos);

    bool maybeWkt = (wkt_or_json.find("(") != wkt_or_json.npos) ||
                    (wkt_or_json.find(")") != wkt_or_json.npos);

    // first byte is 00 or 01
    bool maybeWkb = (wkt_or_json[0] == 0 || wkt_or_json[0] == 1);

    OGRGeometry *newGeom (nullptr);
    std::string srs;

    if (maybeWkb)
    {
        // assume WKB
        newGeom = gdal::createFromWkb(wkt_or_json, srs);
        if (!newGeom)
            throw pdal_error("Unable to create geometry from input WKB");

        if (!newGeom->getSpatialReference() && srs.size())
            newGeom->assignSpatialReference(
                new OGRSpatialReference(SpatialReference(srs).getWKT().data()));

    }
    else if (maybeWkt)
    {
        newGeom = gdal::createFromWkt(wkt_or_json, srs);
        if (!newGeom)
            throw pdal_error("Unable to create geometry from input WKT");

        if (!newGeom->getSpatialReference() && srs.size())
            newGeom->assignSpatialReference(
                new OGRSpatialReference(SpatialReference(srs).getWKT().data()));
    }
    else if (isJson)
    {
        // createFromGeoJson may set the geometry's SRS for us
        // because GeoJSON is 4326. If the user provided a 'srs'
        // node, we're going to override with that, however
        newGeom = gdal::createFromGeoJson(wkt_or_json, srs);
        if (!newGeom)
            throw pdal_error("Unable to create geometry from input GeoJSON");

        if (srs.size())
            newGeom->assignSpatialReference(
                new OGRSpatialReference(SpatialReference(srs).getWKT().data()));
    }
    if (!newGeom)
        throw pdal_error("Unable to create geometry from unknown input string.");

    m_geom.reset(newGeom);
    modified();
}


Geometry& Geometry::operator=(const Geometry& input)
{
    if (m_geom != input.m_geom)
        m_geom.reset(input.m_geom->clone());
    modified();
    return *this;
}


bool Geometry::srsValid() const
{
    const OGRSpatialReference *srs = m_geom->getSpatialReference();
    return srs && srs->GetRoot();
}


Utils::StatusWithReason Geometry::transform(SpatialReference out)
{
    using namespace Utils;

    if (!srsValid() && out.empty())
        return StatusWithReason();

    if (!srsValid())
        return StatusWithReason(-2,
            "Geometry::transform() failed.  NULL source SRS.");
    if (out.empty())
        return StatusWithReason(-2,
            "Geometry::transform() failed.  NULL target SRS.");

    const OGRSpatialReference *inSrs = m_geom->getSpatialReference();
    SrsTransform transform(*inSrs, OGRSpatialReference(out.getWKT().data()));
    if (!transform.valid() || m_geom->transform(transform.get()) != OGRERR_NONE)
        return StatusWithReason(-1, "Geometry::transform() failed.");

    modified();
    return StatusWithReason();
}


void Geometry::setSpatialReference(const SpatialReference& srs)
{
    OGRSpatialReference *oSrs;

    if (!srs.valid())
        oSrs = new OGRSpatialReference();
    else
        oSrs = new OGRSpatialReference(srs.getWKT().data());
    m_geom->assignSpatialReference(oSrs);
    oSrs->Release();
}


SpatialReference Geometry::getSpatialReference() const
{
    SpatialReference srs;

    if (srsValid())
    {
        char *buf;
        const char *options[] = { "FORMAT=WKT2", nullptr };
        m_geom->getSpatialReference()->exportToWkt(&buf, options);
        srs.set(buf);
        CPLFree(buf);
    }
    return srs;
}


BOX3D Geometry::bounds() const
{
    OGREnvelope3D env;
    m_geom->getEnvelope(&env);
    return BOX3D(env.MinX, env.MinY, env.MinZ,
        env.MaxX, env.MaxY, env.MaxZ);
}


double Geometry::distance(double x, double y, double z) const
{
    OGRPoint p(x, y, z);
    throwNoGeos();
    if(!m_geom)
        throw pdal_error("Cannot compare distance of null geometry!");
    return m_geom->Distance((OGRGeometry*)&p);
}

Geometry Geometry::getRing() const
{
    throwNoGeos();

    int count = OGR_G_GetGeometryCount(gdal::toHandle(m_geom.get()));
    if (count)
    {

        OGRGeometryH ring = OGR_G_Clone(OGR_G_GetGeometryRef(gdal::toHandle(m_geom.get()), 0));
        OGRGeometryH linestring = OGR_G_ForceToLineString(ring);

        return Geometry(linestring, getSpatialReference());
    }
    else
        throwNoGeos();

    return Geometry();

}

bool Geometry::valid() const
{
    throwNoGeos();

    return (bool)m_geom->IsValid();
}


std::string Geometry::wkt(double precision, bool bOutputZ) const
{
    // Important note: The precision is not always respected.  Using GDAL
    // it can only be set once.  Because of this, there's no point in saving
    // away the current OGR_WKT_PRECISION.  Same for OGR_WKT_ROUND.
    //
    // Also note that when abs(value) < 1, f-type formatting is used.
    // Otherwise g-type formatting is used.  Precision means different things
    // with the two format types.  With f-formatting it specifies the
    // number of places to the right of the decimal.  In g-formatting, it's
    // the minimum number of digits.  Yuck.

    std::string p(std::to_string((int)precision));
    CPLSetConfigOption("OGR_WKT_PRECISION", p.data());
    CPLSetConfigOption("OGR_WKT_ROUND", "FALSE");

    char *buf;
    OGRErr err = m_geom->exportToWkt(&buf);
    if (err != OGRERR_NONE)
        throw pdal_error("Geometry::wkt: unable to export geometry to WKT.");
    std::string wkt(buf);
    CPLFree(buf);
    return wkt;
}

std::string Geometry::wkb() const
{

    std::string output(m_geom->WkbSize(), '\0');

    OGRErr err = m_geom->exportToWkb(wkbNDR, (unsigned char*) output.data(), wkbVariantIso);
    if (err != OGRERR_NONE)
        throw pdal_error("Geometry::wkb: unable to export geometry to wkb.");

    return output;
}



std::string Geometry::json(double precision) const
{
    CPLStringList aosOptions;
    std::string p(std::to_string((int)precision));
    aosOptions.SetNameValue("COORDINATE_PRECISION", p.data());

    char* json = OGR_G_ExportToJsonEx(gdal::toHandle(m_geom.get()),
        aosOptions.List());
    std::string output(json);
    OGRFree(json);
    return output;
}


void Geometry::clear()
{
    m_geom.reset();
}


std::ostream& operator<<(std::ostream& ostr, const Geometry& p)
{
    ostr << p.wkt();
    return ostr;
}


std::istream& operator>>(std::istream& istr, Geometry& p)
{
    // Read stream into string.
    std::string s(std::istreambuf_iterator<char>(istr), {});

    try
    {
        p.update(s);
    }
    catch (pdal_error& )
    {
        istr.setstate(std::ios::failbit);
    }
    return istr;
}

} // namespace pdal
