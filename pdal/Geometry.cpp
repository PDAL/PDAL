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
#include <pdal/GDALUtils.hpp>
#include "private/SrsTransform.hpp"

namespace pdal
{

void Geometry::throwNoGeos()
{
    if (!OGRGeometryFactory::haveGEOS())
        throw pdal_error("PDAL must be using a version of GDAL built with "
            "GEOS support to use this function.");
}


Geometry::Geometry()
{}


Geometry::Geometry(const std::string& wkt_or_json, SpatialReference ref)
{
    update(wkt_or_json);
    if (ref.valid())
        setSpatialReference(ref);
}


Geometry::Geometry(const Geometry& input) : m_geom(input.m_geom->clone())
{}


Geometry::Geometry(Geometry&& input) : m_geom(std::move(input.m_geom))
{}


Geometry::Geometry(OGRGeometryH g) :
    m_geom((reinterpret_cast<OGRGeometry *>(g))->clone())
{}


Geometry::Geometry(OGRGeometryH g, const SpatialReference& srs) :
    m_geom((reinterpret_cast<OGRGeometry *>(g))->clone())
{
    setSpatialReference(srs);
}


Geometry::~Geometry()
{}


void Geometry::modified()
{}


void Geometry::update(const std::string& wkt_or_json)
{
    bool isJson = (wkt_or_json.find("{") != wkt_or_json.npos) ||
                  (wkt_or_json.find("}") != wkt_or_json.npos);

    OGRGeometry *newGeom;
    std::string srs;
    if (isJson)
    {
        newGeom = gdal::createFromGeoJson(wkt_or_json, srs);
        if (!newGeom)
            throw pdal_error("Unable to create geometry from input GeoJSON");
    }
    else
    {
        newGeom = gdal::createFromWkt(wkt_or_json, srs);
        if (!newGeom)
            throw pdal_error("Unable to create geometry from input WKT");
    }

    // m_geom may be null if update() is called from a ctor.
    if (newGeom->getSpatialReference() && srs.size())
        throw pdal_error("Geometry contains spatial reference and one was "
            "also provided following the geometry specification.");
    if (!newGeom->getSpatialReference() && srs.size())
        newGeom->assignSpatialReference(
            new OGRSpatialReference(SpatialReference(srs).getWKT().data()));
    // m_geom may be null if update() is called from a ctor.
    else if (m_geom)
        newGeom->assignSpatialReference(m_geom->getSpatialReference());
    m_geom.reset(newGeom);
    modified();
}


Geometry& Geometry::operator=(const Geometry& input)
{
    if (m_geom != input.m_geom)
        *m_geom = *input.m_geom;
    modified();
    return *this;
}


bool Geometry::srsValid() const
{
    OGRSpatialReference *srs = m_geom->getSpatialReference();
    return srs && srs->GetRoot();
}


void Geometry::transform(const SpatialReference& out)
{
    if (!srsValid() && out.empty())
        return;

    if (!srsValid())
        throw pdal_error("Geometry::transform() failed.  NULL source SRS.");
    if (out.empty())
        throw pdal_error("Geometry::transform() failed.  NULL target SRS.");

    SrsTransform transform(getSpatialReference(), out);
    if (m_geom->transform(transform.get()) != OGRERR_NONE)
        throw pdal_error("Geometry::transform() failed.");
    modified();
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
        m_geom->getSpatialReference()->exportToWkt(&buf);
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


std::string Geometry::json(double precision) const
{
    char **papszOptions = NULL;
    std::string p(std::to_string((int)precision));
    papszOptions = CSLSetNameValue(papszOptions, "COORDINATE_PRECISION",
        p.data());

    char* json = OGR_G_ExportToJsonEx(gdal::toHandle(m_geom.get()),
        papszOptions);
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
