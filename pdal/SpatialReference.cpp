/******************************************************************************
 * Copyright (c) 2009, Howard Butler
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <pdal/SpatialReference.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/util/FileUtils.hpp>

// gdal
#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wfloat-equal"
#endif
#ifdef PDAL_COMPILER_GCC
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wfloat-equal"
#endif
#include <ogr_spatialref.h>
#ifdef PDAL_COMPILER_GCC
#  pragma GCC diagnostic pop
#endif
#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic pop
#endif
#include <cpl_conv.h>

#include <pdal/util/Utils.hpp>

namespace pdal
{

SpatialReference::SpatialReference(const std::string& s)
{
    set(s);
}


bool SpatialReference::empty() const
{
    return m_wkt.empty();
}


bool SpatialReference::valid() const
{
    OGRSpatialReferenceH current = OSRNewSpatialReference(m_wkt.c_str());
    if (!current)
        return false;

    OGRErr err = OSRValidate(current);
    OSRDestroySpatialReference(current);
    return err == OGRERR_NONE;
}


std::string SpatialReference::getWKT() const
{
    return m_wkt;
}


void SpatialReference::set(std::string v)
{
    m_horizontalWkt.clear();
    if (v.empty())
    {
        m_wkt.clear();
        return;
    }

    std::string newV = FileUtils::readFileIntoString(v);
    if (newV.size())
        v = newV;

    if (isWKT(v))
    {
        m_wkt = v;
        return;
    }

    OGRSpatialReference srs(NULL);

    CPLErrorReset();
    const char* input = v.c_str();
    OGRErr err = srs.SetFromUserInput(const_cast<char *>(input));
    if (err != OGRERR_NONE)
    {
        std::ostringstream oss;
        std::string msg = CPLGetLastErrorMsg();
        if (msg.empty())
            msg = "(unknown reason)";
        oss << "Could not import coordinate system '" << input << "': " <<
            msg << ".";
        throw pdal_error(oss.str());
    }

    char *poWKT = 0;
    srs.exportToWkt(&poWKT);
    m_wkt = poWKT;
    CPLFree(poWKT);
}


std::string SpatialReference::getProj4() const
{
    std::string tmp;

    const char* poWKT = m_wkt.c_str();

    OGRSpatialReference srs(NULL);
    if (OGRERR_NONE == srs.importFromWkt(const_cast<char **>(&poWKT)))
    {
        char* proj4 = nullptr;
        srs.exportToProj4(&proj4);
        tmp = proj4;
        CPLFree(proj4);
        Utils::trim(tmp);
    }

    return tmp;
}


std::string SpatialReference::getVertical() const
{
    std::string tmp;

    OGRSpatialReference* poSRS =
        (OGRSpatialReference*)OSRNewSpatialReference(m_wkt.c_str());

    // Above can fail if m_wkt is bad.
    if (!poSRS)
        return tmp;

    char *pszWKT = NULL;

    OGR_SRSNode* node = poSRS->GetAttrNode("VERT_CS");
    if (node && poSRS)
    {
        node->exportToWkt(&pszWKT);
        tmp = pszWKT;
        CPLFree(pszWKT);
        OSRDestroySpatialReference(poSRS);
    }

    return tmp;
}


std::string SpatialReference::getVerticalUnits() const
{
    std::string tmp;

    std::string wkt = getVertical();
    const char* poWKT = wkt.c_str();
    OGRSpatialReference* poSRS =
        (OGRSpatialReference*)OSRNewSpatialReference(m_wkt.c_str());
    if (poSRS)
    {
        OGR_SRSNode* node = poSRS->GetAttrNode("VERT_CS");
        if (node)
        {
            char* units(nullptr);

            // 'units' remains internal to the OGRSpatialReference
            // and should not be freed, or modified. It may be invalidated
            // on the next OGRSpatialReference call.
            (void)poSRS->GetLinearUnits(&units);
            tmp = units;
            Utils::trim(tmp);
        }
    }

    return tmp;
}


std::string SpatialReference::getHorizontal() const
{
    if (m_horizontalWkt.empty())
    {
        OGRSpatialReference* poSRS =
            (OGRSpatialReference*)OSRNewSpatialReference(m_wkt.c_str());

        if (poSRS)
        {
            char *pszWKT(nullptr);
            poSRS->StripVertical();
            poSRS->exportToWkt(&pszWKT);
            m_horizontalWkt = pszWKT;
            CPLFree(pszWKT);
            OSRDestroySpatialReference(poSRS);
        }
    }
    return m_horizontalWkt;
}


std::string SpatialReference::getHorizontalUnits() const
{
    std::string wkt = getHorizontal();
    const char* poWKT = wkt.c_str();
    OGRSpatialReference* poSRS =
        (OGRSpatialReference*)OSRNewSpatialReference(m_wkt.c_str());

    if (!poSRS)
        return std::string();

    char* units(nullptr);

    // The returned value remains internal to the OGRSpatialReference
    // and should not be freed, or modified. It may be invalidated on
    // the next OGRSpatialReference call.
    double u = poSRS->GetLinearUnits(&units);
    std::string tmp(units);
    Utils::trim(tmp);
    return tmp;
}


bool SpatialReference::equals(const SpatialReference& input) const
{
    if (getWKT() == input.getWKT())
        return true;

    OGRSpatialReferenceH current = OSRNewSpatialReference(getWKT().c_str());
    OGRSpatialReferenceH other = OSRNewSpatialReference(input.getWKT().c_str());
    if (!current || !other)
        return false;

    int output = OSRIsSame(current, other);
    OSRDestroySpatialReference(current);
    OSRDestroySpatialReference(other);

    return (output == 1);
}


bool SpatialReference::operator==(const SpatialReference& input) const
{
    return this->equals(input);
}


bool SpatialReference::operator!=(const SpatialReference& input) const
{
    return !(this->equals(input));
}


const std::string& SpatialReference::getName() const
{
    static std::string name("pdal.spatialreference");
    return name;
}


bool SpatialReference::isGeographic() const
{
    OGRSpatialReferenceH current = OSRNewSpatialReference(m_wkt.c_str());
    if (!current)
        return false;

    bool output = OSRIsGeographic(current);
    OSRDestroySpatialReference(current);
    return output;
}


bool SpatialReference::isGeocentric() const
{
    OGRSpatialReferenceH current = OSRNewSpatialReference(m_wkt.c_str());
    if (!current)
        return false;

    bool output = OSRIsGeocentric(current);
    OSRDestroySpatialReference(current);
    return output;
}

int SpatialReference::calculateZone(double lon, double lat)
{
    int zone = 0;
    lon = Utils::normalizeLongitude(lon);

    // Special Norway processing.
    if (lat >= 56.0 && lat < 64.0 && lon >= 3.0 && lon < 12.0 )
        zone = 32;
    // Special Svalbard processing.
    else if (lat >= 72.0 && lat < 84.0)
    {
        if (lon >= 0.0  && lon < 9.0)
            zone = 31;
        else if (lon >= 9.0  && lon < 21.0)
            zone = 33;
        else if (lon >= 21.0  && lon < 33.0)
            zone = 35;
        else if (lon >= 33.0  && lon < 42.0)
            zone = 37;
    }
    // Everywhere else.
    else
    {
        zone = (int) floor((lon + 180.0) / 6) + 1;
        if (lat < 0)
            zone = -zone;
    }

    return zone;
}


bool SpatialReference::isWKT(const std::string& wkt)
{
    // List comes from GDAL.  WKT includes FITTED_CS, but this isn't
    // included in GDAL list.  Not sure why.
    StringList leaders { "PROJCS", "GEOGCS", "COMPD_CS", "GEOCCS",
        "VERT_CS", "LOCAL_CS",
    // New specification names.
        "GEODCRS", "GEODETICCRS",
        "PROJCRS", "PROJECTEDCRS",
        "VERTCRS", "VERITCALCRS",
        "ENGCRS", "ENGINEERINGCRS",
        "IMAGECRS",
        "PARAMETRICCRS",
        "TIMECRS",
        "COMPOUNDCRS"
    };

    for (const std::string& s : leaders)
        if (wkt.compare(0, s.size(), s) == 0)
            return true;
    return false;
}


std::string SpatialReference::prettyWkt(const std::string& wkt)
{
    std::string outWkt;

    OGRSpatialReference *srs =
        (OGRSpatialReference *)OSRNewSpatialReference(wkt.data());
    if (!srs)
        return outWkt;

    char *buf = nullptr;
    srs->exportToPrettyWkt(&buf, FALSE);
    OSRDestroySpatialReference(srs);

    outWkt = buf;
    CPLFree(buf);
    return outWkt;
}


int SpatialReference::computeUTMZone(const BOX3D& box) const
{
    // Nothing we can do if we're an empty SRS
    if (empty())
        return 0;

    OGRSpatialReferenceH current = OSRNewSpatialReference(m_wkt.c_str());
    if (!current)
        throw pdal_error("Could not fetch current SRS");

    OGRSpatialReferenceH wgs84 = OSRNewSpatialReference(0);

    if (OSRSetFromUserInput(wgs84, "EPSG:4326") != OGRERR_NONE)
    {
        OSRDestroySpatialReference(current);
        OSRDestroySpatialReference(wgs84);
        std::ostringstream msg;
        msg << "Could not import GDAL input spatial reference for WGS84";
        throw pdal_error(msg.str());
    }

    void* transform = OCTNewCoordinateTransformation(current, wgs84);

    if (! transform)
    {
        OSRDestroySpatialReference(current);
        OSRDestroySpatialReference(wgs84);
        throw pdal_error("Could not compute transform from "
            "coordinate system to WGS84");
    }

    double minx(0.0), miny(0.0), minz(0.0);
    double maxx(0.0), maxy(0.0), maxz(0.0);

    // OCTTransform modifies values in-place
    minx = box.minx; miny = box.miny; minz = box.minz;
    maxx = box.maxx; maxy = box.maxy; maxz = box.maxz;

    int ret = OCTTransform(transform, 1, &minx, &miny, &minz);
    if (ret == 0)
    {
        OCTDestroyCoordinateTransformation(transform);
        OSRDestroySpatialReference(current);
        OSRDestroySpatialReference(wgs84);
        std::ostringstream msg;
        msg << "Could not project minimum point for computeUTMZone::" <<
            CPLGetLastErrorMsg() << ret;
        throw pdal_error(msg.str());
    }

    ret = OCTTransform(transform, 1, &maxx, &maxy, &maxz);
    if (ret == 0)
    {
        OCTDestroyCoordinateTransformation(transform);
        OSRDestroySpatialReference(current);
        OSRDestroySpatialReference(wgs84);
        std::ostringstream msg;
        msg << "Could not project maximum point for computeUTMZone::" <<
            CPLGetLastErrorMsg() << ret;
        throw pdal_error(msg.str());
    }

    int min_zone(0);
    int max_zone(0);
    min_zone = calculateZone(minx, miny);
    max_zone = calculateZone(maxx, maxy);

    if (min_zone != max_zone)
    {
        OCTDestroyCoordinateTransformation(transform);
        OSRDestroySpatialReference(current);
        OSRDestroySpatialReference(wgs84);
        std::ostringstream msg;
        msg << "Minimum zone is " << min_zone <<"' and maximum zone is '" <<
            max_zone << "'. They do not match because they cross a "
            "zone boundary";
        throw pdal_error(msg.str());
    }

    OCTDestroyCoordinateTransformation(transform);
    OSRDestroySpatialReference(current);
    OSRDestroySpatialReference(wgs84);

    return min_zone;
}


MetadataNode SpatialReference::toMetadata() const
{
    MetadataNode root("srs");
    root.add("horizontal", getHorizontal());
    root.add("vertical", getVertical());
    root.add("isgeographic", isGeographic());
    root.add("isgeocentric", isGeocentric());
    root.add("proj4", getProj4());
    root.add("prettywkt", prettyWkt(getHorizontal()));
    root.add("wkt", getHorizontal());
    root.add("compoundwkt", getWKT());
    root.add("prettycompoundwkt", prettyWkt(m_wkt));

    MetadataNode units = root.add("units");
    units.add("vertical", getVerticalUnits());
    units.add("horizontal", getHorizontalUnits());

    return root;
}


void SpatialReference::dump() const
{
    std::cout << *this;
}


std::ostream& operator<<(std::ostream& ostr, const SpatialReference& srs)
{
    ostr << SpatialReference::prettyWkt(srs.m_wkt);
    return ostr;
}


std::istream& operator>>(std::istream& istr, SpatialReference& srs)
{
    SpatialReference ref;

    std::ostringstream oss;
    oss << istr.rdbuf();
    srs.set(oss.str());

    return istr;
}

} // namespace pdal
