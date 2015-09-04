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
    setFromUserInput(s);
}


bool SpatialReference::empty() const
{
    return getWKT().empty();
}


std::string SpatialReference::getWKT(WKTModeFlag mode_flag) const
{
    return getWKT(mode_flag, false);
}


/// Fetch the SRS as WKT
std::string SpatialReference::getWKT(WKTModeFlag mode_flag , bool pretty) const
{
    std::string result_wkt = m_wkt;

    if ((mode_flag == eHorizontalOnly
            && strstr(result_wkt.c_str(),"COMPD_CS") != NULL)
            || pretty)
    {
        OGRSpatialReference* poSRS =
            (OGRSpatialReference*)OSRNewSpatialReference(result_wkt.c_str());
        char *pszWKT = NULL;

        if (mode_flag == eHorizontalOnly)
            poSRS->StripVertical();
        if (pretty)
            poSRS->exportToPrettyWkt(&pszWKT, FALSE);
        else
            poSRS->exportToWkt(&pszWKT);

        OSRDestroySpatialReference(poSRS);

        result_wkt = pszWKT;
        CPLFree(pszWKT);
    }

    return result_wkt;
}


void SpatialReference::setFromUserInput(std::string const& v)
{
    char* poWKT = 0;
    const char* input = v.c_str();

    OGRSpatialReference srs(NULL);
    OGRErr err = srs.SetFromUserInput(const_cast<char *>(input));
    if (err != OGRERR_NONE)
    {

        std::ostringstream oss;
        oss << "Could not import coordinate system '" << input << "'";
        oss << " message '" << CPLGetLastErrorMsg() << "'";
        throw pdal_error(oss.str());
    }

    srs.exportToWkt(&poWKT);
    std::string tmp(poWKT);
    CPLFree(poWKT);
    setWKT(tmp);
}


std::string SpatialReference::getProj4() const
{
    std::string tmp;

    std::string wkt = getWKT(eCompoundOK);
    const char* poWKT = wkt.c_str();

    OGRSpatialReference srs(NULL);
    if (OGRERR_NONE == srs.importFromWkt(const_cast<char **>(&poWKT)))
    {
        char* proj4 = 0;
        srs.exportToProj4(&proj4);
        tmp = proj4;
        CPLFree(proj4);

        Utils::trim(tmp);
    }

    return tmp;
}

std::string SpatialReference::getVertical() const
{
    std::string tmp("");

    OGRSpatialReference* poSRS =
        (OGRSpatialReference*)OSRNewSpatialReference(m_wkt.c_str());
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

std::string SpatialReference::getHorizontal() const
{
    std::string tmp("");

    OGRSpatialReference* poSRS =
        (OGRSpatialReference*)OSRNewSpatialReference(m_wkt.c_str());
    char *pszWKT = NULL;

    if (poSRS)
    {
        poSRS->StripVertical();

        poSRS->exportToWkt(&pszWKT);
        tmp = pszWKT;
        CPLFree(pszWKT);
        OSRDestroySpatialReference(poSRS);
    }

    return tmp;
}

void SpatialReference::setProj4(std::string const& v)
{
    char* poWKT = 0;
    const char* poProj4 = v.c_str();

    OGRSpatialReference srs(NULL);
    if (OGRERR_NONE != srs.importFromProj4(const_cast<char *>(poProj4)))
    {
        throw std::invalid_argument("could not import proj4 into OSRSpatialReference SetProj4");
    }

    srs.exportToWkt(&poWKT);
    m_wkt = poWKT;
    CPLFree(poWKT);
}


bool SpatialReference::equals(const SpatialReference& input) const
{
    OGRSpatialReferenceH current =
        OSRNewSpatialReference(getWKT(eCompoundOK, false).c_str());
    OGRSpatialReferenceH other =
        OSRNewSpatialReference(input.getWKT(eCompoundOK, false).c_str());

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
    OGRSpatialReferenceH current =
        OSRNewSpatialReference(getWKT(eCompoundOK, false).c_str());
    bool output = OSRIsGeographic(current);
    OSRDestroySpatialReference(current);
    return output;
}


int SpatialReference::calculateZone(double lon, double lat)
{
    // Force longitude [-180, 180)
    lon = fmod(lon, 360.0);
    if (lon < -180.0)
        lon += 360.0;
    else if (lon >= 180.0)
        lon -= 360.0;

    int zone = 0;

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
        zone = floor((lon + 180.0) / 6) + 1;
        if (lat < 0)
            zone = -zone;
    }

    return zone;
}


int SpatialReference::computeUTMZone(const BOX3D& box) const
{
    // Nothing we can do if we're an empty SRS
    if (empty())
        return 0;

    OGRSpatialReferenceH current =
        OSRNewSpatialReference(getWKT(eHorizontalOnly, false).c_str());
    if (! current)
        throw std::invalid_argument("Could not fetch current SRS");

    OGRSpatialReferenceH wgs84 = OSRNewSpatialReference(0);

    if (OSRSetFromUserInput(wgs84, "EPSG:4326") != OGRERR_NONE)
    {
        OSRDestroySpatialReference(current);
        OSRDestroySpatialReference(wgs84);
        std::ostringstream msg;
        msg << "Could not import GDAL input spatial reference for WGS84";
        throw std::runtime_error(msg.str());
    }

    void* transform = OCTNewCoordinateTransformation(current, wgs84);

    if (! transform)
    {
        OSRDestroySpatialReference(current);
        OSRDestroySpatialReference(wgs84);
        throw std::invalid_argument("could not comput transform from "
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
    std::cerr << "Min X/Y/zone = " << minx << "/" << miny << "/" << min_zone << "!\n";
    max_zone = calculateZone(maxx, maxy);
    std::cerr << "Max X/Y/zone = " << maxx << "/" << maxy << "/" << max_zone << "!\n";

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


void SpatialReference::dump() const
{
    std::cout << *this;
}


std::ostream& operator<<(std::ostream& ostr, const SpatialReference& srs)
{
    std::string wkt = Utils::toPTree(srs).get<std::string>("prettycompoundwkt");
    ostr << wkt;
    return ostr;
}


std::istream& operator>>(std::istream& istr, SpatialReference& srs)
{
    SpatialReference ref;

    std::ostringstream oss;
    oss << istr.rdbuf();

    std::string wkt = oss.str();
    ref.setFromUserInput(wkt.c_str());

    srs = ref;
    return istr;
}

} // namespace pdal
