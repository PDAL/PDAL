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

#include <memory>

#include <pdal/Metadata.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/private/SrsTransform.hpp>
#include <pdal/util/FileUtils.hpp>

// gdal
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wfloat-equal"
#include <ogr_spatialref.h>
#  pragma GCC diagnostic pop

#include <cpl_conv.h>

#include <pdal/util/Utils.hpp>

namespace
{

struct OGRDeleter
{
    void operator()(OGRSpatialReference* o)
    {
        OSRDestroySpatialReference(o);
    };
};

using OGRScopedSpatialReference =
    std::unique_ptr<OGRSpatialReference, OGRDeleter>;

OGRScopedSpatialReference ogrCreateSrs(std::string s = "")
{
    return OGRScopedSpatialReference(
        static_cast<OGRSpatialReference*>(
            OSRNewSpatialReference(s.size() ? s.c_str() : nullptr)));
}

}

namespace pdal
{

SpatialReference::SpatialReference(const std::string& s)
{
    set(s);
}


//NOTE that this ctor allows a string constant to be used in places
// where a SpatialReference is extpected.
SpatialReference::SpatialReference(const char *s)
{
    set(s);
}


bool SpatialReference::empty() const
{
    return m_wkt.empty();
}


bool SpatialReference::valid() const
{
    OGRSpatialReference current(m_wkt.data());

    return OSRValidate(&current) == OGRERR_NONE;
}


std::string SpatialReference::identifyHorizontalEPSG() const
{
    OGRScopedSpatialReference srs(ogrCreateSrs(getHorizontal()));

    if (!srs || srs->AutoIdentifyEPSG() != OGRERR_NONE)
        return "";

    if (const char* c = srs->GetAuthorityCode(nullptr))
        return std::string(c);

    return "";
}


std::string SpatialReference::identifyVerticalEPSG() const
{
    OGRScopedSpatialReference srs(ogrCreateSrs(getVertical()));

    if (!srs || srs->AutoIdentifyEPSG() != OGRERR_NONE)
        return "";

    if (const char* c = srs->GetAuthorityCode(nullptr))
        return std::string(c);

    return "";
}


std::string SpatialReference::getWKT() const
{
    return m_wkt;
}


void SpatialReference::parse(const std::string& s, std::string::size_type& pos)
{
    set(s.substr(pos));
}


void SpatialReference::set(std::string v)
{
    m_horizontalWkt.clear();
    if (v.empty())
    {
        m_wkt.clear();
        return;
    }

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

    OGRScopedSpatialReference poSRS = ogrCreateSrs(m_wkt);

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
    }

    return tmp;
}


std::string SpatialReference::getVerticalUnits() const
{
    std::string tmp;

    OGRScopedSpatialReference poSRS = ogrCreateSrs(m_wkt);
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
        OGRScopedSpatialReference poSRS = ogrCreateSrs(m_wkt);

        if (poSRS)
        {
            char *pszWKT(nullptr);
            poSRS->StripVertical();
            poSRS->exportToWkt(&pszWKT);
            m_horizontalWkt = pszWKT;
            CPLFree(pszWKT);
        }
    }
    return m_horizontalWkt;
}


std::string SpatialReference::getHorizontalUnits() const
{
    OGRScopedSpatialReference poSRS = ogrCreateSrs(m_wkt);

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

    OGRScopedSpatialReference current = ogrCreateSrs(getWKT());
    OGRScopedSpatialReference other = ogrCreateSrs(input.getWKT());

    if (!current || !other)
        return false;

    int output = OSRIsSame(current.get(), other.get());

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
    OGRScopedSpatialReference current = ogrCreateSrs(m_wkt);
    if (!current)
        return false;

    bool output = OSRIsGeographic(current.get());
    return output;
}


bool SpatialReference::isGeocentric() const
{
    OGRScopedSpatialReference current = ogrCreateSrs(m_wkt);
    if (!current)
        return false;

    bool output = OSRIsGeocentric(current.get());
    return output;
}


bool SpatialReference::isProjected() const
{
    OGRScopedSpatialReference current = ogrCreateSrs(m_wkt);
    if (!current)
        return false;

    bool output = OSRIsProjected(current.get());
    return output;
}

std::vector<int> SpatialReference::getAxisOrdering() const
{
    std::vector<int> output;
    OGRScopedSpatialReference current = ogrCreateSrs(m_wkt);
    output = current.get()->GetDataAxisToSRSAxisMapping();
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


/**
  Create a spatial reference that represents a specific UTM zone.

  \param zone  Zone number.  Must be non-zero and <= 60 and >= -60
  \return  A SpatialReference that represents the specified zone, or
    an invalid SpatialReference on error.
*/
SpatialReference SpatialReference::wgs84FromZone(int zone)
{
    uint32_t abszone(std::abs(zone));

    if (abszone == 0 || abszone > 60)
        return SpatialReference();

    std::string code;
    if (zone > 0)
        code = "EPSG:326";
    else
        code = "EPSG:327";

    code += ((abszone < 10) ? "0" : "") + Utils::toString(abszone);
    return SpatialReference(code);
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

    OGRScopedSpatialReference srs = ogrCreateSrs(wkt);
    if (!srs)
        return outWkt;

    char *buf = nullptr;
    srs->exportToPrettyWkt(&buf, FALSE);

    outWkt = buf;
    CPLFree(buf);
    return outWkt;
}

std::string SpatialReference::getWKT1() const
{
    std::string wkt = getWKT();
    if (wkt.empty())
        return wkt;

    OGRScopedSpatialReference srs = ogrCreateSrs(wkt);
    std::string wkt1;
    if (srs)
    {
        char *buf = nullptr;
        const char* apszOptions[] =
            { "FORMAT=WKT1_GDAL", "ALLOW_ELLIPSOIDAL_HEIGHT_AS_VERTICAL_CRS=YES", nullptr };

        srs->exportToWkt(&buf, apszOptions);
        if (buf)
        {
            wkt1 = buf;
            CPLFree(buf);
        }
    }
    if (wkt1.empty())
        throw pdal_error("Couldn't convert spatial reference to WKT version 1.");
    return wkt1;
}


int SpatialReference::getUTMZone() const
{
    OGRScopedSpatialReference current = ogrCreateSrs(m_wkt);
    if (!current)
        throw pdal_error("Could not fetch current SRS");

    int north(0);
    int zone = OSRGetUTMZone(current.get(), &north);
    return (north ? 1 : -1) * zone;
}


int SpatialReference::computeUTMZone(const BOX3D& cbox) const
{
    SrsTransform transform(*this, SpatialReference("EPSG:4326"));

    // We made the argument constant so copy so that we can modify.
    BOX3D box(cbox);

    transform.transform(box.minx, box.miny, box.minz);
    transform.transform(box.maxx, box.maxy, box.maxz);

    int minZone = calculateZone(box.minx, box.miny);
    int maxZone = calculateZone(box.maxx, box.maxy);

    if (minZone != maxZone)
    {
        std::ostringstream msg;
        msg << "computeUTMZone failed due to zone crossing. Zones "
            "are " << minZone << " and " << maxZone << ".";
        throw pdal_error(msg.str());
    }
    return minZone;
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
    std::ostringstream oss;
    oss << istr.rdbuf();
    srs.set(oss.str());

    return istr;
}

} // namespace pdal
