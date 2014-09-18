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

#include <boost/concept_check.hpp>
#include <boost/concept_check.hpp> // ignore_unused_variable_warning

#include <boost/algorithm/string/trim.hpp>

// gdal
#ifdef PDAL_HAVE_GDAL
#include <ogr_spatialref.h>
#include <cpl_conv.h>
#endif

#include <pdal/Utils.hpp>

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
#ifndef PDAL_SRS_ENABLED
    boost::ignore_unused_variable_warning(mode_flag);
    boost::ignore_unused_variable_warning(pretty);
#else
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
#endif
    return result_wkt;
}


void SpatialReference::setFromUserInput(std::string const& v)
{
#ifdef PDAL_SRS_ENABLED

    char* poWKT = 0;
    const char* input = v.c_str();

    OGRSpatialReference srs(NULL);
    OGRErr err = srs.SetFromUserInput(const_cast<char *>(input));
    if (err != OGRERR_NONE)
        throw std::invalid_argument("could not import coordinate system "
            "into OGRSpatialReference SetFromUserInput");

    srs.exportToWkt(&poWKT);
    std::string tmp(poWKT);
    CPLFree(poWKT);
    setWKT(tmp);
#else
    boost::ignore_unused_variable_warning(v);
    throw std::runtime_error("GDAL is not available, SpatialReference "
        "could not be set from WKT");
#endif
}


std::string SpatialReference::getProj4() const
{
    std::string tmp;

#ifdef PDAL_SRS_ENABLED

    std::string wkt = getWKT(eCompoundOK);
    const char* poWKT = wkt.c_str();

    OGRSpatialReference srs(NULL);
    if (OGRERR_NONE == srs.importFromWkt(const_cast<char **>(&poWKT)))
    {
        char* proj4 = 0;
        srs.exportToProj4(&proj4);
        tmp = proj4;
        CPLFree(proj4);

        boost::algorithm::trim(tmp);
    }
#endif
    return tmp;
}


void SpatialReference::setProj4(std::string const& v)
{
#ifdef PDAL_SRS_ENABLED
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
#else
    boost::ignore_unused_variable_warning(v);
#endif
}


bool SpatialReference::equals(const SpatialReference& input) const
{
#ifdef PDAL_SRS_ENABLED
    OGRSpatialReferenceH current =
        OSRNewSpatialReference(getWKT(eCompoundOK, false).c_str());
    OGRSpatialReferenceH other =
        OSRNewSpatialReference(input.getWKT(eCompoundOK, false).c_str());

    int output = OSRIsSame(current, other);
    OSRDestroySpatialReference(current);
    OSRDestroySpatialReference(other);

    return (output == 1);
#else
    boost::ignore_unused_variable_warning(input);
    throw pdal_error("SpatialReference equality testing not available "
        "without GDAL+libgeotiff support");
#endif
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
#ifdef PDAL_SRS_ENABLED
    OGRSpatialReferenceH current =
        OSRNewSpatialReference(getWKT(eCompoundOK, false).c_str());
    bool output = OSRIsGeographic(current);
    OSRDestroySpatialReference(current);
    return output;
#else
    throw std::runtime_error("GDAL is not available, SpatialReference could "
        "not determine if isGeographic");
#endif
}


void SpatialReference::dump() const
{
    std::cout << *this;
}


std::ostream& operator<<(std::ostream& ostr, const SpatialReference& srs)
{
#ifdef PDAL_SRS_ENABLED
    std::string wkt = pdal::utils::toPTree(srs).get<std::string>("prettycompoundwkt");
    ostr << wkt;
    return ostr;
#else
    boost::ignore_unused_variable_warning(ostr);
    boost::ignore_unused_variable_warning(srs);
    ostr << "SpatialReference data is not available without GDAL+libgeotiff "
        "support";
    return ostr;
#endif
}


std::istream& operator>>(std::istream& istr, SpatialReference& srs)
{
#ifdef PDAL_SRS_ENABLED
    SpatialReference ref;

    std::ostringstream oss;
    oss << istr.rdbuf();

    std::string wkt = oss.str();
    ref.setFromUserInput(wkt.c_str());

    srs = ref;
    return istr;
#else
    boost::ignore_unused_variable_warning(istr);
    boost::ignore_unused_variable_warning(srs);
    throw pdal_error("SpatialReference io operator>> is not available "
        "without GDAL+libgeotiff support");
#endif
}

} // namespace pdal

