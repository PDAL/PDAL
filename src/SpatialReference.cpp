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
#include <pdal/exceptions.hpp>

#include <boost/concept_check.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/concept_check.hpp> // ignore_unused_variable_warning


// gdal
#ifdef PDAL_HAVE_GDAL
#include <ogr_spatialref.h>
#include <cpl_conv.h>
#endif

#include <pdal/Utils.hpp>

namespace pdal
{
   
SpatialReference::SpatialReference()
    : m_wkt("")
{
    return;
}


SpatialReference::SpatialReference(const std::string& s)
    : m_wkt("")
{
    this->setFromUserInput(s);
    return;
}


SpatialReference::SpatialReference(SpatialReference const& rhs) 
    : m_wkt(rhs.m_wkt)
{
    return;
}


SpatialReference& SpatialReference::operator=(SpatialReference const& rhs)
{
    if (&rhs != this)
    {
        m_wkt = rhs.m_wkt;
    }
    return *this;
}


SpatialReference::~SpatialReference() 
{
    return;
}


bool SpatialReference::empty() const
{
    return (getWKT() == "");
}


std::string SpatialReference::getWKT( WKTModeFlag mode_flag) const 
{
    return getWKT(mode_flag, false);
}


/// Fetch the SRS as WKT
std::string SpatialReference::getWKT(WKTModeFlag mode_flag , bool pretty) const 
{
#ifndef PDAL_SRS_ENABLED
    boost::ignore_unused_variable_warning(mode_flag);
    boost::ignore_unused_variable_warning(pretty);

    // we don't have a way of making this pretty, or of stripping the compound wrapper.
    return m_wkt;
#else

    std::string result_wkt = m_wkt;

    if( (mode_flag == eHorizontalOnly 
        && strstr(result_wkt.c_str(),"COMPD_CS") != NULL)
        || pretty )
    {
        OGRSpatialReference* poSRS = (OGRSpatialReference*) OSRNewSpatialReference(result_wkt.c_str());
        char *pszWKT = NULL;

        if( mode_flag == eHorizontalOnly )
            poSRS->StripVertical();

        if (pretty) 
            poSRS->exportToPrettyWkt(&pszWKT, FALSE );
        else
            poSRS->exportToWkt( &pszWKT );

        OSRDestroySpatialReference( poSRS );

        result_wkt = pszWKT;
        CPLFree( pszWKT );
    }

    return result_wkt;
#endif
}


void SpatialReference::setFromUserInput(std::string const& v)
{
#ifdef PDAL_SRS_ENABLED

    char* poWKT = 0;
    const char* input = v.c_str();
    
    // OGRSpatialReference* poSRS = (OGRSpatialReference*) OSRNewSpatialReference(NULL);
    OGRSpatialReference srs(NULL);
    OGRErr err = srs.SetFromUserInput(const_cast<char *> (input));
    if (err != OGRERR_NONE)
    {
        throw std::invalid_argument("could not import coordinate system into OSRSpatialReference SetFromUserInput");
    }
    
    srs.exportToWkt(&poWKT);
    
    std::string tmp(poWKT);
    CPLFree(poWKT);
    
    setWKT(tmp);
#else
    boost::ignore_unused_variable_warning(v);
    throw std::runtime_error("GDAL is not available, SpatialReference could not be set from WKT");
#endif
}


void SpatialReference::setWKT(std::string const& v)
{
    m_wkt = v;

    return;
}


std::string SpatialReference::getProj4() const 
{
#ifdef PDAL_SRS_ENABLED
    
    std::string wkt = getWKT(eCompoundOK);
    const char* poWKT = wkt.c_str();
    
    OGRSpatialReference srs(NULL);
    if (OGRERR_NONE != srs.importFromWkt(const_cast<char **> (&poWKT)))
    {
        return std::string();
    }
    
    char* proj4 = 0;
    srs.exportToProj4(&proj4);
    std::string tmp(proj4);
    CPLFree(proj4);
    
    tmp = Utils::trim(tmp);
    return tmp;

#else

    // By default or if we have neither GDAL nor proj.4, we can't do squat
    return std::string();
#endif
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
    
    std::string tmp(poWKT);
    CPLFree(poWKT);

    m_wkt = tmp;

#else
    boost::ignore_unused_variable_warning(v);
#endif

    return;
}


bool SpatialReference::equals(const SpatialReference& input) const
{
#ifdef PDAL_SRS_ENABLED

    OGRSpatialReferenceH current = OSRNewSpatialReference(getWKT(eCompoundOK, false).c_str());
    OGRSpatialReferenceH other = OSRNewSpatialReference(input.getWKT(eCompoundOK, false).c_str());

    int output = OSRIsSame(current, other);

    OSRDestroySpatialReference( current );
    OSRDestroySpatialReference( other );
    
    return (output==1);
    
#else
    boost::ignore_unused_variable_warning(input);
    throw pdal_error ("SpatialReference equality testing not available without GDAL+libgeotiff support");
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


boost::property_tree::ptree SpatialReference::toPTree() const
{
    using boost::property_tree::ptree;
    ptree srs;

#ifdef PDAL_SRS_ENABLED
    srs.put("proj4", getProj4());
    srs.put("prettywkt", getWKT(SpatialReference::eHorizontalOnly, true));
    srs.put("wkt", getWKT(SpatialReference::eHorizontalOnly, false));
    srs.put("compoundwkt", getWKT(eCompoundOK, false));
    srs.put("prettycompoundwkt", getWKT(eCompoundOK, true));

#else

    std::string message;
    if (m_wkt.size() == 0)
    {
        message = "Reference defined with VLR keys, but GeoTIFF and GDAL support are not available to produce definition";
    } 
    else if (m_wkt.size() > 0)
    {
        message = "Reference defined with WKT, but GeoTIFF and GDAL support are not available to produce definition";
    } else
    {
        message = "None";
    }

    srs.put("proj4", message);
    srs.put("prettywkt", message);
    srs.put("wkt", message);
    srs.put("compoundwkt", message);
    srs.put("prettycompoundwkt", message);
    srs.put("gtiff", message);
#endif
    
    return srs;
}


void SpatialReference::dump() const
{
    std::cout << *this;
}


std::ostream& operator<<(std::ostream& ostr, const SpatialReference& srs)
{

#ifdef PDAL_SRS_ENABLED
    
    std::string wkt = srs.toPTree().get<std::string>("prettycompoundwkt");
    ostr << wkt;
    
    return ostr;

#else
    boost::ignore_unused_variable_warning(ostr);
    boost::ignore_unused_variable_warning(srs);
    ostr << "SpatialReference data is not available without GDAL+libgeotiff support";
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
    throw pdal_error ("SpatialReference io operator>> is not available without GDAL+libgeotiff support");
#endif
}




} // namespace pdal
