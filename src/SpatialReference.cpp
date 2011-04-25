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

#include <libpc/libpc.hpp>

// GDAL OSR
#ifdef LIBPC_HAVE_GDAL
#include <ogr_srs_api.h>
#include <cpl_port.h>
#include <geo_normalize.h>
#include <geovalues.h>
#include <ogr_spatialref.h>
#include <gdal.h>
#include <xtiffio.h>
#include <cpl_multiproc.h>
#endif

// GeoTIFF
#ifdef LIBPC_HAVE_LIBGEOTIFF
#include <geotiff.h>
#include <geo_simpletags.h>
#include <geo_normalize.h>
#include <geo_simpletags.h>
#include <geovalues.h>
#endif

#include <libpc/SpatialReference.hpp>

// boost
#include <boost/concept_check.hpp>
#include <boost/cstdint.hpp>

// std
#include <stdexcept>
#include <string>
#include <vector>

#ifdef LIBPC_HAVE_GDAL
#  include "cpl_conv.h"
#endif


namespace libpc
{
   
SpatialReference::TiffStuff::TiffStuff()
    : m_gtiff(0)
    , m_tiff(0)
{
}
SpatialReference::TiffStuff::~TiffStuff()
{
#ifdef LIBPC_SRS_ENABLED
    if (m_gtiff != 0)
    {
        GTIFFree(m_gtiff);
        m_gtiff = 0;
    }
    if (m_tiff != 0)
    {
        ST_Destroy(m_tiff);
        m_tiff = 0;
    }
#endif
}

SpatialReference::SpatialReference()
    : m_tiffstuff(new TiffStuff())
    , m_wkt("")
{
    return;
}


SpatialReference::SpatialReference(SpatialReference const& rhs) 
    : m_tiffstuff(rhs.m_tiffstuff)
    , m_wkt(rhs.m_wkt)
{
    return;
}


SpatialReference& SpatialReference::operator=(SpatialReference const& rhs)
{
    if (&rhs != this)
    {
        this->m_tiffstuff = rhs.m_tiffstuff;
        m_wkt = rhs.m_wkt;
    }
    return *this;
}


SpatialReference::~SpatialReference() 
{
    return;
}


void SpatialReference::rebuildGTIFFromVLRs()
{
#ifndef LIBPC_SRS_ENABLED
    return 0;
#else

    // If we already have m_gtiff and m_tiff, that is because we have 
    // already called GetGTIF once before.  VLRs ultimately drive how the 
    // SpatialReference is defined, not the GeoTIFF keys.  
    if (m_tiffstuff->m_tiff != 0 )
    {
        ST_Destroy(m_tiffstuff->m_tiff);
        m_tiffstuff->m_tiff = 0;
    }

    if (m_tiffstuff->m_gtiff != 0 )
    {
        GTIFFree(m_tiffstuff->m_gtiff);
        m_tiffstuff->m_gtiff = 0;
    }
    
    m_tiffstuff->m_tiff = ST_Create();
   
    //////// here it used to read in the VLRs ////////

    m_tiffstuff->m_gtiff = GTIFNewSimpleTags(m_tiffstuff->m_tiff);
    if (!m_tiffstuff->m_gtiff) 
        throw std::runtime_error("The geotiff keys could not be read from VLR records");
    
    return;
#endif
}


int SpatialReference::geotiff_ST_SetKey(int tag, int count, GeotiffKeyType geotiff_key_type, void *data)
{
    return ST_SetKey(m_tiffstuff->m_tiff, tag, count, (int)geotiff_key_type, data);
}


void SpatialReference::geotiff_SetTags()
{
    m_tiffstuff->m_gtiff = GTIFNewSimpleTags(m_tiffstuff->m_tiff);
    if (!m_tiffstuff->m_gtiff) 
        throw std::runtime_error("The geotiff keys could not be read from VLR records");
    return;
}


void SpatialReference::geotiff_ResetTags()
{
    // If we already have m_gtiff and m_tiff, that is because we have 
    // already called GetGTIF once before.  VLRs ultimately drive how the 
    // SpatialReference is defined, not the GeoTIFF keys.  
    if (m_tiffstuff->m_tiff != 0 )
    {
        ST_Destroy(m_tiffstuff->m_tiff);
        m_tiffstuff->m_tiff = 0;
    }

    if (m_tiffstuff->m_gtiff != 0 )
    {
        GTIFFree(m_tiffstuff->m_gtiff);
        m_tiffstuff->m_gtiff = 0;
    }

    m_tiffstuff->m_tiff = ST_Create();

    return;
}


int SpatialReference::geotiff_ST_GetKey(int tag, int *count, int *st_type, void **data_ptr) const
{
    if (m_tiffstuff->m_tiff == 0) return 0;
    return ST_GetKey(m_tiffstuff->m_tiff, tag, count, st_type, data_ptr);
}




std::string SpatialReference::getWKT( WKTModeFlag mode_flag) const 
{
    return getWKT(mode_flag, false);
}


/// Fetch the SRS as WKT
std::string SpatialReference::getWKT(WKTModeFlag mode_flag , bool pretty) const 
{
#ifndef LIBPC_SRS_ENABLED
    boost::ignore_unused_variable_warning(mode_flag);
    boost::ignore_unused_variable_warning(pretty);

    // we don't have a way of making this pretty, or of stripping the compound wrapper.
    return m_wkt;
#else

    // If we already have Well Known Text then try return it, possibly
    // after some preprocessing.
    if( m_wkt != "" )
    {
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
    }

    // Otherwise build WKT from GeoTIFF VLRs. 

    GTIFDefn sGTIFDefn;
    char* pszWKT = 0;
    if (!m_tiffstuff->m_gtiff)
    {
        return std::string();
    }

    if (GTIFGetDefn(m_tiffstuff->m_gtiff, &sGTIFDefn))
    {
        pszWKT = GTIFGetOGISDefn( m_tiffstuff->m_gtiff, &sGTIFDefn );

        if (pretty) {
            OGRSpatialReference* poSRS = (OGRSpatialReference*) OSRNewSpatialReference(NULL);
            char *pszOrigWKT = pszWKT;
            poSRS->importFromWkt( &pszOrigWKT );
            
            CPLFree( pszWKT );
            pszWKT = NULL;
            poSRS->exportToPrettyWkt(&pszWKT, false);
            OSRDestroySpatialReference( poSRS );

        }

        // save this for future calls, etc.
        // m_wkt = std::string( pszWKT );

        if( pszWKT 
            && mode_flag == eHorizontalOnly 
            && strstr(pszWKT,"COMPD_CS") != NULL )
        {
            OGRSpatialReference* poSRS = (OGRSpatialReference*) OSRNewSpatialReference(NULL);
            char *pszOrigWKT = pszWKT;
            poSRS->importFromWkt( &pszOrigWKT );

            CPLFree( pszWKT );
            pszWKT = NULL;

            poSRS->StripVertical();
            if (pretty) 
                poSRS->exportToPrettyWkt(&pszWKT, false);
            else
                poSRS->exportToWkt( &pszWKT );
            
            OSRDestroySpatialReference( poSRS );
        }

        if (pszWKT)
        {
            std::string tmp(pszWKT);
            CPLFree(pszWKT);
            return tmp;
        }
    }
    return std::string();
#endif
}


void SpatialReference::setFromUserInput(std::string const& v)
{
#ifdef LIBPC_SRS_ENABLED

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

    if (!m_tiffstuff->m_gtiff)
    {
        rebuildGTIFFromVLRs(); 
    }

#ifdef LIBPC_SRS_ENABLED
    int ret = 0;
    ret = GTIFSetFromOGISDefn( m_tiffstuff->m_gtiff, v.c_str() );
    if (!ret) 
    {
        throw std::invalid_argument("could not set m_gtiff from WKT");
    }

    ret = GTIFWriteKeys(m_tiffstuff->m_gtiff);
    if (!ret) 
    {
        throw std::runtime_error("The geotiff keys could not be written");
    }
#else
    boost::ignore_unused_variable_warning(v);
#endif

    ////////////////////////////////ResetVLRs();
}


void SpatialReference::setVerticalCS(boost::int32_t verticalCSType, 
                                     std::string const& citation,
                                     boost::int32_t verticalDatum,
                                     boost::int32_t verticalUnits)
{
    if (!m_tiffstuff->m_gtiff)
    {
        rebuildGTIFFromVLRs(); 
    }

#ifdef LIBPC_SRS_ENABLED
    if( verticalCSType != KvUserDefined && verticalCSType > 0 )
        GTIFKeySet( m_tiffstuff->m_gtiff, VerticalCSTypeGeoKey, TYPE_SHORT, 1,
                    verticalCSType );

    if( citation != "" )
        GTIFKeySet( m_tiffstuff->m_gtiff, VerticalCitationGeoKey, TYPE_ASCII, 0, 
                    citation.c_str() );			       

    if( verticalDatum > 0 && verticalDatum != KvUserDefined )
        GTIFKeySet( m_tiffstuff->m_gtiff, VerticalDatumGeoKey, TYPE_SHORT, 1,
                    verticalDatum );
        
    if( verticalUnits > 0 && verticalUnits != KvUserDefined )
        GTIFKeySet( m_tiffstuff->m_gtiff, VerticalUnitsGeoKey, TYPE_SHORT, 1,
                    verticalUnits );

    int ret = GTIFWriteKeys(m_tiffstuff->m_gtiff);
    if (!ret) 
    {
        throw std::runtime_error("The geotiff keys could not be written");
    }

    // Clear WKT so it gets regenerated 
    m_wkt = std::string("");
    
    ///////////////////////////////ResetVLRs();
#else
    boost::ignore_unused_variable_warning(citation);
    boost::ignore_unused_variable_warning(verticalUnits);
    boost::ignore_unused_variable_warning(verticalDatum);
    boost::ignore_unused_variable_warning(verticalCSType);
#endif
}
    

std::string SpatialReference::getProj4() const 
{
#ifdef LIBPC_SRS_ENABLED
    
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
    
    return tmp;

#else

    // By default or if we have neither GDAL nor proj.4, we can't do squat
    return std::string();
#endif
}


void SpatialReference::setProj4(std::string const& v)
{
    if (!m_tiffstuff->m_gtiff)
    {
        rebuildGTIFFromVLRs();
    }
   
#ifdef LIBPC_SRS_ENABLED
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
        
    int ret = 0;
    ret = GTIFSetFromOGISDefn( m_tiffstuff->m_gtiff, tmp.c_str() );
    if (!ret)
    {
        throw std::invalid_argument("could not set m_gtiff from Proj4");
    }

    ret = GTIFWriteKeys(m_tiffstuff->m_gtiff);
    if (!ret) 
    {
        throw std::runtime_error("The geotiff keys could not be written");
    }

    GTIFDefn defn;

    if (m_tiffstuff->m_gtiff && GTIFGetDefn(m_tiffstuff->m_gtiff, &defn)) 
    {
        char* proj4def = GTIFGetProj4Defn(&defn);
        std::string tmp(proj4def);
        GTIFFreeMemory( proj4def );
    }
#else
    boost::ignore_unused_variable_warning(v);
#endif

    ////////////////////////////////ResetVLRs();
}


boost::property_tree::ptree SpatialReference::getPTree( ) const
{
    using boost::property_tree::ptree;
    ptree srs;

#ifdef LIBPC_SRS_ENABLED
    srs.put("proj4", getProj4());
    srs.put("prettywkt", getWKT(SpatialReference::eHorizontalOnly, true));
    srs.put("wkt", getWKT(SpatialReference::eHorizontalOnly, false));
    srs.put("compoundwkt", getWKT(eCompoundOK, false));
    srs.put("prettycompoundwkt", getWKT(eCompoundOK, true));
    srs.put("gtiff", getGTIFFText());

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


// Utility functor with accompanying to print GeoTIFF directory.
struct geotiff_dir_printer
{
    geotiff_dir_printer() {}

    std::string output() const { return m_oss.str(); }
    std::string::size_type size() const { return m_oss.str().size(); }

    void operator()(char* data, void* aux)
    {     
        ::boost::ignore_unused_variable_warning(aux);

        if (0 != data)
        {
            m_oss << data;
        }
    }

private:
    std::ostringstream m_oss;
};


static int libpcGeoTIFFPrint(char* data, void* aux)
{
    geotiff_dir_printer* printer = reinterpret_cast<geotiff_dir_printer*>(aux);
    (*printer)(data, 0);
    return static_cast<int>(printer->size());
}



std::string SpatialReference::getGTIFFText() const
{
#ifndef LIBPC_SRS_ENABLED
    return std::string("");
#else

    if( m_tiffstuff->m_gtiff == NULL )
        return std::string("");

    geotiff_dir_printer geotiff_printer;
    GTIFPrint(m_tiffstuff->m_gtiff, libpcGeoTIFFPrint, &geotiff_printer);
    return geotiff_printer.output();
#endif
}


std::ostream& operator<<(std::ostream& ostr, const SpatialReference& srs)
{
    ostr << "SRS: ";
    ostr << srs.getWKT();
    ostr << std::endl;
    return ostr;
}


} // namespace libpc



#if 0



/// Keep a copy of the VLRs that are related to GeoTIFF SRS information.
void SpatialReference::SetVLRs(std::vector<VariableRecord> const& vlrs)
{
    
    std::string const uid("LASF_Projection");
    
    // Wipe out any existing VLRs that might exist on the SpatialReference
    m_vlrs.clear();
    
    // We only copy VLR records from the list which are related to GeoTIFF keys.
    // They must have an id of "LASF_Projection" and a record id that's related.
    std::vector<VariableRecord>::const_iterator it;
    for (it = vlrs.begin(); it != vlrs.end(); ++it)
    {
        VariableRecord const& vlr = *it;
        if (IsGeoVLR(vlr))
        {
            m_vlrs.push_back(vlr);
        }
    }
}


#endif