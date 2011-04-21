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

#ifndef INCLUDED_SPATIALREFERENCE_HPP
#define INCLUDED_SPATIALREFERENCE_HPP

#include <libpc/libpc.hpp>

#include <string>
#include <ostream>

namespace libpc
{

class LIBPC_DLL SpatialReference
{
public:
    SpatialReference();
    SpatialReference(std::string wkt);
    SpatialReference(const SpatialReference&);

    SpatialReference& operator=(SpatialReference const& rhs);

    bool operator==(SpatialReference const& rhs) const;

    const std::string& getWKT() const;

private:
    // for now, we'll just fake it
    std::string m_wkt;
};


std::ostream& operator<<(std::ostream& ostr, const SpatialReference& srs);


} // namespace libpc

#endif


#if 0

#include <liblas/detail/fwd.hpp>
#include <liblas/detail/private_utility.hpp>
#include <liblas/variablerecord.hpp>
#include <liblas/exception.hpp>
#include <liblas/capi/las_config.h>
#include <liblas/export.hpp>
#include <liblas/external/property_tree/ptree.hpp>

// std
#include <stdexcept> // std::out_of_range
#include <cstdlib> // std::size_t
#include <string>

// Fake out the compiler if we don't have libgeotiff includes already
#if !defined(__geotiff_h_)
typedef struct GTIFS *GTIF;
#endif
#if !defined(__geo_simpletags_h_)
typedef struct ST_TIFFS *ST_TIFF;
#endif

namespace liblas {

/// Spatial Reference System container for libLAS
class LAS_DLL SpatialReference
{
public:
    enum WKTModeFlag
    {
        eHorizontalOnly = 1,
        eCompoundOK = 2
    };

    enum GeoVLRType
    {
        eGeoTIFF = 1,
        eOGRWKT = 2
    };

    /// Default constructor.
    SpatialReference();

    /// Destructor.
    /// If libgeotiff is enabled, deallocates libtiff and libgeotiff objects used internally.
    ~SpatialReference();

    /// Constructor creating SpatialReference instance from given Variable-Length Record.
    SpatialReference(std::vector<VariableRecord> const& vlrs);

    /// Copy constryctor.
    SpatialReference(SpatialReference const& other);

    /// Assignment operator.
    SpatialReference& operator=(SpatialReference const& rhs);
    
    /// Returns a pointer to the internal GTIF*.  Only available if 
    /// you have libgeotiff linked in.
    const GTIF* GetGTIF();

    void SetGTIF(GTIF* pgtiff, ST_TIFF* ptiff);

    /// Returns the OGC WKT describing Spatial Reference System.
    /// If GDAL is linked, it uses GDAL's operations and methods to determine 
    /// the WKT.  If GDAL is not linked, no WKT is returned.
    /// \param mode_flag May be eHorizontalOnly indicating the WKT will not 
    /// include vertical coordinate system info (the default), or 
    /// eCompoundOK indicating the the returned WKT may be a compound 
    /// coordinate system if there is vertical coordinate system info 
    /// available.
    std::string GetWKT(WKTModeFlag mode_flag = eHorizontalOnly) const;
    std::string GetWKT(WKTModeFlag mode_flag, bool pretty) const;
    
    /// Sets the SRS using GDAL's OGC WKT. If GDAL is not linked, this 
    /// operation has no effect.
    /// \param v - a string containing the WKT string.  
    void SetWKT(std::string const& v);

    /// Sets the vertical coordinate system using geotiff key values.
    /// This operation should normally be done after setting the horizontal
    /// portion of the coordinate system with something like SetWKT(), 
    /// SetProj4(), SetGTIF() or SetFromUserInput()
    /// \param verticalCSType - An EPSG vertical coordinate system code, 
    /// normally in the range 5600 to 5799, or -1 if one is not available.
    /// \param citation - a textual description of the vertical coordinate 
    /// system or an empty string if nothing is available.
    /// \param verticalDatum - the EPSG vertical datum code, often in the 
    /// range 5100 to 5299 - implied by verticalCSType if that is provided, or 
    /// -1 if no value is available.
    /// \param verticalUnits - the EPSG vertical units code, often 9001 for Metre.
    void SetVerticalCS(boost::int32_t verticalCSType, 
                       std::string const& citation = std::string(0),
                       boost::int32_t verticalDatum = -1,
                       boost::int32_t verticalUnits = 9001);

    /// Sets the SRS using GDAL's SetFromUserInput function. If GDAL is not linked, this 
    /// operation has no effect.
    /// \param v - a string containing the definition (filename, proj4, wkt, etc).  
    void SetFromUserInput(std::string const& v);
        
    /// Returns the Proj.4 string describing the Spatial Reference System.
    /// If GDAL is linked, it uses GDAL's operations and methods to determine 
    /// the Proj.4 string -- otherwise, if libgeotiff is linked, it uses 
    /// that.  Note that GDAL's operations are much more mature and 
    /// support more coordinate systems and descriptions.
    std::string GetProj4() const;

    /// Sets the Proj.4 string describing the Spatial Reference System.
    /// If GDAL is linked, it uses GDAL's operations and methods to determine 
    /// the Proj.4 string -- otherwise, if libgeotiff is linked, it uses 
    /// that.  Note that GDAL's operations are much more mature and 
    /// support more coordinate systems and descriptions.
    /// \param v - a string containing the Proj.4 string.
    void SetProj4(std::string const& v);
    
    /// Set the LASVLRs for the SpatialReference.  SetVLRs will only copy 
    /// VLR records that pertain to the GeoTIFF keys, and extraneous 
    /// VLR records will not be copied.
    /// \param vlrs - A list of VLRs that contains VLRs describing GeoTIFF keys
    void SetVLRs(std::vector<VariableRecord> const& vlrs);
    
    /// Add a VLR representing GeoTIFF keys to the SRS
    void AddVLR(VariableRecord const& vlr);
    
    /// Return a copy of the LASVLRs that SpatialReference maintains
    std::vector<VariableRecord> GetVLRs() const;

    void ClearVLRs( GeoVLRType eType );

    liblas::property_tree::ptree GetPTree() const;    
private:

    // FIXME: Define as shared_ptr<GTIF> with custom deleter to get rid of bloated mem management, unsafe anyway --mloskot
    GTIF*       m_gtiff;
    ST_TIFF*    m_tiff;

    std::string m_wkt;

    std::vector<VariableRecord> m_vlrs;
    bool IsGeoVLR(VariableRecord const& vlr) const;
    std::string GetGTIFFText() const;

    /// Reset the VLRs of the SpatialReference using the existing GTIF* and ST_TIF*
    /// Until this method is called, 
    /// the SpatialReference will only contain a SRS description using the VLRs 
    /// that it was first instantiated with.  SetWKT and SetProj4 can 
    /// be used to change the GTIF* 
    void ResetVLRs();
};

} // namespace liblas

LAS_C_START
#if defined(__geotiff_h_)
#if defined(GEO_NORMALIZE_H_INCLUDED)
char LAS_DLL * GTIFGetOGISDefn(GTIF*, GTIFDefn*);
#endif

int  LAS_DLL   GTIFSetFromOGISDefn(GTIF*, const char*);
void SetLinearUnitCitation(GTIF* psGTIF, char* pszLinearUOMName);

#if defined(_OGR_SRS_API_H_INCLUDED)
void SetGeogCSCitation(GTIF* psGTIF, OGRSpatialReference* poSRS, char* angUnitName, int nDatum, short nSpheroid);
#endif // defined _OGR_SRS_API_H_INCLUDED
#endif // defined __geotiff_h_

LAS_C_END

#endif 
