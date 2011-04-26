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

// We have two modes: with GDAL and geotiff, SRS is enabled -- otherwise, it's disabled
// we use one macro to simplify the control
#if defined(LIBPC_HAVE_GDAL) && defined(LIBPC_HAVE_LIBGEOTIFF)
#define LIBPC_SRS_ENABLED
#endif

#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>

#include <string>
#include <ostream>

// Fake out the compiler if we don't have libgeotiff includes already
#if !defined(__geotiff_h_)
typedef struct GTIFS *GTIF;
#endif
#if !defined(__geo_simpletags_h_)
typedef struct ST_TIFFS *ST_TIFF;
#endif


namespace libpc
{

class LIBPC_DLL SpatialReference
{
public:
    enum WKTModeFlag
    {
        eHorizontalOnly = 1,
        eCompoundOK = 2
    };


    /// Default constructor.
    SpatialReference();

    /// Destructor.
    /// If libgeotiff is enabled, deallocates libtiff and libgeotiff objects used internally.
    ~SpatialReference();

    /// Copy constryctor.
    SpatialReference(SpatialReference const& other);

    /// Assignment operator.
    SpatialReference& operator=(SpatialReference const& rhs);
    
    /// Returns a pointer to the internal GTIF*.  Only available if 
    /// you have libgeotiff linked in.
    void rebuildGTIF();

    /// Returns the OGC WKT describing Spatial Reference System.
    /// If GDAL is linked, it uses GDAL's operations and methods to determine 
    /// the WKT.  If GDAL is not linked, no WKT is returned.
    /// \param mode_flag May be eHorizontalOnly indicating the WKT will not 
    /// include vertical coordinate system info (the default), or 
    /// eCompoundOK indicating the the returned WKT may be a compound 
    /// coordinate system if there is vertical coordinate system info 
    /// available.
    std::string getWKT(WKTModeFlag mode_flag = eHorizontalOnly) const;
    std::string getWKT(WKTModeFlag mode_flag, bool pretty) const;
    
    /// Sets the SRS using GDAL's OGC WKT. If GDAL is not linked, this 
    /// operation has no effect.
    /// \param v - a string containing the WKT string.  
    void setWKT(std::string const& v);

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
    void setVerticalCS(boost::int32_t verticalCSType, 
                       std::string const& citation = std::string(0),
                       boost::int32_t verticalDatum = -1,
                       boost::int32_t verticalUnits = 9001);

    /// Sets the SRS using GDAL's SetFromUserInput function. If GDAL is not linked, this 
    /// operation has no effect.
    /// \param v - a string containing the definition (filename, proj4, wkt, etc).  
    void setFromUserInput(std::string const& v);
        
    /// Returns the Proj.4 string describing the Spatial Reference System.
    /// If GDAL is linked, it uses GDAL's operations and methods to determine 
    /// the Proj.4 string -- otherwise, if libgeotiff is linked, it uses 
    /// that.  Note that GDAL's operations are much more mature and 
    /// support more coordinate systems and descriptions.
    std::string getProj4() const;

    /// Sets the Proj.4 string describing the Spatial Reference System.
    /// If GDAL is linked, it uses GDAL's operations and methods to determine 
    /// the Proj.4 string -- otherwise, if libgeotiff is linked, it uses 
    /// that.  Note that GDAL's operations are much more mature and 
    /// support more coordinate systems and descriptions.
    /// \param v - a string containing the Proj.4 string.
    void setProj4(std::string const& v);
    
    boost::property_tree::ptree getPTree() const;    

    enum GeotiffKeyType
    {
        Geotiff_KeyType_SHORT=1,
        Geotiff_KeyType_DOUBLE=2,
        Geotiff_KeyType_ASCII=3
    };
    int geotiff_ST_SetKey(int tag, int count, GeotiffKeyType geotiff_key_type, void *data);
    void geotiff_SetTags();
    void geotiff_ResetTags();
    int geotiff_ST_GetKey(int tag, int *count, int *st_key_type, void **data_ptr) const;

private:
    std::string getGTIFFText() const;

    class TiffStuff
    {
    public:
        TiffStuff();
        ~TiffStuff();
        GTIF*       m_gtiff;
        ST_TIFF*    m_tiff;
    };
    boost::shared_ptr<TiffStuff> m_tiffstuff;

    std::string m_wkt;
};


extern LIBPC_DLL std::ostream& operator<<(std::ostream& ostr, const SpatialReference& srs);

} // namespace libpc


LIBPC_C_START
#ifdef __geotiff_h_

#ifdef GEO_NORMALIZE_H_INCLUDED
char LIBPC_DLL * GTIFGetOGISDefn(GTIF*, GTIFDefn*);
#endif

int LIBPC_DLL GTIFSetFromOGISDefn(GTIF*, const char*);
void SetLinearUnitCitation(GTIF* psGTIF, char* pszLinearUOMName);

#ifdef _OGR_SRS_API_H_INCLUDED
void SetGeogCSCitation(GTIF* psGTIF, OGRSpatialReference* poSRS, char* angUnitName, int nDatum, short nSpheroid);
#endif // defined _OGR_SRS_API_H_INCLUDED

#endif // defined __geotiff_h_
LIBPC_C_END

#endif
