/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include "GeotiffSupport.hpp"

#include <sstream>

// GDAL
#include <geo_normalize.h>
#include <ogr_spatialref.h>

// See http://lists.osgeo.org/pipermail/gdal-dev/2013-November/037429.html
#define CPL_SERV_H_INCLUDED

#include <geo_simpletags.h>
#include <cpl_conv.h>

PDAL_C_START

// These functions are available from GDAL, but they
// aren't exported.
char CPL_DLL * GTIFGetOGISDefn(GTIF*, GTIFDefn*);
int CPL_DLL GTIFSetFromOGISDefn(GTIF*, const char*);

PDAL_C_END

#include <pdal/GDALUtils.hpp>

struct StTiff : public ST_TIFF
{};

namespace pdal
{

GeotiffSupport::~GeotiffSupport()
{
#ifdef PDAL_HAVE_LIBGEOTIFF
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


void GeotiffSupport::resetTags()
{
    // If we already have m_gtiff and m_tiff, that is because we have
    // already called GetGTIF once before.  VLRs ultimately drive how the
    // SpatialReference is defined, not the GeoTIFF keys.
    if (m_tiff != 0)
    {
        ST_Destroy(m_tiff);
        m_tiff = 0;
    }

    if (m_gtiff != 0)
    {
        GTIFFree(m_gtiff);
        m_gtiff = 0;
    }

    m_tiff = (StTiff *)ST_Create();

    return;
}


bool GeotiffSupport::setShortKeys(int tag, void *data, int size)
{
    // Make sure struct is 16 bytes.
#pragma pack(push)
#pragma pack(1)
    struct ShortKeyHeader
    {
        uint16_t dirVersion;
        uint16_t keyRev;
        uint16_t minorRev;
        uint16_t numKeys;
    };
#pragma pack(pop)

    ShortKeyHeader *header = (ShortKeyHeader *)data;
    int declaredSize = (header->numKeys + 1) * 4;
    if (size < declaredSize)
        return false;
    ST_SetKey(m_tiff, tag, (1 + header->numKeys) * 4, STT_SHORT, data);
    return true;
}


bool GeotiffSupport::setDoubleKeys(int tag, void *data, int size)
{
    ST_SetKey(m_tiff, tag, size / sizeof(double), STT_DOUBLE, data);
    return true;
}


bool GeotiffSupport::setAsciiKeys(int tag, void *data, int size)
{
    ST_SetKey(m_tiff, tag, size, STT_ASCII, data);
    return true;
}


/// Get the geotiff data associated with a tag.
/// \param tag - geotiff tag.
/// \param count - Number of items fetched.
/// \param data_ptr - Pointer to fill with address of filled data.
/// \return  Size of data referred to by \c data_ptr
size_t GeotiffSupport::getKey(int tag, int *count, void **data_ptr) const
{
    int st_type;

    if (m_tiff == 0)
        return 0;

    if (!ST_GetKey(m_tiff, tag, count, &st_type, data_ptr))
        return 0;

    if (st_type == STT_ASCII)
        return *count;
    else if (st_type == STT_SHORT)
        return 2 * *count;
    else if (st_type == STT_DOUBLE)
        return 8 * *count;
    return 8 * *count;
}


void GeotiffSupport::setTags()
{
    m_gtiff = GTIFNewSimpleTags(m_tiff);
    if (!m_gtiff)
        throw std::runtime_error("The geotiff keys could not be read "
            "from VLR records");
}


std::string GeotiffSupport::getWkt(bool horizOnly, bool pretty) const
{
    GTIFDefn sGTIFDefn;
    char* pszWKT = 0;

    gdal::ErrorHandler::ExceptionSuspender suspender;

    if (!m_gtiff)
        return std::string();

    if (!GTIFGetDefn(m_gtiff, &sGTIFDefn))
        return std::string();

    pszWKT = GTIFGetOGISDefn(m_gtiff, &sGTIFDefn);

    if (pretty)
    {
        OGRSpatialReference* poSRS =
            (OGRSpatialReference*) OSRNewSpatialReference(NULL);
        char *pszOrigWKT = pszWKT;
        poSRS->importFromWkt(&pszOrigWKT);

        CPLFree(pszWKT);
        pszWKT = NULL;
        poSRS->exportToPrettyWkt(&pszWKT, false);
        OSRDestroySpatialReference(poSRS);

    }

    if (pszWKT
            && horizOnly
            && strstr(pszWKT,"COMPD_CS") != NULL)
    {
        OGRSpatialReference* poSRS =
            (OGRSpatialReference*) OSRNewSpatialReference(NULL);
        char *pszOrigWKT = pszWKT;
        poSRS->importFromWkt(&pszOrigWKT);

        CPLFree(pszWKT);
        pszWKT = NULL;

        poSRS->StripVertical();
        if (pretty)
            poSRS->exportToPrettyWkt(&pszWKT, false);
        else
            poSRS->exportToWkt(&pszWKT);

        OSRDestroySpatialReference(poSRS);
    }

    if (pszWKT)
    {
        std::string tmp(pszWKT);
        CPLFree(pszWKT);
        return tmp;
    }

    return std::string();
}


void GeotiffSupport::rebuildGTIF()
{
    // If we already have m_gtiff and m_tiff, that is because we have
    // already called GetGTIF once before.  VLRs ultimately drive how the
    // SpatialReference is defined, not the GeoTIFF keys.
    if (m_tiff != 0)
    {
        ST_Destroy(m_tiff);
        m_tiff = 0;
    }

    if (m_gtiff != 0)
    {
        GTIFFree(m_gtiff);
        m_gtiff = 0;
    }

    m_tiff = (StTiff *)ST_Create();

    // (here it used to read in the VLRs)

    m_gtiff = GTIFNewSimpleTags(m_tiff);
    if (!m_gtiff)
        throw std::runtime_error("The geotiff keys could not be read from "
            "VLR records");
}


void GeotiffSupport::setWkt(const std::string& v)
{
    if (!m_gtiff)
        rebuildGTIF();

    if (v == "")
        return;

    if (!GTIFSetFromOGISDefn(m_gtiff, v.c_str()))
        throw std::invalid_argument("could not set m_gtiff from WKT");

    int ret = 0;
    ret = GTIFSetFromOGISDefn(m_gtiff, v.c_str());
    if (!ret)
    {
        throw std::invalid_argument("could not set m_gtiff from WKT");
    }

    ret = GTIFWriteKeys(m_gtiff);
    if (!ret)
    {
        throw std::runtime_error("The geotiff keys could not be written");
    }

    return;
}


// Utility functor with accompanying to print GeoTIFF directory.
struct geotiff_dir_printer
{
    geotiff_dir_printer() {}

    std::string output() const
    {
        return m_oss.str();
    }
    std::string::size_type size() const
    {
        return m_oss.str().size();
    }

    void operator()(char* data, void* /*aux*/)
    {
        if (0 != data)
        {
            m_oss << data;
        }
    }

private:
    std::ostringstream m_oss;
};


static int pdalGeoTIFFPrint(char* data, void* aux)
{
    geotiff_dir_printer* printer = reinterpret_cast<geotiff_dir_printer*>(aux);
    (*printer)(data, 0);
    return static_cast<int>(printer->size());
}


std::string GeotiffSupport::getText() const
{
    if (m_gtiff == NULL)
        return std::string("");

    geotiff_dir_printer geotiff_printer;
    GTIFPrint(m_gtiff, pdalGeoTIFFPrint, &geotiff_printer);
    const std::string s = geotiff_printer.output();
    return s;
}

} // namespace pdal
