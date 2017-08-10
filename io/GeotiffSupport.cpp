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

#include <geo_normalize.h>
#include <ogr_spatialref.h>
#include <geo_simpletags.h>

PDAL_C_START

// These functions are available from GDAL, but they
// aren't exported.
char CPL_DLL * GTIFGetOGISDefn(GTIF*, GTIFDefn*);
int CPL_DLL GTIFSetFromOGISDefn(GTIF*, const char*);

PDAL_C_END

#include <io/LasVLR.hpp>

namespace pdal
{

namespace
{

struct GeotiffCtx
{
public:
    GeotiffCtx() : gtiff(nullptr)
    {
        tiff = ST_Create();
    }

    ~GeotiffCtx()
    {
        if (gtiff)
            GTIFFree(gtiff);
        ST_Destroy(tiff);
    }

    ST_TIFF *tiff;
    GTIF *gtiff;
};

}

GeotiffSrs::GeotiffSrs(const std::vector<uint8_t>& directoryRec,
    const std::vector<uint8_t>& doublesRec,
    const std::vector<uint8_t>& asciiRec)
{
    GeotiffCtx ctx;

    if (directoryRec.empty())
        return;

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

    const ShortKeyHeader *header = (const ShortKeyHeader *)directoryRec.data();
    size_t declaredSize = (header->numKeys + 1) * 4;
    if (directoryRec.size() < declaredSize)
        return;

    uint8_t *dirData = const_cast<uint8_t *>(directoryRec.data());
    ST_SetKey(ctx.tiff, GEOTIFF_DIRECTORY_RECORD_ID,
        (1 + header->numKeys) * 4, STT_SHORT, (void *)dirData);

    if (doublesRec.size())
    {
        uint8_t *doubleData = const_cast<uint8_t *>(doublesRec.data());
        ST_SetKey(ctx.tiff, GEOTIFF_DOUBLES_RECORD_ID,
            doublesRec.size() / sizeof(double), STT_DOUBLE,
            (void *)doubleData);
    }

    if (asciiRec.size())
    {
        uint8_t *asciiData = const_cast<uint8_t *>(asciiRec.data());
        ST_SetKey(ctx.tiff, GEOTIFF_ASCII_RECORD_ID,
            asciiRec.size(), STT_ASCII, (void *)asciiData);
    }

    ctx.gtiff = GTIFNewSimpleTags(ctx.tiff);

    GTIFDefn sGTIFDefn;
    if (GTIFGetDefn(ctx.gtiff, &sGTIFDefn))
    {
        char *wkt = GTIFGetOGISDefn(ctx.gtiff, &sGTIFDefn);
        if (wkt)
            m_srs.set(wkt);
    }
}


GeotiffTags::GeotiffTags(const SpatialReference& srs)
{
    if (srs.empty())
        return;

    GeotiffCtx ctx;
    ctx.gtiff = GTIFNewSimpleTags(ctx.tiff);

    // Set tiff tags from WKT
    if (!GTIFSetFromOGISDefn(ctx.gtiff, srs.getWKT().c_str()))
        throw error("Could not set m_gtiff from WKT");
    GTIFWriteKeys(ctx.gtiff);

    auto sizeFromType = [](int type, int count) -> size_t
    {
        if (type == STT_ASCII)
            return count;
        else if (type == STT_SHORT)
            return 2 * count;
        else if (type == STT_DOUBLE)
            return 8 * count;
        return 8 * count;
    };

    int count;
    int st_type;
    uint8_t *data;
    if (ST_GetKey(ctx.tiff, GEOTIFF_DIRECTORY_RECORD_ID,
        &count, &st_type, (void **)&data))
    {
        size_t size = sizeFromType(st_type, count);
        m_directoryRec.resize(size);
        std::copy(data, data + size, m_directoryRec.begin());
    }
    if (ST_GetKey(ctx.tiff, GEOTIFF_DOUBLES_RECORD_ID,
        &count, &st_type, (void **)&data))
    {
        size_t size = sizeFromType(st_type, count);
        m_doublesRec.resize(size);
        std::copy(data, data + size, m_doublesRec.begin());
    }
    if (ST_GetKey(ctx.tiff, GEOTIFF_ASCII_RECORD_ID,
        &count, &st_type, (void **)&data))
    {
        size_t size = sizeFromType(st_type, count);
        m_asciiRec.resize(size);
        std::copy(data, data + size, m_asciiRec.begin());
    }
}

} // namespace pdal
