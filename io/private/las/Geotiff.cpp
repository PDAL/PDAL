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

#include "Geotiff.hpp"

#include <sstream>

#include <geo_normalize.h>
#include <geo_simpletags.h>
#include <geo_tiffp.h>

namespace pdal
{
namespace las
{

// Utility functor with accompanying to print GeoTIFF directory.
struct geotiff_dir_printer
{
    geotiff_dir_printer() {}

    std::string output() const { return m_oss.str(); }
    std::string::size_type size() const { return m_oss.str().size(); }

    void operator()(char* data, void* /*aux*/)
    {

        if (0 != data)
        {
            m_oss << data;
        }
    }

private:
    std::stringstream m_oss;
};

} // namespace las
} // namespace pdal

extern "C"
{

// These functions are available from GDAL, but they aren't provided in a header file.
char PDAL_DLL * GTIFGetOGISDefn(GTIF*, GTIFDefn*);
int PDAL_DLL GTIFSetFromOGISDefn(GTIF*, const char*);
void VSIFree(void *data);

int PDALGeoTIFFPrint(char* data, void* aux)
{
    pdal::las::geotiff_dir_printer* printer =
        reinterpret_cast<pdal::las::geotiff_dir_printer*>(aux);
    (*printer)(data, 0);
    return static_cast<int>(printer->size());
}

} // extern "C"

#include "Vlr.hpp"

namespace pdal
{
namespace las
{

namespace
{

struct GeotiffCtx
{
public:
    GeotiffCtx() : gtiff(nullptr)
    {
        tiff = ST_Create();
        GTIFSetSimpleTagsMethods(&methods);
    }

    ~GeotiffCtx()
    {
        if (gtiff)
            GTIFFree(gtiff);
        ST_Destroy(tiff);
    }

    ST_TIFF *tiff;
    GTIF *gtiff;
    TIFFMethod methods;
};

static void PDALGTIFErrorFunction(GTIF* gt, int level, const char* msg, ...)
{
    va_list list;
    (void)gt;

    va_start(list, msg);
    if( level == LIBGEOTIFF_WARNING )
        fprintf(stderr, "GEOTIFF Warning: ");
    else if( level == LIBGEOTIFF_ERROR )
        fprintf(stderr, "GEOTIFF Error: ");
    vfprintf(stderr, msg, list);
    fprintf(stderr, "\n");
    va_end(list);
}

} // unnamed namespace

#pragma pack(push)
#pragma pack(1)
struct Entry
{
    uint16_t key;
    uint16_t location;
    uint16_t count;
    uint16_t offset;
};
#pragma pack(pop)

GeotiffSrs::GeotiffSrs(const std::vector<char>& directoryRec,
    const std::vector<char>& doublesRec,
    const std::vector<char>& asciiRec, LogPtr log) : m_log(log)
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

    validateDirectory((const Entry *)(header + 1), header->numKeys,
        doublesRec.size() / sizeof(double), asciiRec.size());

    char *dirData = const_cast<char *>(directoryRec.data());
    ST_SetKey(ctx.tiff, GeotiffDirectoryRecordId,
        (1 + header->numKeys) * 4, STT_SHORT, (void *)dirData);

    if (doublesRec.size())
    {
        char *doubleData = const_cast<char *>(doublesRec.data());
        ST_SetKey(ctx.tiff, GeotiffDoublesRecordId,
            doublesRec.size() / sizeof(double), STT_DOUBLE,
            (void *)doubleData);
    }

    if (asciiRec.size())
    {
        char *asciiData = const_cast<char *>(asciiRec.data());
        ST_SetKey(ctx.tiff, GeotiffAsciiRecordId,
            asciiRec.size(), STT_ASCII, (void *)asciiData);
    }

    ctx.gtiff = GTIFNewWithMethodsEx(ctx.tiff,
                                     &ctx.methods,
                                     PDALGTIFErrorFunction,
                                     (void*) this);
    if (!ctx.gtiff)
        throw Geotiff::error("Couldn't create Geotiff tags from "
            "Geotiff definition.");

    GTIFDefn sGTIFDefn;
    if (GTIFGetDefn(ctx.gtiff, &sGTIFDefn))
    {
        char *wkt = GTIFGetOGISDefn(ctx.gtiff, &sGTIFDefn);
        if (wkt)
        {
            m_srs.set(wkt);
            VSIFree(wkt);
        }
    }

    geotiff_dir_printer geotiff_printer;
    GTIFPrint(ctx.gtiff, PDALGeoTIFFPrint, &geotiff_printer);

    m_gtiff_print_string = geotiff_printer.output();
}


void GeotiffSrs::validateDirectory(const Entry *ent, size_t numEntries,
    size_t numDoubles, size_t asciiSize)
{
    for (size_t i = 0; i < numEntries; ++i)
    {
        if (ent->count == 0)
            m_log->get(LogLevel::Warning) << "Geotiff directory contains " <<
                "key " << ent->key << " with 0 count." << std::endl;
        if (ent->location == 0 && ent->count != 1)
            m_log->get(LogLevel::Error) << "Geotiff directory contains key " <<
                ent->key << " with short entry and more than one value." <<
                std::endl;
        if (ent->location == GeotiffDirectoryRecordId)
            if (ent->offset + ent->count > numDoubles)
                m_log->get(LogLevel::Error) << "Geotiff directory contains " <<
                    "key " << ent->key << " with count/offset outside of valid "
                    "range of doubles record." << std::endl;
        if (ent->location == GeotiffAsciiRecordId)
            if (ent->offset + ent->count > asciiSize)
                m_log->get(LogLevel::Error) << "Geotiff directory contains " <<
                    " key " << ent->key << " with count/offset outside of "
                    "valid range of ascii record." << std::endl;
        ent++;
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
        throw Geotiff::error("Could not set m_gtiff from WKT");
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
    char *data;
    if (ST_GetKey(ctx.tiff, GeotiffDirectoryRecordId,
        &count, &st_type, (void **)&data))
    {
        size_t size = sizeFromType(st_type, count);
        m_directoryRec.resize(size);
        std::copy(data, data + size, m_directoryRec.begin());
    }
    if (ST_GetKey(ctx.tiff, GeotiffDoublesRecordId,
        &count, &st_type, (void **)&data))
    {
        size_t size = sizeFromType(st_type, count);
        m_doublesRec.resize(size);
        std::copy(data, data + size, m_doublesRec.begin());
    }
    if (ST_GetKey(ctx.tiff, GeotiffAsciiRecordId,
        &count, &st_type, (void **)&data))
    {
        size_t size = sizeFromType(st_type, count);
        m_asciiRec.resize(size);
        std::copy(data, data + size, m_asciiRec.begin());
    }
}

} // namespace las
} // namespace pdal
