/******************************************************************************
* Copyright (c) 2018, Kyle Mann (kyle@hobu.co)
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

#include "EsriUtil.hpp"

//ABELL
#include <iostream>

#include <nlohmann/json.hpp>

#include "../lepcc/src/include/lepcc_c_api.h"
#include "../lepcc/src/include/lepcc_types.h"

namespace pdal
{
namespace i3s
{

/*Return value of data in json format*/
NL::json parse(const std::string& data)
{
    return parse(data, "Error during parsing: ");
}


NL::json parse(const std::string& data, const std::string& error)
{
    NL::json j;

    if (data.size())
    {
        try
        {
            j = NL::json::parse(data);
        }
        catch (const NL::json::exception& err)
        {
            throw EsriError(error + err.what());
        }
    }
    return j;
}


std::vector<lepcc::Point3D> decompressXYZ(std::vector<char>* compData)
{
    int nInfo = lepcc_getBlobInfoSize();
    lepcc_ContextHdl ctx(nullptr);
    ctx = lepcc_createContext();
    lepcc_blobType bt;
    lepcc::uint32 blobSize = 0;

    const unsigned char* compressed = reinterpret_cast<const unsigned char*>
        (compData->data());
    lepcc_status stat;
    std::vector<lepcc::Point3D> decVec;
    lepcc::uint32 xyzPts = 0;

    lepcc::ErrCode errCode = (lepcc::ErrCode)lepcc_getBlobInfo(ctx,
        compressed, nInfo, &bt, &blobSize);

    int nBytes = (errCode == lepcc::ErrCode::Ok) ? (int)blobSize : -1;
    if (nBytes > 0)
    {
        const lepcc::Byte* pByte = compressed;
        stat = lepcc_getPointCount(ctx, pByte, nBytes, &xyzPts);
        if (stat != (lepcc_status) lepcc::ErrCode::Ok)
            throw EsriError("LEPCC point count fetch failed");

        decVec.resize(xyzPts);
        stat = lepcc_decodeXYZ(ctx, &pByte, nBytes, &xyzPts,
            (double*)(&decVec[0]));
        if (stat != (lepcc_status) lepcc::ErrCode::Ok)
            throw EsriError("LEPCC decompression failed");
    }
    return decVec;
}


std::vector<lepcc::RGB_t> decompressRGB(std::vector<char>* compData)
{
    const unsigned char* compressed = reinterpret_cast<const unsigned char*>
        (compData->data());
    int nInfo = lepcc_getBlobInfoSize();
    lepcc_ContextHdl ctx(nullptr);
    ctx = lepcc_createContext();
    lepcc_blobType bt;
    lepcc::uint32 blobSize = 0;

    lepcc_status stat;
    std::vector<lepcc::RGB_t> rgbVec;

    lepcc::uint32 nPts = 0;
    lepcc::ErrCode errCode =
        (lepcc::ErrCode)lepcc_getBlobInfo(
                ctx, compressed, nInfo, &bt, &blobSize);

    int nBytes = (errCode == lepcc::ErrCode::Ok) ? (int)blobSize : -1;

    if (nBytes > 0)
    {
        const lepcc::Byte* pByte = compressed;
        stat = lepcc_getRGBCount(ctx, pByte, nBytes, &nPts);
        if (stat != (lepcc_status) lepcc::ErrCode::Ok)
            throw EsriError("RGB point count fetch failed");

        rgbVec.resize(nPts);
        stat = lepcc_decodeRGB(
                ctx, &pByte, nBytes, &nPts, (lepcc::Byte*)(&rgbVec[0]));
        if (stat != (lepcc_status) lepcc::ErrCode::Ok)
            throw EsriError("RGB decompression failed");
    }
    return rgbVec;
}


std::vector<uint16_t> decompressIntensity(std::vector<char>* compData)
{
    const unsigned char* compressed = reinterpret_cast<const unsigned char*>
        (compData->data());
    int nInfo = lepcc_getBlobInfoSize();
    lepcc_ContextHdl ctx(nullptr);
    ctx = lepcc_createContext();
    lepcc_blobType bt;
    lepcc::uint32 blobSize = 0;

    lepcc_status stat;
    lepcc::uint32 nPts = 0;
    lepcc::ErrCode errCode =
        (lepcc::ErrCode)lepcc_getBlobInfo(
                ctx, compressed, nInfo, &bt, &blobSize);

    int nBytes = (errCode == lepcc::ErrCode::Ok) ? (int)blobSize : -1;
    std::vector<uint16_t> intVec;
    if (nBytes > 0)
    {
        const lepcc::Byte* pByte = compressed;
        stat = lepcc_getIntensityCount(ctx, pByte, nBytes, &nPts);
        if (stat != (lepcc_status) lepcc::ErrCode::Ok)
            throw EsriError("Intensity point count fetch failed");
        intVec.resize(nPts);
        stat = lepcc_decodeIntensity(
                ctx, &pByte, nBytes, &nPts, (unsigned short*)(&intVec[0]));
        if (stat != (lepcc_status) lepcc::ErrCode::Ok)
            throw EsriError("Intensity decompression failed");
    }
    return intVec;
}

} // namespace i3s
} // namespace pdal
