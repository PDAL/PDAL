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
#include "i3sReceiver.hpp"

namespace pdal
{
    std::vector<lepcc::Point3D> decompressXYZ(std::vector<char>* compData)
    {
        unsigned char* c = new unsigned char [compData->size()];

        std::copy(compData->begin(), compData->end(), c);

        const unsigned char* compressed = c;
        int nInfo = lepcc_getBlobInfoSize();
        lepcc_ContextHdl ctx(nullptr);
        ctx = lepcc_createContext();
        lepcc_blobType bt;
        lepcc::uint32 blobSize = 0;


        lepcc::Byte vec;
        lepcc_status stat;
        std::vector<lepcc::Point3D> decVec;
        lepcc::uint32 xyzPts = 0;

        lepcc::ErrCode errCode =
            (lepcc::ErrCode)lepcc_getBlobInfo(
                    ctx, compressed, nInfo, &bt, &blobSize);

        const lepcc::Byte* pByte = &compressed[0];
        int nBytes = (errCode == lepcc::ErrCode::Ok) ? (int)blobSize : -1;
        if (nBytes > 0)
        {
            stat = lepcc_getPointCount(ctx, pByte, nBytes, &xyzPts);
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << "lepcc_getPointCount() failed. \n";
                return decVec;//TODO throw error here
            }
            decVec.resize(xyzPts);
            stat = lepcc_decodeXYZ(
                    ctx, &pByte, nBytes, &xyzPts, (double*)(&decVec[0]));
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << "lepcc_decodeXYZ() failed. \n";
                return decVec;//TODO throw error here
            }
        }
        return decVec;
    }


    std::vector<lepcc::RGB_t> decompressRGB(std::vector<char>* compData)
    {
        unsigned char* c = new unsigned char [compData->size()];

        std::copy(compData->begin(), compData->end(), c);

        const unsigned char* compressed = c;
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
        const lepcc::Byte* pByte = &compressed[0];
        if (nBytes > 0)
        {
            stat = lepcc_getRGBCount(ctx, pByte, nBytes, &nPts);
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << stat << std::endl;
                std::cout << "lepcc_getPointRGB() failed. \n";
                //TODO throw error here
            }
            rgbVec.resize(nPts);
            stat = lepcc_decodeRGB(
                    ctx, &pByte, nBytes, &nPts, (lepcc::Byte*)(&rgbVec[0]));
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << "lepcc_decodergb() failed. \n";
                return rgbVec;//TODO throw error here
            }
        }
        return rgbVec;
    }


    std::vector<uint16_t> decompressIntensity(std::vector<char>* compData)
    {

        unsigned char* c = new unsigned char [compData->size()];

        std::copy(compData->begin(), compData->end(), c);

        const unsigned char* compressed = c;
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
        const lepcc::Byte* pByte = &compressed[0];
        std::vector<uint16_t> intVec;
        if (nBytes > 0)
        {
            stat = lepcc_getIntensityCount(ctx, pByte, nBytes, &nPts);
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << stat << std::endl;
                std::cout << "lepcc_getPointRGB() failed. \n";
                //TODO throw error here
            }
            intVec.resize(nPts);
            stat = lepcc_decodeIntensity(
                    ctx, &pByte, nBytes, &nPts, (unsigned short*)(&intVec[0]));
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << "lepcc_decodergb() failed. \n";
                return intVec;//TODO throw error here
            }

            return intVec;
        }
        return intVec;
    }
}
