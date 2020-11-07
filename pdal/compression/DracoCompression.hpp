/******************************************************************************
* Copyright (c) 2020, Howard Butler (howard@hobu.co)
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
#pragma once

#include "Compression.hpp"

#include <pdal/DimType.hpp>


namespace pdal
{

class DracoCompressorImpl;

class DracoCompressor : public Compressor
{
public:
    PDAL_DLL DracoCompressor(BlockCb cb, const DimTypeList& dims);
    PDAL_DLL ~DracoCompressor();

    PDAL_DLL void compress(const char *buf, size_t bufsize);
    PDAL_DLL void done();

private:
    std::unique_ptr<DracoCompressorImpl> m_impl;
};


class DracoDecompressorImpl;

// NOTE - The DracoDecompressor is different from others, even though the
//   interface is the same, in that it always executes the callback after
//   a point's worth of data is read.
class DracoDecompressor : public Decompressor
{
public:
    PDAL_DLL DracoDecompressor(BlockCb cb, const DimTypeList& dims,
        size_t numPoints);
    PDAL_DLL ~DracoDecompressor();

    PDAL_DLL void decompress(const char *buf, size_t bufsize);

private:
    std::unique_ptr<DracoDecompressorImpl> m_impl;
};


} // pdal
