/******************************************************************************
* Copyright (c) 2014, Howard Butler (howard@hobu.co)
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

#include <pdal/pdal_internal.hpp>
#include <functional>

namespace pdal
{

enum class CompressionType
{
    None = 0,
//    Ght = 1,   -- Removed compression type
    Dimensional = 2,
    Lazperf = 3,
    Unknown = 256
};

using BlockCb = std::function<void(char *buf, size_t bufsize)>;
const size_t CHUNKSIZE(1000000);

class compression_error : public std::runtime_error
{
public:
    compression_error() : std::runtime_error("General compression error")
    {}

    compression_error(const std::string& s) :
        std::runtime_error("Compression: " + s)
    {}
};


class PDAL_EXPORT Compressor
{
public:
    virtual ~Compressor()
    {}

    virtual void compress(const char *buf, size_t bufsize) = 0;
    virtual void done()
    {}
};


class PDAL_EXPORT Decompressor
{
public:
    virtual ~Decompressor()
    {}

    virtual void decompress(const char *buf, size_t bufsize) = 0;
    virtual void done()
    {}
};

} // namespace pdal

