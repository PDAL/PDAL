/******************************************************************************
* Copyright (c) 2015, Hobu Inc., hobu@hobu.co
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

#include <stdexcept>
#include <ostream>

#include <pdal/pdal_features.hpp>
#include <pdal/util/Charbuf.hpp>
#include <pdal/util/OStream.hpp>

#ifdef PDAL_HAVE_ZLIB
#include <zlib.h>
#endif // PDAL_HAVE_ZLIB

namespace pdal
{

class BpfCompressor
{
public:
    struct error : public std::runtime_error
    {
        error(const std::string& err) : std::runtime_error(err)
        {}
    };

#ifdef PDAL_HAVE_ZLIB
    BpfCompressor(OLeStream& out, size_t maxSize) :
        m_out(out), m_inbuf(maxSize), m_blockStart(out), m_rawSize(0),
        m_compressedSize(0)
    {}
#else
    BpfCompressor(OLeStream&, size_t)
    {}
#endif // PDAL_HAVE_ZLIB

    void startBlock();
    void finish();
    void compress();

private:
    static const int CHUNKSIZE = 1000000;

#ifdef PDAL_HAVE_ZLIB
    OLeStream& m_out;
    Charbuf m_charbuf;
    std::vector<char> m_inbuf;
    z_stream m_strm;
    unsigned char m_tmpbuf[CHUNKSIZE];
    OStreamMarker m_blockStart;
    size_t m_rawSize;
    size_t m_compressedSize;
#endif // PDAL_HAVE_ZLIB
};

} // namespace pdal

