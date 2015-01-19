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

#include "BpfCompressor.hpp"

#include <pdal/pdal_internal.hpp>

namespace pdal
{

BpfCompressor::BpfCompressor(std::ostream& out, int compressionLevel) :
    m_out(out), m_written(0)
{
    m_strm.zalloc = Z_NULL;
    m_strm.zfree = Z_NULL;
    m_strm.opaque = Z_NULL;
    if (deflateInit(&m_strm, Z_DEFAULT_COMPRESSION) != Z_OK)
        throw pdal_error("Could not initialize BPF compressor.");
}


void BpfCompressor::compress(char *buf, size_t insize)
{
    m_strm.avail_in = insize;
    m_strm.next_in = (unsigned char *)buf;
    m_strm.avail_out = CHUNKSIZE;
    m_strm.next_out = m_tmpbuf;

    while (m_strm.avail_in)
    {
        int ret = ::deflate(&m_strm, Z_NO_FLUSH);
        size_t written = CHUNKSIZE - m_strm.avail_out;
        m_written += written;
        m_out.write((const char *)m_tmpbuf, written);
        m_strm.avail_out = CHUNKSIZE;
        m_strm.next_out = m_tmpbuf;
    }
}


size_t BpfCompressor::finish()
{
    int ret = Z_OK;
    while (ret == Z_OK)
    {
        ret = ::deflate(&m_strm, Z_FINISH);
        size_t written = CHUNKSIZE - m_strm.avail_out;
        m_written += written;
        m_out.write((const char *)m_tmpbuf, written);
        m_strm.avail_out = CHUNKSIZE;
        m_strm.next_out = m_tmpbuf;
    }
    if (ret != Z_STREAM_END)
        throw pdal_error("Couldn't close BPF compression stream.");
    deflateEnd(&m_strm);
    return m_written;
}
   
} // namespace pdal
