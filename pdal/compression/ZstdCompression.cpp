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

#include "ZstdCompression.hpp"

#include <zstd.h>

namespace pdal
{

class ZstdCompressorImpl
{
public:
    ZSTD_CStream *m_strm;
    ZSTD_inBuffer m_inBuf;
    char m_tmpbuf[CHUNKSIZE];
    BlockCb m_cb;

    ZstdCompressorImpl(BlockCb cb, int compressionLevel) : m_cb(cb)
    {
        m_strm = ZSTD_createCStream();
        ZSTD_initCStream(m_strm, compressionLevel);
    }

    ~ZstdCompressorImpl()
    { ZSTD_freeCStream(m_strm); }

    void compress(const char *buf, size_t bufsize)
    {
        m_inBuf.src = reinterpret_cast<const void *>(buf);
        m_inBuf.size = bufsize;
        m_inBuf.pos = 0;

        size_t ret;
        do
        {
            ZSTD_outBuffer outBuf
            { reinterpret_cast<void *>(m_tmpbuf), CHUNKSIZE, 0 };
            ret = ZSTD_compressStream(m_strm, &outBuf, &m_inBuf);
            if (ZSTD_isError(ret))
                break;
            if (outBuf.pos)
                m_cb(m_tmpbuf, outBuf.pos);
        } while (m_inBuf.pos != m_inBuf.size);
    }

    void done()
    {
        size_t ret;
        do
        {
            ZSTD_outBuffer outBuf
                { reinterpret_cast<void *>(m_tmpbuf), CHUNKSIZE, 0 };
            ret = ZSTD_endStream(m_strm, &outBuf);
            if (ZSTD_isError(ret))
                break;
            if (outBuf.pos)
                m_cb(m_tmpbuf, outBuf.pos);
        } while (ret);
    }
};

ZstdCompressor::ZstdCompressor(BlockCb cb) :
    m_impl(new ZstdCompressorImpl(cb, 15))
{}


ZstdCompressor::ZstdCompressor(BlockCb cb, int compressionLevel) :
    m_impl(new ZstdCompressorImpl(cb, compressionLevel))
{}


ZstdCompressor::~ZstdCompressor()
{}


void ZstdCompressor::compress(const char *buf, size_t bufsize)
{
    m_impl->compress(buf, bufsize);
}


void ZstdCompressor::done()
{
    m_impl->done();
}


class ZstdDecompressorImpl
{
public:
    ZstdDecompressorImpl(BlockCb cb) : m_cb(cb)
    {
        m_strm = ZSTD_createDStream();
        ZSTD_initDStream(m_strm);
    }

    ~ZstdDecompressorImpl()
    {
        ZSTD_freeDStream(m_strm);
    }

    void decompress(const char *buf, size_t bufsize)
    {
        m_inBuf.src = reinterpret_cast<const void *>(buf);
        m_inBuf.size = bufsize;
        m_inBuf.pos = 0;

        size_t ret;
        do
        {
            ZSTD_outBuffer outBuf
                { reinterpret_cast<void *>(m_tmpbuf), CHUNKSIZE, 0 };
            ret = ZSTD_decompressStream(m_strm, &outBuf, &m_inBuf);
            if (ZSTD_isError(ret))
                break;
            if (outBuf.pos)
                m_cb(m_tmpbuf, outBuf.pos);
        } while (m_inBuf.pos != m_inBuf.size);
    }

private:
    BlockCb m_cb;

    ZSTD_DStream *m_strm;
    ZSTD_inBuffer m_inBuf;
    char m_tmpbuf[CHUNKSIZE];
};

ZstdDecompressor::ZstdDecompressor(BlockCb cb) :
    m_impl(new ZstdDecompressorImpl(cb))
{}


ZstdDecompressor::~ZstdDecompressor()
{}


void ZstdDecompressor::decompress(const char *buf, size_t bufsize)
{
    m_impl->decompress(buf, bufsize);
}

} // namespace pdal
