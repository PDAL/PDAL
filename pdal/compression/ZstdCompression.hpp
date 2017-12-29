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

#ifdef PDAL_HAVE_ZSTD

#include <pdal/Compression.hpp>

#include <zstd.h>

namespace pdal
{

class ZstdCompressor
{
public:
    ZstdCompressor(BlockCb cb) : m_cb(cb)
    {
        m_strm = ZSTD_createCStream();
        ZSTD_initCStream(m_strm, 15);
    }

    ~ZstdCompressor()
        { ZSTD_freeCStream(m_strm); }


    void compress(const char *buf, size_t bufsize)
    {
        std::cerr << "Compress!\n";
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
            std::cerr << "In buf pos/size = " << m_inBuf.pos << "/" << m_inBuf.size << "!\n";
            std::cerr << "Out buf pos/size = " << outBuf.pos << "/" << outBuf.size << "!\n";
        } while (m_inBuf.pos != m_inBuf.size);
    }

    void done()
    {
        std::cerr << "Done compression!\n";
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

private:
    BlockCb m_cb;

    ZSTD_CStream *m_strm;
    ZSTD_inBuffer m_inBuf;
    char m_tmpbuf[CHUNKSIZE];
};

class ZstdDecompressor
{
public:
    ZstdDecompressor(BlockCb cb) : m_cb(cb)
    {
        m_strm = ZSTD_createDStream();
        ZSTD_initDStream(m_strm);
    }

    ~ZstdDecompressor()
    {
        ZSTD_freeDStream(m_strm);
    }

    void decompress(const char *buf, size_t bufsize)
    {
        std::cerr << "DE-Compress!\n";
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

    void done()
    {}

private:
    BlockCb m_cb;

    ZSTD_DStream *m_strm;
    ZSTD_inBuffer m_inBuf;
    char m_tmpbuf[CHUNKSIZE];
};

} // namespace pdal

#endif // PDAL_HAVE_ZSTD
