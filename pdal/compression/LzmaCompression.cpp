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

#include "LzmaCompression.hpp"

#include <lzma.h>

namespace pdal
{

class Lzma
{
protected:
    Lzma(BlockCb cb) : m_cb(cb)
    {
        m_strm = LZMA_STREAM_INIT;
    }

    ~Lzma()
    {
        lzma_end(&m_strm);
    }

    void run(const char *buf, size_t bufsize, lzma_action mode)
    {
        m_strm.avail_in = bufsize;
        m_strm.next_in = reinterpret_cast<unsigned char *>(
            const_cast<char *>(buf));
        int ret = LZMA_OK;
        do
        {
            m_strm.avail_out = CHUNKSIZE;
            m_strm.next_out = m_tmpbuf;
            ret = lzma_code(&m_strm, mode);
            size_t written = CHUNKSIZE - m_strm.avail_out;
            if (written)
                m_cb(reinterpret_cast<char *>(m_tmpbuf), written);
        } while (ret == LZMA_OK);
        if (ret == LZMA_STREAM_END)
            return;

        switch (ret)
        {
        case LZMA_MEM_ERROR:
            throw compression_error("Memory allocation failure.");
        case LZMA_DATA_ERROR:
            throw compression_error("LZMA data error.");
        case LZMA_OPTIONS_ERROR:
            throw compression_error("Unsupported option.");
        case LZMA_UNSUPPORTED_CHECK:
            throw compression_error("Unsupported integrity check.");
        }
    }

protected:
    lzma_stream m_strm;

private:
    unsigned char m_tmpbuf[CHUNKSIZE];
    BlockCb m_cb;
};


class LzmaCompressorImpl : public Lzma
{
public:
    LzmaCompressorImpl(BlockCb cb) : Lzma(cb)
    {
        if (lzma_easy_encoder(&m_strm, 2, LZMA_CHECK_CRC64) != LZMA_OK)
            throw compression_error("Can't create compressor");
    }

    void compress(const char *buf, size_t bufsize)
    {
        run(buf, bufsize, LZMA_RUN);
    }

    void done()
    {
        run(nullptr, 0, LZMA_FINISH);
    }
};


LzmaCompressor::LzmaCompressor(BlockCb cb) :
    m_impl(new LzmaCompressorImpl(cb))
{}


LzmaCompressor::~LzmaCompressor()
{}


void LzmaCompressor::compress(const char *buf, size_t bufsize)
{
    m_impl->compress(buf, bufsize);
}


void LzmaCompressor::done()
{
    m_impl->done();
}


class LzmaDecompressorImpl : public Lzma
{
public:
    LzmaDecompressorImpl(BlockCb cb) : Lzma(cb)
    {
        if (lzma_auto_decoder(&m_strm, (std::numeric_limits<uint32_t>::max)(),
            LZMA_TELL_UNSUPPORTED_CHECK))
            throw compression_error("Can't create decompressor");
    }

    void decompress(const char *buf, size_t bufsize)
    {
        run(buf, bufsize, LZMA_RUN);
    }

    void done()
    {
        run(nullptr, 0, LZMA_FINISH);
    }
};

LzmaDecompressor::LzmaDecompressor(BlockCb cb) :
    m_impl(new LzmaDecompressorImpl(cb))
{}


LzmaDecompressor::~LzmaDecompressor()
{}


void LzmaDecompressor::decompress(const char *buf, size_t bufsize)
{
    m_impl->decompress(buf, bufsize);
}


void LzmaDecompressor::done()
{
    m_impl->done();
}

} // namespace pdal
