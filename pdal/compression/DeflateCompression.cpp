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

#include "DeflateCompression.hpp"
#include "GzipCompression.hpp"

#include <zlib.h>

namespace pdal
{

class DeflateCompressorImpl
{
public:
    DeflateCompressorImpl(BlockCb cb) : m_cb(cb)
    {
        m_strm.zalloc = Z_NULL;
        m_strm.zfree = Z_NULL;
        m_strm.opaque = Z_NULL;
        switch (deflateInit(&m_strm, Z_DEFAULT_COMPRESSION))
        {
        case Z_OK:
            return;
        case Z_MEM_ERROR:
            throw compression_error("Memory allocation failure.");
        case Z_STREAM_ERROR:
            throw compression_error("Internal error.");
        case Z_VERSION_ERROR:
            throw compression_error("Incompatible version.");
        default:
            throw compression_error();
        }
    }

    ~DeflateCompressorImpl()
    {
        deflateEnd(&m_strm);
    }

    void compress(const char *buf, size_t bufsize)
    {
        run(buf, bufsize, Z_NO_FLUSH);
    }

    void done()
    {
        run(nullptr, 0, Z_FINISH);
    }

private:
    void run(const char *buf, size_t bufsize, int mode)
    {
        auto handleError = [](int ret) -> void
        {
            switch (ret)
            {
            case Z_OK:
            case Z_STREAM_END:
                return;
            case Z_STREAM_ERROR:
                throw compression_error("Internal error.");
            case Z_DATA_ERROR:
                throw compression_error("Corrupted data.");
            case Z_MEM_ERROR:
                throw compression_error("Memory allocation failure.");
            default:
                std::cerr << "Compression error !\n";
                throw compression_error();
            }
        };

        if (buf)
        {
            m_strm.avail_in = bufsize;
            m_strm.next_in = reinterpret_cast<unsigned char *>(
                    const_cast<char *>(buf));
        }
        int ret = Z_OK;
        do
        {
            m_strm.avail_out = CHUNKSIZE;
            m_strm.next_out = m_tmpbuf;
            ret = ::deflate(&m_strm, mode);
            handleError(ret);
            size_t written = CHUNKSIZE - m_strm.avail_out;
            if (written)
                m_cb(reinterpret_cast<char *>(m_tmpbuf), written);
        } while (m_strm.avail_out == 0);
    }

private:
    BlockCb m_cb;
    z_stream m_strm;
    unsigned char m_tmpbuf[CHUNKSIZE];
};


DeflateCompressor::DeflateCompressor(BlockCb cb) :
    m_impl(new DeflateCompressorImpl(cb))
{}


DeflateCompressor::~DeflateCompressor()
{}


void DeflateCompressor::compress(const char *buf, size_t bufsize)
{
    m_impl->compress(buf, bufsize);
}


void DeflateCompressor::done()
{
    m_impl->done();
}


class DeflateDecompressorImpl
{
public:
    DeflateDecompressorImpl(BlockCb cb, int windowBits = 15) : m_cb(cb)
    {
        m_strm.zalloc = Z_NULL;
        m_strm.zfree = Z_NULL;
        m_strm.opaque = Z_NULL;
        m_strm.avail_in = 0;
        m_strm.next_in = Z_NULL;
        switch (inflateInit2(&m_strm, windowBits))
        {
        case Z_OK:
            return;
        case Z_MEM_ERROR:
            throw compression_error("Memory allocation failure.");
        case Z_STREAM_ERROR:
            throw compression_error("Internal error.");
        case Z_VERSION_ERROR:
            throw compression_error("Incompatible version.");
        default:
            throw compression_error();
        }
    }

    ~DeflateDecompressorImpl()
    {
        inflateEnd(&m_strm);
    }

    void decompress(const char *buf, size_t bufsize)
    {
        run(buf, bufsize, Z_NO_FLUSH);
    }

    void done()
    {
        run(nullptr, 0, Z_FINISH);
    }

private:
    void run(const char *buf, size_t bufsize, int mode)
    {
        auto handleError = [](int ret) -> void
        {
            switch (ret)
            {
            case Z_OK:
            case Z_STREAM_END:
                return;
            case Z_STREAM_ERROR:
                throw compression_error("Internal error.");
            case Z_DATA_ERROR:
                throw compression_error("Corrupted data.");
            case Z_MEM_ERROR:
                throw compression_error("Memory allocation failure.");
            default:
                throw compression_error();
            }
        };

        m_strm.next_in = reinterpret_cast<unsigned char *>(
            const_cast<char *>(buf));
        m_strm.avail_in = bufsize;
        int ret = Z_OK;
        do
        {
            m_strm.avail_out = CHUNKSIZE;
            m_strm.next_out = m_tmpbuf;
            ret = inflate(&m_strm, mode);
            handleError(ret);
            size_t written = CHUNKSIZE - m_strm.avail_out;
            if (written)
                m_cb(reinterpret_cast<char *>(m_tmpbuf), written);
        } while (m_strm.avail_out == 0);
    }

private:
    BlockCb m_cb;
    z_stream m_strm;
    unsigned char m_tmpbuf[CHUNKSIZE];
};


DeflateDecompressor::DeflateDecompressor(BlockCb cb) :
    m_impl(new DeflateDecompressorImpl(cb))
{}


DeflateDecompressor::~DeflateDecompressor()
{}


void DeflateDecompressor::decompress(const char *buf, size_t bufsize)
{
    m_impl->decompress(buf, bufsize);
}


void DeflateDecompressor::done()
{
    m_impl->done();
}

// GZIP

GzipDecompressor::GzipDecompressor(BlockCb cb) : m_impl(new DeflateDecompressorImpl(cb, 47))
{}


GzipDecompressor::~GzipDecompressor()
{}


void GzipDecompressor::decompress(const char *buf, size_t bufsize)
{
    m_impl->decompress(buf, bufsize);
}


void GzipDecompressor::done()
{
    m_impl->done();
}


} // namespace pdal
