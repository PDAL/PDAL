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

#include <pdal/util/OStream.hpp>

#pragma push_macro("min")
#pragma push_macro("max")
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif


#pragma pop_macro("max")
#pragma pop_macro("min")

#include "DracoCompression.hpp"

#include <draco/point_cloud/point_cloud_builder.h>

namespace pdal
{

namespace
{

template<typename DracoEngine>
size_t addFields(DracoEngine& engine, const DimTypeList& dims)
{
    using namespace Dimension;
    using namespace draco;

    size_t pointSize = 0;
    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        switch (di->m_type)
        {
        case Type::Double:
            engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_FLOAT64);
            break;
        case Type::Float:
            engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_FLOAT32);
            break;
        case Type::Signed64:
            engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_INT64);
            break;
        case Type::Signed32:
            engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_INT32);
            break;
        case Type::Signed16:
            engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_INT16);
            break;
        case Type::Signed8:
            engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_INT8);
            break;
        case Type::Unsigned64:
            engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_UINT64);
            break;
        case Type::Unsigned32:
            engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_UINT32);
            break;
        case Type::Unsigned16:
            engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_UINT16);
            break;
        case Type::Unsigned8:
            engine.AddAttribute(GeometryAttribute::GENERIC, 1, DT_UINT8);
            break;
        default:
            return 0;
        }
        pointSize += Dimension::size(di->m_type);
    }
    return pointSize;
}

} // anonymous namespace


class DracoCompressorImpl
{
public:
    DracoCompressorImpl(BlockCb cb, const DimTypeList& dims) :
        m_cb(cb),
//         m_encoder(*this),
        m_pointSize(0),
        m_avail(CHUNKSIZE)
    {
        m_compressor.Start(CHUNKSIZE);
        m_pointSize = addFields(m_compressor, dims);
    }

    void putBytes(const uint8_t* buf, size_t bufsize)
    {
        while (bufsize)
        {
            size_t copyCnt = (std::min)(m_avail, bufsize);
            std::copy(buf, buf + copyCnt, m_tmpbuf + (CHUNKSIZE - m_avail));
            m_avail -= copyCnt;
            if (m_avail == 0)
            {
                m_cb(reinterpret_cast<char *>(m_tmpbuf), CHUNKSIZE);
                m_avail = CHUNKSIZE;
            }
            buf += copyCnt;
            bufsize -= copyCnt;
        }
    }

    void putByte(unsigned char b)
    {
        m_tmpbuf[CHUNKSIZE - m_avail] = b;
        if (--m_avail == 0)
        {
            m_cb(reinterpret_cast<char *>(m_tmpbuf), CHUNKSIZE);
            m_avail = CHUNKSIZE;
        }
    }

    void compress(const char *buf, size_t bufsize)
    {
        while (bufsize >= m_pointSize)
        {
//             m_compressor->compress(buf);
            buf += m_pointSize;
            bufsize -= m_pointSize;
        }
    }

    void done()
    {
        m_encoder.Finalize(false);
        if (m_avail != CHUNKSIZE)
            m_cb(reinterpret_cast<char *>(m_tmpbuf), CHUNKSIZE - m_avail);
        m_avail = CHUNKSIZE;
    }

private:
    BlockCb m_cb;
    draco::EncoderBuffer m_compressor;
    size_t m_pointSize;
    unsigned char m_tmpbuf[CHUNKSIZE];
    size_t m_avail;
};


DracoCompressor::DracoCompressor(BlockCb cb, const DimTypeList& dims) :
    m_impl(new DracoCompressorImpl(cb, dims))
{}


DracoCompressor::~DracoCompressor()
{}


void DracoCompressor::compress(const char *buf, size_t bufsize)
{
    m_impl->compress(buf, bufsize);
}


void DracoCompressor::done()
{
    m_impl->done();
}


class DracoDecompressorImpl
{
public:
    DracoDecompressorImpl(BlockCb cb, const DimTypeList& dims,
            size_t numPoints) :
//         m_decoder(*this),
//         m_decompressor(laszip::formats::make_dynamic_decompressor(m_decoder)),
        m_cb(cb),
        m_numPoints(numPoints)
    {
//         m_pointSize = addFields(m_decompressor, dims);
    }

    unsigned char getByte()
    {
        if (m_srcsize)
        {
            m_srcsize--;
            return *m_srcbuf++;
        }
        return 0;
    }

    void getBytes(uint8_t *dst, size_t dstsize)
    {
        size_t count = (std::min)(dstsize, m_srcsize);
        uint8_t *src = const_cast<uint8_t *>(
            reinterpret_cast<const uint8_t *>(m_srcbuf));
        std::copy(src, src + count, dst);
        m_srcbuf += count;
        m_srcsize -= count;
    }

    void decompress(const char *buf, size_t bufsize)
    {
        m_srcbuf = buf;
        m_srcsize = bufsize;

        std::vector<char> outbuf(m_pointSize);
        while (m_numPoints--)
        {
//             m_decompressor->decompress(outbuf.data());
            m_cb(outbuf.data(), m_pointSize);
        }
    }

private:
//     typedef laszip::decoders::arithmetic<DracoDecompressorImpl> Decoder;
//     Decoder m_decoder;
//     typedef typename laszip::formats::dynamic_field_decompressor<Decoder>::ptr
//         Decompressor;
//     Decompressor m_decompressor;
    BlockCb m_cb;
    size_t m_numPoints;
    const char *m_srcbuf;
    size_t m_srcsize;
    size_t m_pointSize;
};

DracoDecompressor::DracoDecompressor(BlockCb cb, const DimTypeList& dims,
        size_t numPoints) :
    m_impl(new DracoDecompressorImpl(cb, dims, numPoints))
{}


DracoDecompressor::~DracoDecompressor()
{}


void DracoDecompressor::decompress(const char *buf, size_t bufsize)
{
    m_impl->decompress(buf, bufsize);
}

} // namespace pdal


