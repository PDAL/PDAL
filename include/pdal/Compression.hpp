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
#include <boost/algorithm/string.hpp>

#ifdef PDAL_HAVE_LAZPERF
#include <laz-perf/common/common.hpp>
#include <laz-perf/compressor.hpp>
#include <laz-perf/decompressor.hpp>

#include <laz-perf/encoder.hpp>
#include <laz-perf/decoder.hpp>
#include <laz-perf/formats.hpp>
#include <laz-perf/las.hpp>
#endif

#include <pdal/Dimension.hpp>

#include <map>
#include <vector>

namespace pdal
{

namespace
{

template<typename LasZipEngine>
size_t addFields(LasZipEngine& engine, const DimTypeList& dims)
{
    using namespace Dimension;

    size_t pointSize = 0;
    for (auto di = dims.begin(); di != dims.end(); ++di)
    {
        switch (di->m_type)
        {
        case Type::Signed64:
            engine->template add_field<int32_t>();
            engine->template add_field<int32_t>();
            break;
        case Type::Signed32:
        case Type::Float:
            engine->template add_field<int32_t>();
            break;
        case Type::Signed16:
            engine->template add_field<int16_t>();
            break;
        case Type::Signed8:
            engine->template add_field<int8_t>();
            break;
        case Type::Unsigned64:
        case Type::Double:
            engine->template add_field<uint32_t>();
            engine->template add_field<uint32_t>();
            break;
        case Type::Unsigned32:
            engine->template add_field<uint32_t>();
            break;
        case Type::Unsigned16:
            engine->template add_field<uint16_t>();
            break;
        case Type::Unsigned8:
            engine->template add_field<uint8_t>();
            break;
        default:
            return 0;
        }
        pointSize += Dimension::size(di->m_type);
    }
    return pointSize;
}

} // anonymous namespace


namespace CompressionType
{

enum Enum
{
    None = 0,
    Ght = 1,
    Dimensional = 2,
    Lazperf = 3,
    Unknown = 256
};

} // namespace CompressionType


#ifdef PDAL_HAVE_LAZPERF
template<typename OutputStream>
class LazPerfCompressor
{
public:
    LazPerfCompressor(OutputStream& output, const DimTypeList& dims) :
        m_encoder(output),
        m_compressor(laszip::formats::make_dynamic_compressor(m_encoder)),
        m_pointSize(0)
    { m_pointSize = addFields(m_compressor, dims); }

    ~LazPerfCompressor()
        { done(); }

    size_t pointSize() const
        { return m_pointSize; }
    point_count_t compress(const char *inbuf, size_t bufsize)
    {
        point_count_t numRead = 0;

        const char *end = inbuf + bufsize;
        while (inbuf + m_pointSize <= end)
        {
            m_compressor->compress(inbuf);
            inbuf += m_pointSize;
            numRead++;
        }
        return numRead;
    }
    void done()
        { m_encoder.done(); }

private:
    typedef laszip::encoders::arithmetic<OutputStream> Encoder;
    Encoder m_encoder;
    typedef typename laszip::formats::dynamic_field_compressor<Encoder>::ptr
            Compressor;
    Compressor m_compressor;
    size_t m_pointSize;
};


template<typename InputStream>
class LazPerfDecompressor
{
public:
    LazPerfDecompressor(InputStream& input, const DimTypeList& dims) :
        m_decoder(input),
        m_decompressor(laszip::formats::make_dynamic_decompressor(m_decoder))
    { m_pointSize = addFields(m_decompressor, dims); }

    size_t pointSize() const
        { return m_pointSize; }
    point_count_t decompress(char *outbuf, size_t bufsize)
    {
        point_count_t numWritten = 0;

        char *end = outbuf + bufsize;
        while (outbuf + m_pointSize <= end)
        {
            m_decompressor->decompress(outbuf);
            outbuf += m_pointSize;
            numWritten++;
        }
        return numWritten;
    }

private:
    typedef laszip::decoders::arithmetic<InputStream> Decoder;
    Decoder m_decoder;
    typedef typename laszip::formats::dynamic_field_decompressor<Decoder>::ptr
        Decompressor;
    Decompressor m_decompressor;
    size_t m_pointSize;
};
#endif  // PDAL_HAVE_LAZPERF

} // namespace pdal

