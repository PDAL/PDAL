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
#include <laz-perf/io.hpp>
#include <laz-perf/las.hpp>
#endif

#include <pdal/Dimension.hpp>
#include <pdal/util/OStream.hpp>

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


// This is a utility input/output buffer for the compressor/decompressor.
template <typename CTYPE = unsigned char>
class TypedLazPerfBuf
{
    typedef std::vector<CTYPE> LazPerfRawBuf;
private:

    LazPerfRawBuf& m_buf;
    size_t m_idx;

public:
    TypedLazPerfBuf(LazPerfRawBuf& buf) : m_buf(buf), m_idx(0)
    {}

    void putBytes(const unsigned char *b, size_t len)
        { m_buf.insert(m_buf.end(), (CTYPE *)b, (CTYPE *)(b + len)); }
    void putByte(const unsigned char b)
        {   m_buf.push_back((CTYPE)b); }
    unsigned char getByte()
        { return (unsigned char)m_buf[m_idx++]; }
    void getBytes(unsigned char *b, int len)
    {
        memcpy(b, m_buf.data() + m_idx, len);
        m_idx += len;
    }
};
typedef TypedLazPerfBuf<char> SignedLazPerfBuf;
typedef TypedLazPerfBuf<unsigned char> LazPerfBuf;


#ifdef PDAL_HAVE_LAZPERF
template<typename OutputStream>
class LazPerfCompressor
{
public:
    LazPerfCompressor(OutputStream& output, const DimTypeList& dims) :
        m_encoder(output),
        m_compressor(laszip::formats::make_dynamic_compressor(m_encoder)),
        m_pointSize(0),
        m_done(false)
    { m_pointSize = addFields(m_compressor, dims); }

    ~LazPerfCompressor()
    {
        if (!m_done)
            std::cerr << "LazPerfCompressor destroyed without a call "
               "to done()";
    }

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
    {
        if (!m_done)
           m_encoder.done();
        m_done = true;
    }

private:
    typedef laszip::encoders::arithmetic<OutputStream> Encoder;
    Encoder m_encoder;
    typedef typename laszip::formats::dynamic_field_compressor<Encoder>::ptr
            Compressor;
    Compressor m_compressor;
    size_t m_pointSize;
    bool m_done;
};

class LazPerfVlrCompressor
{
    typedef laszip::io::__ofstream_wrapper<std::ostream> OutputStream;
    typedef laszip::encoders::arithmetic<OutputStream> Encoder;
    typedef laszip::formats::dynamic_compressor Compressor;
    typedef laszip::factory::record_schema Schema;

public:
    LazPerfVlrCompressor(std::ostream& stream, const Schema& schema,
        uint32_t chunksize) :
        m_stream(stream), m_outputStream(stream), m_schema(schema),
        m_chunksize(chunksize), m_chunkPointsWritten(0), m_chunkInfoPos(0),
        m_chunkOffset(0)
    {}

    ~LazPerfVlrCompressor()
    {
        if (m_encoder)
            std::cerr << "LazPerfVlrCompressor destroyed without a call "
               "to done()";
    }


    void compress(const char *inbuf)
    {
        // First time through.
        if (!m_encoder || !m_compressor)
        {
            // Get the position 
            m_chunkInfoPos = m_stream.tellp();
            // Seek over the chunk info offset value
            m_stream.seekp(sizeof(uint64_t), std::ios::cur);
            m_chunkOffset = m_stream.tellp();
            resetCompressor();
        }
        else if (m_chunkPointsWritten == m_chunksize)
        {
            resetCompressor();
            newChunk();
        }
        m_compressor->compress(inbuf);
        m_chunkPointsWritten++;
    }

    void done()
    {
        // Close and clear the point encoder.
        m_encoder->done();
        m_encoder.reset();

        newChunk();

        // Save our current position.  Go to the location where we need
        // to write the chunk table offset at the beginning of the point data.
        std::streampos chunkTablePos = m_stream.tellp();
        m_stream.seekp(m_chunkInfoPos);
        OLeStream out(&m_stream);
        out << (uint64_t)chunkTablePos;

        // Move to the start of the chunk table.
        m_stream.seekp(chunkTablePos);

        // Write the chunk table header.
        out << (uint32_t)0;  // Version (?)
        out << (uint32_t)m_chunkTable.size();

        // Encode and write the chunk table.
        OutputStream outputStream(m_stream);
        Encoder encoder(outputStream);
        laszip::compressors::integer compressor(32, 2);
        compressor.init();

        uint32_t predictor = 0;
        for (uint32_t offset : m_chunkTable)
        {
            offset = htole32(offset);
            compressor.compress(encoder, predictor, offset, 1);
            predictor = offset;
        }
        encoder.done();
    }

private:
    void resetCompressor()
    {
        if (m_encoder)
            m_encoder->done();
        m_encoder.reset(new Encoder(m_outputStream));
        m_compressor = laszip::factory::build_compressor(*m_encoder, m_schema);
    }

    void newChunk()
    {
        std::streampos offset = m_stream.tellp();
        m_chunkTable.push_back((uint32_t)(offset - m_chunkOffset));
        m_chunkOffset = offset;
        m_chunkPointsWritten = 0;
    }

    std::ostream& m_stream;
    OutputStream m_outputStream;
    std::unique_ptr<Encoder> m_encoder;
    Compressor::ptr m_compressor;
    Schema m_schema;
    uint32_t m_chunksize;
    uint32_t m_chunkPointsWritten;
    std::streampos m_chunkInfoPos;
    std::streampos m_chunkOffset;
    std::vector<uint32_t> m_chunkTable;
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

class LazPerfVlrDecompressor
{
public:
    LazPerfVlrDecompressor(std::istream& stream, const char *vlrData,
        std::streamoff pointOffset) :
        m_stream(stream), m_inputStream(stream), m_chunksize(0),
        m_chunkPointsRead(0)
    {
        laszip::io::laz_vlr zipvlr(vlrData);
        m_chunksize = zipvlr.chunk_size;
        m_schema = laszip::io::laz_vlr::to_schema(zipvlr);
        m_stream.seekg(pointOffset + sizeof(int64_t));
    }

    size_t pointSize() const
        { return (size_t)m_schema.size_in_bytes(); }

    void decompress(char *outbuf)
    {
        if (m_chunkPointsRead == m_chunksize || !m_decoder || !m_decompressor)
        {
            resetDecompressor();
            m_chunkPointsRead = 0;
        }
        m_decompressor->decompress(outbuf);
        m_chunkPointsRead++;
    }

private:
    void resetDecompressor()
    {
        m_decoder.reset(new Decoder(m_inputStream));
        m_decompressor =
            laszip::factory::build_decompressor(*m_decoder, m_schema);
    }

    typedef laszip::io::__ifstream_wrapper<std::istream> InputStream;
    typedef laszip::decoders::arithmetic<InputStream> Decoder;
    typedef laszip::formats::dynamic_decompressor Decompressor;
    typedef laszip::factory::record_schema Schema;

    std::istream& m_stream;
    InputStream m_inputStream;
    std::unique_ptr<Decoder> m_decoder;
    Decompressor::ptr m_decompressor;
    Schema m_schema;
    uint32_t m_chunksize;
    uint32_t m_chunkPointsRead;
};

#else

typedef char LazPerfVlrCompressor;
typedef char LazPerfVlrDecompressor;

#endif  // PDAL_HAVE_LAZPERF

} // namespace pdal

