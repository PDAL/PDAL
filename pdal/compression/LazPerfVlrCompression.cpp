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

#pragma push_macro("min")
#pragma push_macro("max")
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#include <laz-perf/common/common.hpp>
#include <laz-perf/compressor.hpp>
#include <laz-perf/decompressor.hpp>

#include <laz-perf/encoder.hpp>
#include <laz-perf/decoder.hpp>
#include <laz-perf/formats.hpp>
#include <laz-perf/io.hpp>
#include <laz-perf/las.hpp>

#pragma pop_macro("max")
#pragma pop_macro("min")

#include "LazPerfVlrCompression.hpp"

namespace pdal
{

// This compressor write data in chunks to a stream. At the beginning of the
// data is an offset to the end of the data, where the chunk table is
// stored.  The chunk table keeps a list of the offsets to the beginning of
// each chunk.  Chunks consist of a fixed number of points (last chunk may
// have fewer points).  Each time a chunk starts, the compressor is reset.
// This allows decompression of some set of points that's less than the
// entire set when desired.
// The compressor uses the schema of the point data in order to compress
// the point stream.  The schema is also stored in a VLR that isn't
// handled as part of the compression process itself.
class LazPerfVlrCompressorImpl
{
    typedef laszip::io::__ofstream_wrapper<std::ostream> OutputStream;
    typedef laszip::encoders::arithmetic<OutputStream> Encoder;
    typedef laszip::formats::las_compressor Compressor;

public:
    LazPerfVlrCompressorImpl(std::ostream& stream, int format, int ebCount, uint32_t chunksize) :
        m_stream(stream), m_outputStream(stream), m_format(format), m_ebCount(ebCount),
        m_chunksize(chunksize), m_chunkPointsWritten(0), m_chunkInfoPos(0),
        m_chunkOffset(0)
    {}

    std::vector<uint8_t> vlrData() const
    {
        laszip::laz_vlr vlr(m_format, m_ebCount, m_chunksize);
        return vlr.data();
    }

    void compress(const char *inbuf)
    {
        // First time through.
        if (!m_compressor)
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
        m_compressor->done();

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
        if (m_compressor)
            m_compressor->done();
        m_compressor = laszip::factory::build_las_compressor(m_outputStream, m_format, m_ebCount);
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
    Compressor::ptr m_compressor;
    int m_format;
    int m_ebCount;
    uint32_t m_chunksize;
    uint32_t m_chunkPointsWritten;
    std::streampos m_chunkInfoPos;
    std::streampos m_chunkOffset;
    std::vector<uint32_t> m_chunkTable;
};


LazPerfVlrCompressor::LazPerfVlrCompressor(std::ostream& stream, int format, int ebCount,
        uint32_t chunksize) :
    m_impl(new LazPerfVlrCompressorImpl(stream, format, ebCount, chunksize))
{}


LazPerfVlrCompressor::LazPerfVlrCompressor(std::ostream& stream, int format, int ebCount) :
    m_impl(new LazPerfVlrCompressorImpl(stream, format, ebCount, DefaultChunkSize))
{}


LazPerfVlrCompressor::~LazPerfVlrCompressor()
{}


std::vector<uint8_t> LazPerfVlrCompressor::vlrData() const
{
    return m_impl->vlrData();
}


void LazPerfVlrCompressor::compress(const char *inbuf)
{
    m_impl->compress(inbuf);
}


void LazPerfVlrCompressor::done()
{
    m_impl->done();
}


class LazPerfVlrDecompressorImpl
{
public:
    LazPerfVlrDecompressorImpl(std::istream& stream, int format, int ebCount,
        std::streamoff pointOffset, uint32_t chunksize) :
        m_stream(stream), m_inputStream(stream), m_format(format), m_ebCount(ebCount),
        m_chunksize(chunksize), m_chunkPointsRead(0)
    {
        m_stream.seekg(pointOffset + sizeof(int64_t));
    }

    void decompress(char *outbuf)
    {
        if (m_chunkPointsRead == m_chunksize || !m_decompressor)
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
        m_decompressor =
            laszip::factory::build_las_decompressor(m_inputStream, m_format, m_ebCount);
    }

    typedef laszip::io::__ifstream_wrapper<std::istream> InputStream;
    typedef laszip::formats::las_decompressor Decompressor;

    std::istream& m_stream;
    InputStream m_inputStream;
    Decompressor::ptr m_decompressor;
    int m_format;
    int m_ebCount;
    uint32_t m_chunksize;
    uint32_t m_chunkPointsRead;
};

LazPerfVlrDecompressor::LazPerfVlrDecompressor(std::istream& stream, int format,
        int ebCount, std::streamoff pointOffset, const char *vlrData) :
    m_impl(new LazPerfVlrDecompressorImpl(stream, format, ebCount, pointOffset,
            laszip::laz_vlr(vlrData).chunk_size))
{}


LazPerfVlrDecompressor::~LazPerfVlrDecompressor()
{}


void LazPerfVlrDecompressor::decompress(char *outbuf)
{
    m_impl->decompress(outbuf);
}

} // namespace pdal

