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

// This only exist in version 1.3+, so is an acceptable version test for now.
#include <lazperf/lazperf.hpp>
#include <lazperf/filestream.hpp>
#include <lazperf/vlr.hpp>
#ifndef LAZPERF_VERSION
#error "LAZperf version 2+ (supporting LAS version 1.4) not found
#endif

#include <pdal/util/IStream.hpp>
#include <pdal/util/OStream.hpp>

#include "LazPerfVlrCompression.hpp"

namespace pdal
{

namespace
{

size_t baseCount(int format)
{
    switch (format)
    {
    case 0:
        return 20;
    case 1:
        return 28;
    case 2:
        return 26;
    case 3:
        return 34;
    case 6:
        return 30;
    case 7:
        return 36;
    case 8:
        return 38;
    default:
        return 0;
    }
}

} // unnamed namespace

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
public:
    LazPerfVlrCompressorImpl(std::ostream& stream, int format, int ebCount) :
        m_stream(stream), m_outputStream(stream), m_format(format), m_ebCount(ebCount),
        m_chunksize(50000), m_chunkPointsWritten(0), m_chunkInfoPos(0), m_chunkOffset(0)
    {}

    std::vector<char> vlrData() const
    {
        lazperf::laz_vlr vlr(m_format, m_ebCount, m_chunksize);
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

        lazperf::OutFileStream stream(m_stream);
        lazperf::compress_chunk_table(stream.cb(), m_chunkTable);
    }

private:
    void resetCompressor()
    {
        if (m_compressor)
            m_compressor->done();
        m_compressor = lazperf::build_las_compressor(m_outputStream.cb(), m_format, m_ebCount);
    }

    void newChunk()
    {
        std::streampos offset = m_stream.tellp();
        m_chunkTable.push_back((uint32_t)(offset - m_chunkOffset));
        m_chunkOffset = offset;
        m_chunkPointsWritten = 0;
    }

    std::ostream& m_stream;
    lazperf::OutFileStream m_outputStream;
    lazperf::las_compressor::ptr m_compressor;
    int m_format;
    int m_ebCount;
    uint32_t m_chunksize;
    uint32_t m_chunkPointsWritten;
    std::streampos m_chunkInfoPos;
    std::streampos m_chunkOffset;
    std::vector<uint32_t> m_chunkTable;
};


LazPerfVlrCompressor::LazPerfVlrCompressor(std::ostream& stream, int format, int ebCount) :
    m_impl(new LazPerfVlrCompressorImpl(stream, format, ebCount))
{}


LazPerfVlrCompressor::~LazPerfVlrCompressor()
{}


std::vector<char> LazPerfVlrCompressor::vlrData() const
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
            std::streamoff pointOffset, const char *vlrdata) :
        m_stream(stream), m_fileStream(stream), m_format(format), m_ebCount(ebCount),
        m_chunkPointsRead(0), m_vlr(vlrdata)
    {
        m_stream.seekg(pointOffset);
        ILeStream in(&stream);

        uint64_t chunkTablePos;
        in >> chunkTablePos;

        in.seek(chunkTablePos);
        uint32_t version;
        uint32_t numChunks;
        in >> version;
        in >> numChunks;

        assert(version == 0);

        std::vector<uint32_t> chunks =
            lazperf::decompress_chunk_table(m_fileStream.cb(), numChunks);
        m_chunkOffsets.push_back(pointOffset + sizeof(uint64_t));
        for (uint32_t chunkSize : chunks)
            m_chunkOffsets.push_back(m_chunkOffsets.back() + chunkSize);

        // Clear EOF
        m_stream.clear();
        resetDecompressor();
        m_stream.seekg(m_chunkOffsets[0]);
        m_fileStream.reset();
    }

    
    bool seek(int64_t record)
    {
        if (record < 0)
            return false;

        std::vector<char> buf(baseCount(m_format) + m_ebCount);
        int64_t chunk = record / m_vlr.chunk_size;
        int64_t offset = record % m_vlr.chunk_size;

        m_stream.seekg(m_chunkOffsets[chunk]);
        m_fileStream.reset();
        while (offset > 0)
        {
            decompress(buf.data());
            offset--;
        }
        return true;
    }

    void decompress(char *outbuf)
    {
        if (m_chunkPointsRead == m_vlr.chunk_size)
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
        m_decompressor = lazperf::build_las_decompressor(m_fileStream.cb(), m_format, m_ebCount);
    }

    std::istream& m_stream;
    lazperf::InFileStream m_fileStream;
    lazperf::las_decompressor::ptr m_decompressor;
    int m_format;
    int m_ebCount;
    uint32_t m_chunkPointsRead;
    lazperf::laz_vlr m_vlr;
    // Note that these offsets are actual file offsets. The values stored in the chunk table
    // are chunk sizes.
    std::vector<uint64_t> m_chunkOffsets;
};

LazPerfVlrDecompressor::LazPerfVlrDecompressor(std::istream& stream, int format,
        int ebCount, std::streamoff pointOffset, const char *vlrdata) :
    m_impl(new LazPerfVlrDecompressorImpl(stream, format, ebCount, pointOffset, vlrdata))
{}


LazPerfVlrDecompressor::~LazPerfVlrDecompressor()
{}


void LazPerfVlrDecompressor::decompress(char *outbuf)
{
    m_impl->decompress(outbuf);
}

bool LazPerfVlrDecompressor::seek(int64_t record)
{
    return m_impl->seek(record);
}

} // namespace pdal

