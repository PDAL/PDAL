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

#ifdef _MSC_VER
#pragma warning (push)
#pragma warning (disable: 4251)
#endif

#include <lazperf/lazperf.hpp>
#include <lazperf/filestream.hpp>
#include <lazperf/vlr.hpp>

#ifdef _MSC_VER
#pragma warning (pop)
#endif

// This only exist in version 1.3+, so is an acceptable version test for now.
#ifndef LAZPERF_VERSION
#error "LAZperf version 2+ (supporting LAS version 1.4) not found"
#endif

#include <pdal/util/IStream.hpp>
#include <pdal/util/OStream.hpp>
#include <pdal/pdal_types.hpp>
#include <io/private/las/Header.hpp>

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
        if (m_compressor)
        {
            m_compressor->done();
            newChunk();
        }

        // If we didn't write any points, chunk info pos will be 0 and we need to
        // set the chunk info pos. Could do this as an "else" case of the
        // above, but this seems safer in case some other compressor creation logic
        // comes about.
        if (m_chunkInfoPos == 0)
        {
            m_chunkInfoPos = m_stream.tellp();
            m_stream.seekp(sizeof(uint64_t), std::ios::cur);
        }

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
    using ChunkList = std::vector<lazperf::chunk>;
    using ChunkIter = std::vector<lazperf::chunk>::iterator;

public:
    LazPerfVlrDecompressorImpl(std::istream& stream, const las::Header& header,
            const char *vlrdata) :
        m_stream(stream), m_fileStream(stream), m_format(header.pointFormat()),
        m_pointLen(header.pointSize), m_ebCount(header.ebCount()),
        m_pointCount(header.pointCount()), m_vlr(vlrdata), m_chunkPointsTotal(0),
        m_chunkPointsRead(0), m_curChunk(m_chunks.end())
    {
        m_stream.seekg(header.pointOffset);
        ILeStream in(&stream);

        uint64_t chunkTablePos;
        in >> chunkTablePos;

        in.seek(chunkTablePos);
        uint32_t version;
        uint32_t numChunks;
        in >> version;
        in >> numChunks;

        if (version != 0)
            throw pdal_error("Invalid version " + std::to_string(version) +
                " found in LAZ chunk table.");

        bool variable = (m_vlr.chunk_size == lazperf::VariableChunkSize);

        if (numChunks)
            m_chunks = lazperf::decompress_chunk_table(m_fileStream.cb(), numChunks, variable);

        // If the chunk size is fixed, set the counts to the chunk size since
        // they aren't stored in the chunk table..
        if (!variable)
        {
            uint64_t remaining = m_pointCount;
            for (lazperf::chunk& chunk : m_chunks)
            {
                chunk.count = (std::min)((uint64_t)m_vlr.chunk_size, remaining);
                remaining -= chunk.count;
            }
            assert(remaining == 0);
        }

        // Add a chunk at the beginning that has a count of 0 and an offset of the
        // start of the first chunk.
        m_chunks.insert(m_chunks.begin(), {0, header.pointOffset + sizeof(uint64_t)});

        // Fix up the chunk table such that the offsets are absolute offsets to the
        // chunk and the counts are cumulative counts of points before the chunk.
        // When we're done, the chunk table looks like this, where N is the number of chunks:

        // Chunk table entry 1: offset to chunk 1, count of 0
        // Chunk table entry 2: offset to chunk 2, count of chunk 1
        // Chunk table entry 3: offset to chunk 3, count of chunk 1 + 2
        // ...
        // Chunk table entry N: offset to chunk N, count of chunk 1 + ... + N
        // Chunk table entry N + 1: offset to end of chunks (start of chunk table),
        //   count is the total number of points in all chunks.

        for (size_t i = 1; i < m_chunks.size(); ++i)
        {
            m_chunks[i].offset += m_chunks[i - 1].offset;
            m_chunks[i].count += m_chunks[i - 1].count;
        }

        // Clear EOF
        m_stream.clear();
        resetDecompressor();
        setChunk(m_chunks.begin());
        m_stream.seekg(m_curChunk->offset);
        m_fileStream.reset();
    }


    bool seek(uint64_t record)
    {
        if (record >= m_pointCount || m_chunks.empty())
            return false;

        // Search for the chunk containing the requested record.
        auto ci = std::upper_bound(m_chunks.begin(), m_chunks.end(), record,
            [](uint64_t record, const lazperf::chunk& c) {  return record < c.count; });

        if (ci == m_chunks.begin()) // Should never happen.
            return false;

        ci--;

        // Calculate the number of points we need to skip in the located chunk.
        setChunk(ci);
        uint64_t toRead = record - ci->count;
        m_stream.seekg(ci->offset);
        m_fileStream.reset();
        std::vector<char> buf(m_pointLen);
        while (toRead--)
            if (!decompress(buf.data()))
                return false;
        return true;
    }

    bool decompress(char *outbuf)
    {
        if (chunkDone())
        {
            if (!nextChunk())
                return false;
            resetDecompressor();
        }
        m_decompressor->decompress(outbuf);
        m_chunkPointsRead++;
        return true;
    }

private:
    void resetDecompressor()
    {
        m_decompressor = lazperf::build_las_decompressor(m_fileStream.cb(), m_format, m_ebCount);
    }

    bool nextChunk()
    {
        if (m_curChunk == m_chunks.end())
            return false;
        if (!setChunk(m_curChunk + 1))
            return false;
        m_chunkPointsRead = 0;
        return true;
    }

    bool setChunk(ChunkIter chunk)
    {
        m_curChunk = chunk;
        auto nextChunk = chunk + 1;
        // The chunk table entries are written at the *end* of a creating a chunk. When we
        // read the chunk table, we stick an entry at the front indicating the start of
        // the first chunk. The last chunk table entry points to the end of the chunks
        // and has a count value equal to the total number of points.
        if (chunk == m_chunks.end() || nextChunk == m_chunks.end())
            return false;

        m_chunkPointsTotal = (int)(nextChunk->count - chunk->count);
        return true;
    }

    bool chunkDone() const
        { return m_chunkPointsRead == m_chunkPointsTotal; }

    std::istream& m_stream;
    lazperf::InFileStream m_fileStream;
    lazperf::las_decompressor::ptr m_decompressor;
    int m_format;
    int m_pointLen;
    int m_ebCount;
    uint64_t m_pointCount;
    lazperf::laz_vlr m_vlr;
    int m_chunkPointsTotal;
    int m_chunkPointsRead;

    ChunkList m_chunks;
    ChunkIter m_curChunk;
};

LazPerfVlrDecompressor::LazPerfVlrDecompressor(std::istream& stream,
        const las::Header& header, const char *vlrdata) :
    m_impl(new LazPerfVlrDecompressorImpl(stream, header, vlrdata))
{}


LazPerfVlrDecompressor::~LazPerfVlrDecompressor()
{}


bool LazPerfVlrDecompressor::decompress(char *outbuf)
{
    return m_impl->decompress(outbuf);
}

bool LazPerfVlrDecompressor::seek(uint64_t record)
{
    return m_impl->seek(record);
}

} // namespace pdal

