/******************************************************************************
 * Copyright (c) 2022, Hobu Inc.
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
 *     * Neither the name of Hobu, Inc. nor the
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

#include <istream>
#include <pdal/util/IStream.hpp>
#include <lazperf/vlr.hpp>
#include "Header.hpp"

namespace pdal
{
namespace las
{

class ChunkInfo
{
public:
    void load(std::istream& stream, uint64_t pointOffset, uint64_t numPoints, uint32_t chunkSize)
    {
        m_chunks.clear();
        m_numPoints = numPoints;

        stream.seekg(pointOffset);
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

        bool variable = (chunkSize == lazperf::VariableChunkSize);

        if (numChunks)
            m_chunks = lazperf::decompress_chunk_table(stream, numChunks, variable);

        // If the chunk size is fixed, set the counts to the chunk size since
        // they aren't stored in the chunk table.
        if (!variable)
        {
            uint64_t remaining = m_numPoints;
            for (lazperf::chunk& chunk : m_chunks)
            {
                chunk.count = (std::min)((uint64_t)chunkSize, remaining);
                remaining -= chunk.count;
            }
            assert(remaining == 0);
        }

        // Add a chunk at the beginning that has a count of 0 and an offset of the
        // start of the first chunk.
        m_chunks.insert(m_chunks.begin(), {0, pointOffset + sizeof(uint64_t)});

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
    }

    // Find the # of the chunk containing a point. -1 if no such chunk exists.
    int32_t chunk(uint64_t point) const
    {
        if (point >= m_numPoints || m_chunks.empty())
            return -1;

        // Search for the chunk containing the requested point.
        // The chunk count is used, but it contains the number of points in all the *previous*
        // chunks, so...
        auto ci = std::upper_bound(m_chunks.begin(), m_chunks.end(), point,
            [](uint64_t point, const lazperf::chunk& c) {  return point < c.count; });

        // ...we have to back up one to find the proper chunk containing the record.
        if (ci == m_chunks.begin()) // Should never happen.
            return -1;
        ci--;
        size_t distance = std::distance(m_chunks.begin(), ci);
        return static_cast<int32_t>(distance);
    }

    // Find the offset of a point in a chunk. -1 if the offset isn't in the provided chunk.
    int32_t index(uint64_t point, int32_t chunk) const
    {
        if (point > m_chunks[chunk + 1].count || point < m_chunks[chunk].count)
            return -1L;
        return static_cast<int32_t>(point - m_chunks[chunk].count);
    }

    // Get the number of points in the chunk.
    uint32_t chunkPoints(int32_t chunk)
    {
        size_t sum = m_chunks[chunk + 1].count - m_chunks[chunk].count;
        return static_cast<uint32_t>(sum);
    }

    // Get the ID of the first point in the chunk.
    uint32_t firstPoint(int32_t chunk)
    {
        return static_cast<uint32_t>(m_chunks[chunk].count);
    }

    // Get the position of the chunk in the file.
    uint64_t chunkOffset(int32_t chunk)
    {
        return m_chunks[chunk].offset;
    }

    // Get the size of the chunk.
    uint32_t chunkSize(int32_t chunk)
    {
        return (uint32_t)(m_chunks[chunk + 1].offset - m_chunks[chunk].offset);
    }

    size_t numChunks() const
    {
        return m_chunks.size() - 1;
    }

private:
    std::vector<lazperf::chunk> m_chunks;
    uint64_t m_numPoints;
};

} // namespace las
} // namespace pdal

