/******************************************************************************
* Copyright (c) 2021, Hobu Inc. (info@hobu.co)
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

#include <unordered_map>

#include "Common.hpp"
#include "VoxelKey.hpp"

#include <lazperf/lazperf.hpp>
#include <lazperf/vlr.hpp>

namespace pdal
{
namespace copcwriter
{

struct BaseInfo;

class Output
{
public:
    struct Entry
    {
        uint64_t offset;
        int32_t byteSize;
        int32_t pointCount;
    };

    using CountMap = std::unordered_map<VoxelKey, point_count_t>;
    using Entries = std::vector<std::pair<VoxelKey, Entry>>;

    Output(const BaseInfo& b);
    void finish(const CountMap& childCounts);
    uint64_t newChunk(const VoxelKey& key, int32_t size, int32_t count);

private:
    const BaseInfo& b;
    std::ofstream m_f;
    lazperf::header14 m_header;
    lazperf::copc_info_vlr m_copcVlr;
    std::vector<char> m_vlrBuf;
    std::vector<lazperf::chunk> m_chunkTable;
    uint64_t m_chunkOffsetPos;
    uint64_t m_pointPos;
    std::unordered_map<VoxelKey, Entry> m_hierarchy;

    void writeHeader();
    void writeVlrData();
    void writeChunkTable();
    void writeHierarchy(const CountMap& counts);
    Entry emitRoot(const VoxelKey& root, const CountMap& counts);
    void emitChildren(const VoxelKey& root, const CountMap& counts,
        Entries& entries, int stopLevel);
    void setupVlrs();
    void writeEvlrs();
    void addCopcVlr();
    void addLazVlr();
    void addWktVlr();
    void addEbVlr();
};

} // namesapce copcwriter
} // namesapce pdal
