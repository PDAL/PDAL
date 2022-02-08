/*****************************************************************************
 *   Copyright (c) 2021, Hobu, Inc. (info@hobu.co)                           *
 *                                                                           *
 *   All rights reserved.                                                    *
 *                                                                           *
 *   This program is free software; you can redistribute it and/or modify    *
 *   it under the terms of the GNU General Public License as published by    *
 *   the Free Software Foundation; either version 3 of the License, or       *
 *   (at your option) any later version.                                     *
 *                                                                           *
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
    struct Hierarchy
    {
        uint64_t offset;
        int32_t byteSize;
        int32_t pointCount;
    };

    using CountMap = std::unordered_map<VoxelKey, point_count_t>;
    using Entries = std::vector<std::pair<VoxelKey, Hierarchy>>;

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
    std::unordered_map<VoxelKey, Hierarchy> m_hierarchy;

    void writeHeader();
    void writeVlrData();
    void writeChunkTable();
    void writeHierarchy(const CountMap& counts);
    Hierarchy emitRoot(const VoxelKey& root, const CountMap& counts);
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
