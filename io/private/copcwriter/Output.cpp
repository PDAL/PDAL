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

#include <iostream>
#include <limits>

#include "Output.hpp"

#include <pdal/util/Algorithm.hpp>
#include <pdal/util/OStream.hpp>
#include <pdal/util/Extractor.hpp>

#include <lazperf/filestream.hpp>

#include "Common.hpp"

namespace pdal
{
namespace copcwriter
{

Output::Output(const BaseInfo& b) : b(b)
{
    m_header.file_source_id = b.opts.filesourceId.val();
    m_header.global_encoding = b.opts.globalEncoding.val() | (1 << 4); // Set for WKT
    b.opts.projectId.val().pack(m_header.guid);
    m_header.version.major = 1;
    m_header.version.minor = 4;
    memcpy(m_header.system_identifier, b.opts.systemId.val().data(), 32);
    memcpy(m_header.generating_software, b.opts.softwareId.val().data(), 32);
    m_header.creation.day = b.opts.creationDoy.val();
    m_header.creation.year = b.opts.creationYear.val();
    m_header.header_size = lazperf::header14::Size;
    m_header.point_format_id = b.pointFormatId | (1 << 7);  // Bit for laszip
    m_header.point_record_length = lazperf::baseCount(b.pointFormatId) + b.numExtraBytes;
    //ABELL - Check this.
    m_header.scale.x = b.scaling.m_xXform.m_scale.m_val;
    m_header.scale.y = b.scaling.m_yXform.m_scale.m_val;
    m_header.scale.z = b.scaling.m_zXform.m_scale.m_val;
    m_header.offset.x = b.scaling.m_xXform.m_offset.m_val;
    m_header.offset.y = b.scaling.m_yXform.m_offset.m_val;
    m_header.offset.z = b.scaling.m_zXform.m_offset.m_val;
    m_header.vlr_count = 0;

    m_header.minx = b.stats[(int)stats::Index::X].minimum();
    m_header.maxx = b.stats[(int)stats::Index::X].maximum();
    m_header.miny = b.stats[(int)stats::Index::Y].minimum();
    m_header.maxy = b.stats[(int)stats::Index::Y].maximum();
    m_header.minz = b.stats[(int)stats::Index::Z].minimum();
    m_header.maxz = b.stats[(int)stats::Index::Z].maximum();

    // legacy point counts are all zero, since COPC is LAS 1.4 only.
    std::fill(std::begin(m_header.points_by_return), std::end(m_header.points_by_return), 0);
    m_header.point_count = 0;

    for (int i = 1; i <= 15; ++i)
    {
        point_count_t count = 0;
        try
        {
            count = b.stats[(int)stats::Index::ReturnNumber].values().at(i);
        }
        catch (const std::out_of_range&)
        {}

        m_header.points_by_return_14[i - 1] = count;
    }

    // Note that only these VLRs go in the standard VLR area. The rest are written as
    // extended VLRs.
    setupVlrs();
    m_header.point_offset = lazperf::header14::Size + m_vlrBuf.size();

    // The chunk table offset is written as the first 8 bytes of the point data in LAZ.
    m_chunkOffsetPos = m_header.point_offset;

    // The actual point data comes after the chunk table offset. m_pointPos is updated
    // as points are written.
    m_pointPos = m_chunkOffsetPos + sizeof(uint64_t);

    m_f.open(b.opts.filename, std::ios::out | std::ios::binary);
}

// For this to work properly, the VLRs can't change size.
void Output::setupVlrs()
{
    m_vlrBuf.clear();
    m_header.vlr_count = 0;
    addCopcVlr();
    addLazVlr();
    addWktVlr();
    addEbVlr();
}

void Output::addCopcVlr()
{
    // The copc VLR isn't local because we need to fill in the hierarchy root node
    // offset/size when we know it.
    m_copcVlr.center_x = (b.bounds.maxx / 2) + (b.bounds.minx / 2);
    m_copcVlr.center_y = (b.bounds.maxy / 2) + (b.bounds.miny / 2);
    m_copcVlr.center_z = (b.bounds.maxz / 2) + (b.bounds.minz / 2);
    m_copcVlr.halfsize = (b.bounds.maxx - b.bounds.minx) / 2;
    m_copcVlr.spacing = (2.0 * m_copcVlr.halfsize) / RootCellCount;
    m_copcVlr.gpstime_minimum = b.stats[(int)stats::Index::GpsTime].minimum();
    m_copcVlr.gpstime_maximum = b.stats[(int)stats::Index::GpsTime].maximum();

    std::vector<char> buf = m_copcVlr.header().data();
    m_vlrBuf.insert(m_vlrBuf.end(), buf.begin(), buf.end());

    buf = m_copcVlr.data();
    m_vlrBuf.insert(m_vlrBuf.end(), buf.begin(), buf.end());
    m_header.vlr_count++;
}

void Output::addLazVlr()
{
    lazperf::laz_vlr laz(b.pointFormatId, b.numExtraBytes, lazperf::VariableChunkSize);

    std::vector<char> buf = laz.header().data();
    m_vlrBuf.insert(m_vlrBuf.end(), buf.begin(), buf.end());

    buf = laz.data();
    m_vlrBuf.insert(m_vlrBuf.end(), buf.begin(), buf.end());
    m_header.vlr_count++;
}

void Output::addWktVlr()
{
    if (!b.srs.valid())
        return;

    lazperf::wkt_vlr wkt(b.srs.getWKT1());

    std::vector<char> buf = wkt.header().data();
    m_vlrBuf.insert(m_vlrBuf.end(), buf.begin(), buf.end());

    buf = wkt.data();
    m_vlrBuf.insert(m_vlrBuf.end(), buf.begin(), buf.end());
    m_header.vlr_count++;
}

void Output::addEbVlr()
{
    if (!b.numExtraBytes)
        return;

    lazperf::eb_vlr eb;

    for (const las::ExtraDim& d : b.extraDims)
    {
        lazperf::eb_vlr::ebfield f;
        f.name = d.m_name;
        f.data_type = las::lasType(d.m_dimType.m_type, 1);
        eb.addField(f);
    }

    std::vector<char> buf = eb.header().data();
    m_vlrBuf.insert(m_vlrBuf.end(), buf.begin(), buf.end());

    buf = eb.data();
    m_vlrBuf.insert(m_vlrBuf.end(), buf.begin(), buf.end());
    m_header.vlr_count++;
}


void Output::finish(const std::unordered_map<VoxelKey, point_count_t>& childCounts)
{
    writeChunkTable();
    writeHierarchy(childCounts);
    writeEvlrs();

    // These items are last because we don't have all their data until the end.
    // Note that we call setupVlrs() a second time here to make sure the data describing
    // the hierarchy is properly written.
    writeHeader();
    setupVlrs();
    writeVlrData();
}

/// \param  size  Size of the chunk in bytes
/// \param  count  Number of points in the chunk
/// \param  key  Key of the voxel the chunk represents
/// \return  The offset of the chunk in the file.
uint64_t Output::newChunk(const VoxelKey& key, int32_t size, int32_t count)
{
    // Chunks of size zero are a special case.
    if (count == 0)
    {
        m_hierarchy[key] = { 0, 0, 0 };
        return 0;
    }

    uint64_t chunkStart = m_pointPos;
    m_pointPos += size;
    assert(count <= (std::numeric_limits<int32_t>::max)() && count >= 0);
    m_chunkTable.push_back({(uint64_t)count, (uint64_t)size});
    m_header.point_count_14 += count;
    m_hierarchy[key] = { chunkStart, size, count };
    return chunkStart;
}

void Output::writeHeader()
{
    std::ostream& out = m_f;

    out.seekp(0);
    m_header.write(out);
}

void Output::writeVlrData()
{
    std::ostream& out = m_f;

    out.seekp(m_header.header_size);
    out.write(m_vlrBuf.data(), m_vlrBuf.size());
}

void Output::writeChunkTable()
{
    // Write chunk table offset
    pdal::OLeStream out(&m_f);
    out.seek(m_chunkOffsetPos);
    out << m_pointPos;

    // Write chunk table header.
    out.seek(m_pointPos);
    uint32_t version = 0;
    out << version;
    out << (uint32_t)m_chunkTable.size();

    // Write the chunk table itself.
    lazperf::OutFileStream stream(m_f);
    lazperf::compress_chunk_table(stream.cb(), m_chunkTable, true);

    // The ELVRs start after the chunk table. Add one for the hierarchy VLR.
    m_header.evlr_count = b.vlrs.size() + 1;
    m_header.evlr_offset = out.position();
}

void Output::writeHierarchy(const CountMap& counts)
{
    // Move to the location *after* the EVLR header.
    m_f.seekp(m_header.evlr_offset + lazperf::evlr_header::Size);

    uint64_t beginPos = m_f.tellp();

    Entry root = emitRoot(VoxelKey(0, 0, 0, 0), counts);
    m_copcVlr.root_hier_offset = root.offset;
    m_copcVlr.root_hier_size = root.byteSize;
    uint64_t endPos = m_f.tellp();

    // Now write VLR header.
    lazperf::evlr_header h { 0, "copc", 1000, (endPos - beginPos), "EPT Hierarchy" };
    m_f.seekp(m_header.evlr_offset);
    h.write(m_f);
}

Output::Entry Output::emitRoot(const VoxelKey& root, const CountMap& counts)
{
    const int LevelBreak = 4;
    Entries entries;

    int stopLevel = root.level() + LevelBreak;
    entries.push_back({root, m_hierarchy[root]});
    emitChildren(root, counts, entries, stopLevel);

    pdal::OLeStream out(&m_f);
    uint64_t startPos = out.position();
    for (auto it = entries.begin(); it != entries.end(); ++it)
    {
        VoxelKey& key = it->first;
        Entry& e = it->second;
        out << key.level() << key.x() << key.y() << key.z();
        out << e.offset << e.byteSize << e.pointCount;
    }
    uint64_t endPos = out.position();

    // This is the information about where the hierarchy node was written, to be
    // written with the parent.
    return { startPos, (int32_t)(endPos - startPos), -1 };
}

void Output::emitChildren(const VoxelKey& p, const CountMap& counts,
    Entries& entries, int stopLevel)
{
    const int MinHierarchySize = 50;

    for (int i = 0; i < 8; ++i)
    {
        VoxelKey c = p.child(i);
        auto ci = counts.find(c);
        if (ci != counts.end())
        {
            // If we're not at a stop level or the number of child nodes is less than 50,
            // just stick them here.
            if (c.level() != stopLevel || ci->second <= MinHierarchySize)
            {
                entries.push_back({c, m_hierarchy[c]});
                emitChildren(c, counts, entries, stopLevel);
            }
            else
                entries.push_back({c, emitRoot(c, counts)});
        }
    }
}

void Output::writeEvlrs()
{
    m_f.seekp(0, std::ios_base::end);

    for (const las::Evlr& v : b.vlrs)
    {
        std::vector<char> header = v.headerData();
        m_f.write(header.data(), header.size());
        m_f.write(v.data(), v.dataSize());
    }
}

} // namespace copcwriter
} // namespace pdal

