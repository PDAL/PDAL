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


#include <numeric>
#include <random>

#include <pdal/StageFactory.hpp>
#include <io/BufferReader.hpp>

#include <lazperf/writers.hpp>

#include "GridKey.hpp"
#include "Processor.hpp"
#include "PyramidManager.hpp"

namespace pdal
{
namespace copcwriter
{

Processor::Processor(PyramidManager& manager, const VoxelInfo& v, const BaseInfo& b) :
    m_vi(v), b(b), m_manager(manager)
{}


void Processor::run()
{
    m_loader.init(b.pointFormatId, b.scaling, b.extraDims);
    m_vi.initParentOctant();

    size_t totalPoints = 0;
    for (int i = 0; i < 8; ++i)
    {
        OctantInfo& child = m_vi.child(i);

        totalPoints += child.numPoints();
        if (child.numPoints() < MinimumPoints)
            m_vi.octant().movePoints(child);
    }
    // It's possible that all the file infos have been moved above, but this is cheap.
    if (totalPoints < MinimumTotalPoints)
        for (int i = 0; i < 8; ++i)
        {
            OctantInfo& child = m_vi.child(i);
            m_vi.octant().movePoints(child);
        }

    sample();
    write();
    m_manager.queue(m_vi.octant());
}

void Processor::write()
{
    OctantInfo& parent = m_vi.octant();

    for (int i = 0; i < 8; ++i)
    {
        OctantInfo& child = m_vi.child(i);
        PointViewPtr& v = child.source();
        if (v->size() || child.mustWrite())
        {
            writeCompressed(child.key(), child.source());
            parent.setMustWrite(true);
        }
    }
    if (m_vi.key() == VoxelKey(0, 0, 0, 0))
        writeCompressed(parent.key(), parent.source());
}


void Processor::sample()
{
    // Accepted points are those that will go in this (the parent) cell.
    // Rejected points will remain in the child cell they were in previously.

    std::unique_ptr<std::mt19937> g;
    if (b.opts.fixedSeed)
    {
        std::vector<int32_t> v{1234};
        std::seed_seq seed(v.begin(), v.end());
        g.reset(new std::mt19937(seed));
    }
    else
    {
        std::random_device rd;
        g.reset(new std::mt19937(rd()));
    }

    OctantInfo& parent = m_vi.octant();
    PointViewPtr accepted = parent.source();

    for (int i = 0; i < 8; ++i)
    {
        OctantInfo& child = m_vi.child(i);
        if (child.numPoints() == 0)
            continue;

        PointViewPtr& v = child.source();
        PointViewPtr rejected = v->makeNew();

        std::shuffle(v->begin(), v->end(), *g);
        for (PointId idx = 0; idx < v->size(); ++idx)
        {
            GridKey k = m_vi.gridKey(PointRef(*v, idx));

            // If we're accepting this point into this voxel from it's child, add it
            // to the accepted list and also stick it in the grid.
            if (acceptable(k))
            {
                accepted->appendPoint(*v, idx);
                m_vi.grid().insert(k);
            }
            else
                rejected->appendPoint(*v, idx);
        }
        v = rejected;
    }
}


bool Processor::acceptable(GridKey key)
{
    VoxelInfo::Grid& grid = m_vi.grid();

    auto it = grid.find(key);

    // If the cell is already occupied the point is not acceptable.
    if (it != grid.end())
        return false;
    return true;
}


void Processor::writeCompressed(VoxelKey k, PointViewPtr v)
{
    if (v->size() == 0)
    {
        m_manager.newChunk(k, 0, 0);
        return;
    }

    PointLayoutPtr layout = v->layout();

    std::vector<char> buf(lazperf::baseCount(b.pointFormatId) + b.numExtraBytes);
    lazperf::writer::chunk_compressor compressor(b.pointFormatId, b.numExtraBytes);

    if (b.sortDims.size())
    {
        // Sort based on the dimensions and ordering we were given
        for (Dimension::Id dimId: b.sortDims)
        {
            std::sort(v->begin(), v->end(), [dimId](const PointRef& p1, const PointRef& p2)
                { return p1.compare(dimId, p2); });
        }
    } else
    {
        // Sort by GPS time - no-op if there's no GPS time.
        std::sort(v->begin(), v->end(), [](const PointRef& p1, const PointRef& p2)
            { return p1.compare(Dimension::Id::GpsTime, p2); });

    }

    for (PointId idx = 0; idx < v->size(); ++idx)
    {
        PointRef point(*v, idx);
        m_loader.pack(point, buf.data(), buf.size());
        compressor.compress(buf.data());
    }
    std::vector<unsigned char> chunk = compressor.done();
    uint64_t location = m_manager.newChunk(k, (uint32_t)chunk.size(), (uint32_t)v->size());

    std::ofstream out(b.filename, std::ios::out | std::ios::in | std::ios::binary);
    out.seekp(std::ofstream::pos_type(location));
    out.write(reinterpret_cast<const char *>(chunk.data()), chunk.size());
    out.close();
    if (!out)
        throw pdal_error("Failure writing to '" + b.filename + "'.");
}

} // namespace copcwriter
} // namespace pdal
