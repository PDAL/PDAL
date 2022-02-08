/*****************************************************************************
 *   Copyright (c) 2020, Hobu, Inc. (info@hobu.co)                           *
 *                                                                           *
 *   All rights reserved.                                                    *
 *                                                                           *
 *   This program is free software; you can redistribute it and/or modify    *
 *   it under the terms of the GNU General Public License as published by    *
 *   the Free Software Foundation; either version 3 of the License, or       *
 *   (at your option) any later version.                                     *
 *                                                                           *
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
    m_manager.queue(m_vi.octant());
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
        if (child.mustWrite())
            parent.setMustWrite(true);
        if (!child.numPoints())
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
        if (child.mustWrite() || rejected->size())
            writeCompressed(child.key(), rejected);

        // Ditch the point view (to save memory?)
        v.reset();
    }
    if (m_vi.key() == VoxelKey(0, 0, 0, 0))
        writeCompressed(parent.key(), accepted);
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
    for (PointId idx = 0; idx < v->size(); ++idx)
    {
        PointRef point(*v, idx);
        m_loader.pack(point, buf.data(), buf.size());
        compressor.compress(buf.data());
    }
    std::vector<unsigned char> chunk = compressor.done();
    uint64_t location = m_manager.newChunk(k, (uint32_t)chunk.size(), (uint32_t)v->size());

    std::ofstream out(b.opts.filename, std::ios::out | std::ios::in | std::ios::binary);
    out.seekp(std::ofstream::pos_type(location));
    out.write(reinterpret_cast<const char *>(chunk.data()), chunk.size());
    out.close();
    if (!out)
        throw pdal_error("Failure writing to '" + b.opts.filename + "'.");
}

} // namespace copcwriter
} // namespace pdal
