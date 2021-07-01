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

#include "GridKey.hpp"
#include "Processor.hpp"
#include "PyramidManager.hpp"

namespace pdal
{
namespace ept
{

static const int MinimumPoints = 100;
static const int MinimumTotalPoints = 1500;

Processor::Processor(PyramidManager& manager, const VoxelInfo& v, const BaseInfo& b) :
    m_vi(v), m_b(b), m_manager(manager)
{}


void Processor::run()
{
    m_vi.initParentOctant();

/**
    size_t totalPoints = 0;
    for (int i = 0; i < 8; ++i)
    {
        OctantInfo& child = m_vi.child(i);

        totalPoints += child.numPoints();
        if (child.numPoints() < MinimumPoints)
            m_vi.octant().appendSource(child.source());
    }
    // It's possible that all the file infos have been moved above, but this is cheap.
    if (totalPoints < MinimumTotalPoints)
        for (int i = 0; i < 8; ++i)
        {
            OctantInfo& child = m_vi.child(i);
            m_vi.octant().appendSource(child.source());
        }
**/

    sample();
    m_manager.queue(m_vi.octant());
}


void Processor::sample()
{
    // Accepted points are those that will go in this (the parent) cell.
    // Rejected points will remain in the child cell they were in previously.

    std::random_device rd;
    std::mt19937 g(rd());

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

        std::shuffle(v->begin(), v->end(), g);
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
    std::string filename = m_b.outputDir + "/ept-data/" + k.toString() + ".laz";
    try
    {
        writeCompressed(filename, v);
    }
    catch (pdal::pdal_error& err)
    {
        throw pdal_error(err.what());
    }
    assert(v->size() <= (std::numeric_limits<int>::max)());
    m_manager.logOctant(k, (int)v->size());
}


void Processor::writeCompressed(const std::string& filename, PointViewPtr v)
{

    StageFactory factory;

    BufferReader r;
    r.addView(v);

    Stage *prev = &r;

    if (v->layout()->hasDim(Dimension::Id::GpsTime))
    {
        Stage *f = factory.createStage("filters.sort");
        pdal::Options fopts;
        fopts.add("dimension", "gpstime");
        f->setOptions(fopts);
        f->setInput(*prev);
        prev = f;
    }

    Stage *w = factory.createStage("writers.las");
    pdal::Options wopts;
    wopts.add("extra_dims", "all");
    wopts.add("software_id", "PDAL EPT Writer");
    wopts.add("compression", "laszip");
    wopts.add("filename", filename);
    wopts.add("offset_x", m_b.offset[0]);
    wopts.add("offset_y", m_b.offset[1]);
    wopts.add("offset_z", m_b.offset[2]);
    wopts.add("scale_x", m_b.scale[0]);
    wopts.add("scale_y", m_b.scale[1]);
    wopts.add("scale_z", m_b.scale[2]);
    w->setOptions(wopts);
    w->setInput(*prev);
    // Set dataformat ID based on time/rgb, but for now accept the default.

    w->prepare(v->table());
    w->execute(v->table());
}

} // namespace ept
} // namespace pdal
