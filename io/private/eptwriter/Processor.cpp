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

#include "../untwine/GridKey.hpp"

#include <pdal/StageFactory.hpp>
#include <pdal/io/BufferReader.hpp>

#include "Processor.hpp"
#include "PyramidManager.hpp"

namespace untwine
{
namespace bu
{

static const int MinimumPoints = 100;
static const int MinimumTotalPoints = 1500;

Processor::Processor(PyramidManager& manager, const VoxelInfo& v, const BaseInfo& b) :
    m_vi(v), m_b(b), m_manager(manager)
{}


void Processor::run()
{
    // Accepted points are those that will go in this (the parent) cell.
    // Rejected points will remain in the child cell they were in previously.
    PointViewPtr accepted(new PointView(m_b.table());
    PointViewPtr rejected(new PointView(m_b.table());

    //ABELL - This is an error that should be better handled when the threads are
    //  joined by the manager.
    if (!accepted || !rejected)
    {
        std::cerr << "Error creating temporary point source."
        m_manager.queue(OctantInfo());
        return;
    }

    size_t totalPoints = 0;
    for (int i = 0; i < 8; ++i)
    {
        OctantInfo& child = m_vi[i];

        totalPoints += child.numPoints();
        if (child.numPoints() < MinimumPoints)
            m_vi.octant().appendSource(child);
    }
    // It's possible that all the file infos have been moved above, but this is cheap.
    if (totalPoints < MinimumTotalPoints)
        for (int i = 0; i < 8; ++i)
            m_vi.octant().appendSource(m_vi[i]);

    Index accepted;
    Index rejected;

    // If the sources haven't all been hoisted, sample.
    if (m_vi.octant().numSources() != totalFileInfos)
        sample(accepted, rejected);

    write(accepted, rejected);

    m_manager.queue(m_vi.octant());
}


void Processor::sample(PointViewPtr accepted, PointViewPtr rejected)
{
    int totalPoints = 0;
    PointViewPtr all;

    // Put all the child sources in one point view.
    for (int i = 0; i < 8; ++i)
    {
        OctantInfo& child = m_vi[i];
        for (PointViewPtr& v : child.sources())
        {
            if (!all)
                all = v.makeNew();
            all->append(*v);
        }
    }

    std::random_device rd;
    std::mt19937 g(rd());

    // Random shuffle the point view containing all potential points to select from
    // then in a random way.
    //ABELL - This may not be the best way to do this. Probably better to work from some
    //  point (center, whatever) out, but this is cheap because you don't have to do
    //  any computation with the point data. And I would think you should still get good
    //  output, but it may be more sparse. Seems you could fix that by just choosing a
    //  smaller radius.  Should be tested.
    std::shuffle(all->begin(), all->end(), g);

    for (PointRef p : *all)
    {
        GridKey k = m_vi.gridKey(p);

        // If we're accepting this point into this voxel from it's child, add it
        // to the accepted list and also stick it in the grid.
        if (acceptable(k))
        {
            accepted.append(p);
            m_vi.grid().insert(k);
        }
        else
            rejected.push_back(i);
    }
}


void Processor::write(Index& accepted, Index& rejected)
{
/**
std::cerr << m_vi.key() << " Accepted/Rejected/num points = " <<
    accepted.size() << "/" << rejected.size() << "/" << m_vi.numPoints() << "!\n";
**/

    // If this is the final key, append any remaining file infos as accepted points and
    // write the accepted points as compressed.
    if (m_vi.key() == VoxelKey(0, 0, 0, 0))
    {
        appendRemainder(accepted);
        writeOctantCompressed(m_vi.octant(), accepted, accepted.begin());
    }
    else
        writeBinOutput(accepted);
    writeCompressedOutput(rejected);
}


bool Processor::acceptable(GridKey key)
{
    VoxelInfo::Grid& grid = m_vi.grid();

    auto it = grid.find(key);

    // If the cell is already occupied the point is not acceptable.
    if (it != grid.end())
        return false;
    return true;
/**
    //ABELL - Needs some point reference argument.
    // We place points in a voxel grid to reduce the number of tests necessary
    // to determine if a new point can be placed without being too close.
    // We size the voxels such that the diagonal is the length of our radius.
    // This way we KNOW that once a cell is occupied, no other point can
    // be placed there. This means the edge length of the voxel cell is
    // radius / √3 = .577 * radius. So, when trying to place a point on the far
    // right side of a cell, it's possible that there's another point already in
    // a cell 2 cells to the right that's only radius * .577 + ε away.

    // ABELL - This should probably be moved to a Grid class.

    // Ignore cells outside of the area of interest.
    int i0 = std::max(key.i() - 2, 0);
    int j0 = std::max(key.j() - 2, 0);
    int k0 = std::max(key.k() - 2, 0);
    int i1 = std::min(key.i() + 2, m_vi.gridXCount());
    int j1 = std::min(key.j() + 2, m_vi.gridYCount());
    int k1 = std::min(key.k() + 2, m_vi.gridZCount());

    for (int i = i0; i <= i1; ++i)
    for (int j = j0; j <= j1; ++j)
    for (int k = k0; k <= k1; ++k)
    {
        //ABELL - Is it worth skipping key location itself or the corner cells?
        auto gi = grid.find(GridKey(i, j, k));
        if (gi != grid.end() && tooClose(pointId, gi->second))
            return false;
    }
    return true;
**/
}


bool Processor::tooClose(pdal::PointId id1, pdal::PointId id2)
{
    const Point& p1 = m_points[id1];
    const Point& p2 = m_points[id2];

    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    double dz = p1.z() - p2.z();

    return dx * dx + dy * dy + dz * dz <= m_vi.squareSpacing();
}


void Processor::writeBinOutput(Index& index)
{
    if (index.empty())
        return;

    // Write the accepted points in binary format. Create a FileInfo to describe the
    // file and it to the octant representing this voxel as it bubbles up.
    // Note that we write the the input directory, as this will be input to a later
    // pass.
    std::string filename = m_vi.key().toString() + ".bin";
    std::string fullFilename = m_b.inputDir + "/" + filename;
    std::ofstream out(fullFilename, std::ios::binary | std::ios::trunc);
    if (!out)
        fatal("Couldn't open '" + fullFilename + "' for output.");
    for (size_t i = 0; i < index.size(); ++i)
        out.write(m_points[index[i]].cdata(), m_b.pointSize);
    m_vi.octant().appendFileInfo(FileInfo(filename, index.size()));
}


// This is a bit confusing.  When we get to the last node, we have two sets of points that
// need to get written to the final (0-0-0-0) node. Those include the points accepted from
// sampling as well as any points that were simply hoisted here due to small size.
//
// We read the hoisted points to stick them on the PointAccessor then we number the points
// by adding them to the index (accepted list).  Then we move the FileInfos from the
void Processor::appendRemainder(Index& index)
{
    std::sort(index.begin(), index.end());

    // Save the current size;
    size_t offset = m_points.size();

    // Read the points from the remaining FileInfos.
    for (FileInfo& fi : m_vi.octant().fileInfos())
        m_points.read(fi);
    size_t numRead = m_points.size() - offset;
    size_t origIndexSize = index.size();

    // Resize the index to contain the read points.
    index.resize(origIndexSize + numRead);

    // The points in the remaining hoisted FileInfos are just numbered sequentially.
    std::iota(index.begin() + origIndexSize, index.end(), offset);

    // NOTE: We need to maintain the order of the file infos as they were read, which
    //   is why they're prepended in reverse.
    // NOTE: The FileInfo pointers in the PointAccessor should remain valid as the
    //   file infos are spliced from the child octant lists onto the parent list.
    for (int i = 8; i > 0; i--)
    {
        FileInfoList& fil = m_vi[i - 1].fileInfos();
        for (auto fi = fil.rbegin(); fi != fil.rend(); ++fi)
            m_vi.octant().prependFileInfo(*fi);
    }
}


void Processor::writeCompressedOutput(Index& index)
{
    // By sorting the rejected points, they will be ordered to match the FileInfo items --
    // meaning that all points that belong in one file will be consecutive.
    std::sort(index.begin(), index.end());

    IndexIter pos = index.begin();

    // If any of our octants has points, we have to write the parent octant, whether or not
    // it contains points, in order to create a full tree.
    for (int octant = 0; octant < 8; ++octant)
        if (m_vi[octant].numPoints() || m_vi[octant].mustWrite())
        {
            m_vi.octant().setMustWrite(true);
            pos = writeOctantCompressed(m_vi[octant], index, pos);
        }
}


void Processor::writeOctantCompressed(const OctantInfo& o)
{
    try
    {
        flushCompressed(o);
    }
    catch (pdal::pdal_error& err)
    {
        fatal(err.what());
    }
}


void Processor::flushCompressed(const OctantInfo& oi)
{
    std::string filename = m_b.outputDir + "/ept-data/" + oi.key().toString() + ".laz";

    StageFactory factory;

    PointViewPtr v = oi.source();

    // If there are no points to write, just make an empty point view.
    if (!v)
        v.reset(new PointView(m_b.table());
    BufferReader r;
    r.addView(*v);

    Stage *prev = &r;

    if (view->layout()->hasDim(Dimension::Id::GpsTime))
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

    w->prepare(m_b.table());
    w->execute(m_b.table());
}

} // namespace bu
} // namespace untwine
