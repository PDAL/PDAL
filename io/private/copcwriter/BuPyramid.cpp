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

#include <iomanip>
#include <set>
#include <string>
#include <vector>

#include <pdal/util/FileUtils.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "BuPyramid.hpp"
#include "OctantInfo.hpp"

namespace pdal
{
namespace copcwriter
{

namespace
{

void createDirs(const BaseInfo& b)
{
    //ABELL - For streaming.
    /**
    if (FileUtils::fileExists(b.tempDir) && !FileUtils::isDirectory(b.tempDir))
        fatal("Can't use temp directory - exists as a regular or special file.");
    if (b.cleanTempDir)
        FileUtils::deleteDirectory(b.tempDir);
    if (!FileUtils::createDirectory(b.tempDir))
        fatal("Couldn't create temp directory: '" + b.tempDir + "'.");
    **/
}

}

/// BuPyramid

BuPyramid::BuPyramid(BaseInfo& common) : m_b(common), m_manager(m_b)
{}


void BuPyramid::run(CellManager& cells)
{
    createDirs(m_b);
    queueWork(cells);
    std::thread runner(&PyramidManager::run, &m_manager);
    runner.join();
}


size_t BuPyramid::queueWork(CellManager& cells)
{
    std::set<VoxelKey> needed;
    std::set<VoxelKey> parentsToProcess;
    std::vector<OctantInfo> have;
    const VoxelKey root;

    for (auto& kv : cells)
    {
        VoxelKey k = kv.first;
        PointViewPtr v = kv.second;

        // Stick an OctantInfo for this cell in the 'have' list.
        OctantInfo o(k);
        o.source() = v;
        have.push_back(o);

        // Walk up the tree and make sure that we're populated for all children necessary
        // to process to the top level.  We do this in order to facilitate processing --
        // a parent node is ready to be processed when all its children have been processed,
        // so making sure a parent has all 8 children makes this easy.
        while (k != root)
        {
            k = k.parent();
            parentsToProcess.insert(k);
            for (int i = 0; i < 8; ++i)
                needed.insert(k.child(i));
        }
    }

    // Now remove entries for the cells we have and their parents.
    for (const OctantInfo& o : have)
    {
        VoxelKey k = o.key();
        while (k != root)
        {
            needed.erase(k);
            k = k.parent();
        }
    }

    // Queue what we have.
    for (const OctantInfo& o : have)
        m_manager.queue(o);

    // Queue what we need but have no data for.
    for (const VoxelKey& k : needed)
        m_manager.queue(OctantInfo(k));
    return parentsToProcess.size();
}

} // namespace copcwriter
} // namespace pdal
