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

/// BuPyramid

BuPyramid::BuPyramid(BaseInfo& common) : m_manager(common)
{}


void BuPyramid::run(CellManager& cells)
{
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
