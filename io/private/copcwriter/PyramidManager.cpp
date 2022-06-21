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

#include <regex>
#include <string>
#include <vector>

#include <pdal/util/FileUtils.hpp>

#include "Processor.hpp"
#include "PyramidManager.hpp"
#include "VoxelInfo.hpp"
#include "VoxelKey.hpp"

namespace pdal
{
namespace copcwriter
{

PyramidManager::PyramidManager(BaseInfo& b) : m_b(b), m_pool(10), m_totalPoints(0),
    m_output(b)
{}


PyramidManager::~PyramidManager()
{}


void PyramidManager::queue(const OctantInfo& o)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push(o);
    }
    m_cv.notify_one();
}


// Initially, all the leaf nodes will be on the queue.
void PyramidManager::run()
{
    while (true)
    {
        OctantInfo o;
        {
            std::unique_lock<std::mutex> lock(m_mutex);

            m_cv.wait(lock, [this](){return m_queue.size();});
            o = m_queue.front();
            m_queue.pop();
        }

        // We're done if we have processed the root key.
        if (o.key() == VoxelKey(0, 0, 0, 0))
            break;
        process(o);
    }
    // Create the EPT hierarchy files from the data.

    // Calculate the cumulative counts for each cell from the individual cell counts.
    Output::CountMap childCounts;

    std::function<int(const VoxelKey&)> calcCounts;
    calcCounts = [this, &childCounts, &calcCounts](const VoxelKey& k)
    {
        int count = 0;
        for (int i = 0; i < 8; ++i)
        {
            VoxelKey c = k.child(i);
            if (m_written.find(c) != m_written.end())
                count += calcCounts(c);
        }
        childCounts[k] = count;
        return count + 1;
    };

    calcCounts(VoxelKey(0, 0, 0, 0));
    m_output.finish(childCounts);
}

// Called when a processor wants to write a chunk in single-file mode. Returns the location
// where the chunk should be written.
uint64_t PyramidManager::newChunk(const VoxelKey& key, uint32_t size, uint32_t count)
{
    static std::mutex m;

    std::unique_lock<std::mutex> lock(m);
    m_written.insert({key, count});
    m_totalPoints += count;
    return m_output.newChunk(key, size, count);
}

// Take the item off the queue and stick it on the complete list. If we have all 8 octants,
// remove the items from the complete list and queue a Processor job.
void PyramidManager::process(const OctantInfo& o)
{
    VoxelKey parentKey = o.key().parent();
    addComplete(o);
    if (!childrenComplete(parentKey))
        return;

    VoxelInfo vi(m_b.bounds, parentKey);
    for (int i = 0; i < 8; ++i)
        vi.child(i) = removeComplete(parentKey.child(i));

    // If there are no points in this voxel, just queue it as a child.
    if (!vi.hasPoints())
        queue(vi.octant());
    else
    {
        m_pool.add([vi, this]()
        {
            Processor p(*this, vi, m_b);
            p.run();
        });
    }
}


void PyramidManager::addComplete(const OctantInfo& o)
{
    m_completes.insert({o.key(), o});
}


bool PyramidManager::childrenComplete(const VoxelKey& parent)
{
    for (int i = 0; i < 8; ++i)
        if (m_completes.find(parent.child(i)) == m_completes.end())
            return false;
    return true;
}


OctantInfo PyramidManager::removeComplete(const VoxelKey& k)
{
    OctantInfo o;

    auto oi = m_completes.find(k);
    if (oi != m_completes.end())
    {
        o = std::move(oi->second);
        m_completes.erase(oi);
    }
    return o;
}

} // namespace copcwriter
} // namespace pdal
