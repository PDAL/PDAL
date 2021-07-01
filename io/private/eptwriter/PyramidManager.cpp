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
namespace ept
{

//ABELL
//PyramidManager::PyramidManager(const BaseInfo& b) : m_b(b), m_pool(10), m_totalPoints(0)
PyramidManager::PyramidManager(const BaseInfo& b) : m_b(b), m_pool(1), m_totalPoints(0)
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
    createHierarchy();
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


void PyramidManager::logOctant(const VoxelKey& k, int cnt)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_written.insert({k, cnt});
    m_totalPoints += cnt;
}


void PyramidManager::createHierarchy()
{
    std::function<int(const VoxelKey&)> calcCounts;
    calcCounts = [this, &calcCounts](const VoxelKey& k)
    {
        int count = 0;
        for (int i = 0; i < 8; ++i)
        {
            VoxelKey c = k.child(i);
            if (m_written.find(c) != m_written.end())
                count += calcCounts(c);
        }
        m_childCounts[k] = count;
        return count + 1;
    };

    calcCounts(VoxelKey(0, 0, 0, 0));

    std::deque<VoxelKey> roots;

    roots.push_back(VoxelKey(0, 0, 0, 0));
    while (roots.size())
    {
        VoxelKey k = roots.front();
        roots.pop_front();
        auto newRoots = emitRoot(k);
        roots.insert(roots.end(), newRoots.begin(), newRoots.end());
    }
}

std::deque<VoxelKey> PyramidManager::emitRoot(const VoxelKey& root)
{
    int level = root.level();
    int stopLevel = level + LevelBreak;

    Entries entries;
    entries.push_back({root, m_written[root]});
    std::deque<VoxelKey> roots = emit(root, stopLevel, entries);

    std::ofstream out(m_b.outputDir + "/ept-hierarchy/" + root.toString() + ".json");

    out << "{\n";

    for (auto it = entries.begin(); it != entries.end(); ++it)
    {
        if (it != entries.begin())
            out << ",\n";
        out << "\"" << it->first << "\": " << it->second;
    }
    out << "\n";

    out << "}\n";

    return roots;
}


std::deque<VoxelKey> PyramidManager::emit(const VoxelKey& p, int stopLevel, Entries& entries)
{
    std::deque<VoxelKey> roots;

    for (int i = 0; i < 8; ++i)
    {
        VoxelKey c = p.child(i);
        auto ci = m_childCounts.find(c);
        if (ci != m_childCounts.end())
        {

            if (c.level() != stopLevel || ci->second <= MinHierarchySize)
            {
                entries.push_back({c, m_written[c]});
                auto r = emit(c, stopLevel, entries);
                roots.insert(roots.end(), r.begin(), r.end());
            }
            else
            {
                entries.push_back({c, -1});
                roots.push_back(c);
            }
        }
    }
    return roots;
}


/**
Stats *PyramidManager::stats(const std::string& name)
{
    auto si = m_stats.find(name);
    if (si == m_stats.end())
        return nullptr;
    return &si->second;
}
**/

} // namespace ept
} // namespace pdal
