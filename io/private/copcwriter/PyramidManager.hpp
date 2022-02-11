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

#pragma once

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <pdal/util/ThreadPool.hpp>

#include "Common.hpp"
#include "OctantInfo.hpp"
#include "Output.hpp"

namespace pdal
{
namespace copcwriter
{

class OctantInfo;
class Processor;

class PyramidManager
{
    using Entries = std::vector<std::pair<VoxelKey, int>>;
public:
    PyramidManager(const BaseInfo& b);
    ~PyramidManager();

    void queue(const OctantInfo& o);
    void run();
    uint64_t newChunk(const VoxelKey& key, uint32_t size, uint32_t count);
    uint64_t totalPoints() const
        { return m_totalPoints; }

private:
    const BaseInfo& m_b;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::unordered_map<VoxelKey, OctantInfo> m_completes;
    std::queue<OctantInfo> m_queue;
    ThreadPool m_pool;
    uint64_t m_totalPoints;
    Output m_output;
    //
    std::unordered_map<VoxelKey, int> m_written;

    void getInputFiles();
    void process(const OctantInfo& o);
    void addComplete(const OctantInfo& o);
    bool childrenComplete(const VoxelKey& parent);
    OctantInfo removeComplete(const VoxelKey& k);
};

} // namespace copcwriter
} // namespace pdal
