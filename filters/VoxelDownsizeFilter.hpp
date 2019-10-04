/******************************************************************************
 * Copyright (c) 2019, Helix.re
 * Contact Person : Pravin Shinde (pravin@helix.re, https://github.com/pravinshinde825)
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

#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>
#include <cassert>
#include <leveldb/db.h>
#include <leveldb/write_batch.h>


namespace pdal
{

class PointLayout;
class PointView;
class PDAL_DLL Pool
{
public:
    // After numThreads tasks are actively running, and queueSize tasks have
    // been enqueued to wait for an available worker thread, subsequent calls
    // to Pool::add will block until an enqueued task has been popped from the
    // queue.
    Pool(std::size_t numThreads, std::size_t queueSize = 1, bool verbose = true)
        : m_verbose(verbose),
          m_numThreads(std::max<std::size_t>(numThreads, 1)),
          m_queueSize(std::max<std::size_t>(queueSize, 1))
    {
        go();
    }

    ~Pool()
    {
        join();
    }

    // Start worker threads.
    void go()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_running)
            return;
        m_running = true;

        for (std::size_t i(0); i < m_numThreads; ++i)
        {
            m_threads.emplace_back([this]() { work(); });
        }
    }

    // Disallow the addition of new tasks and wait for all currently running
    // tasks to complete.
    void join()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        if (!m_running)
            return;
        m_running = false;
        lock.unlock();

        m_consumeCv.notify_all();
        for (auto& t : m_threads)
            t.join();
        m_threads.clear();
    }

    // Wait for all current tasks to complete.  As opposed to join, tasks may
    // continue to be added while a thread is await()-ing the queue to empty.
    void await()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_produceCv.wait(
            lock, [this]() { return !m_outstanding && m_tasks.empty(); });
    }

    // Join and restart.
    void cycle()
    {
        join();
        go();
    }

    // Change the number of threads.  Current threads will be joined.
    void resize(const std::size_t numThreads)
    {
        join();
        m_numThreads = numThreads;
        go();
    }

    // Not thread-safe, pool should be joined before calling.
    const std::vector<std::string>& errors() const
    {
        return m_errors;
    }

    // Add a threaded task, blocking until a thread is available.  If join() is
    // called, add() may not be called again until go() is called and completes.
    void add(std::function<void()> task)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        if (!m_running)
        {
            throw pdal_error("Attempted to add a task to a stopped Pool");
        }

        m_produceCv.wait(lock,
                         [this]() { return m_tasks.size() < m_queueSize; });

        m_tasks.emplace(task);

        // Notify worker that a task is available.
        lock.unlock();
        m_consumeCv.notify_all();
    }

    std::size_t size() const
    {
        return m_numThreads;
    }
    std::size_t numThreads() const
    {
        return m_numThreads;
    }

private:
    // Worker thread function.  Wait for a task and run it - or if stop() is
    // called, complete any outstanding task and return.
    void work();

    bool m_verbose;
    std::size_t m_numThreads;
    std::size_t m_queueSize;
    std::vector<std::thread> m_threads;
    std::queue<std::function<void()>> m_tasks;

    std::vector<std::string> m_errors;
    std::mutex m_errorMutex;

    std::size_t m_outstanding = 0;
    bool m_running = false;

    mutable std::mutex m_mutex;
    std::condition_variable m_produceCv;
    std::condition_variable m_consumeCv;

    // Disable copy/assignment.
    Pool(const Pool& other);
    Pool& operator=(const Pool& other);
};

class PDAL_DLL VoxelDownsizeFilter : public Filter, public Streamable
{
public:
    VoxelDownsizeFilter();
    VoxelDownsizeFilter& operator=(const VoxelDownsizeFilter&) = delete;
    VoxelDownsizeFilter(const VoxelDownsizeFilter&) = delete;

    std::string getName() const override;

private:
    virtual void addArgs(ProgramArgs& args) override;
    virtual PointViewSet run(PointViewPtr view) override;
    virtual void ready(PointTableRef) override;
    virtual bool processOne(PointRef& point) override;
    virtual void prepared(PointTableRef) override;
    virtual void done(PointTableRef) override;
    bool find(int gx, int gy, int gz);
    bool insert(int gx, int gy, int gz);

    bool voxelize(PointRef point);

    double m_cell;
    std::set<std::tuple<int, int, int>> m_populatedVoxels;
    int m_pivotVoxel[3]; // [0]: X dimension, [1]: Y dimension, [2]: Z
                             // dimension.
    bool m_pivotVoxelInitialized;
    std::string m_mode;

	bool m_isFirstInVoxelMode; // True: firstinvoxel mode, False: voxelcenter mode
	leveldb::DB* m_ldb;
	point_count_t m_batchSize=10000000;
    std::unique_ptr<Pool> m_pool;
};

} // namespace pdal




