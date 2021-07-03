/******************************************************************************
 * Copyright (c) 2018, Connor Manning
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <cassert>
#include <condition_variable>
#include <functional>
#include <queue>
#include <thread>

#include <pdal/pdal_types.hpp>

namespace pdal
{

class ThreadPool
{
public:
    // After numThreads tasks are actively running, and queueSize tasks have
    // been enqueued to wait for an available worker thread, subsequent calls
    // to Pool::add will block until an enqueued task has been popped from the
    // queue.
    PDAL_DLL ThreadPool(std::size_t numThreads, int64_t queueSize = -1,
            bool verbose = true) :
        m_queueSize(queueSize),
        m_numThreads(std::max<std::size_t>(numThreads, 1)), m_verbose(verbose)
    {
        assert(m_queueSize != 0);
        go();
    }

    PDAL_DLL ~ThreadPool()
    { join(); }

    ThreadPool(const ThreadPool& other) = delete;
    ThreadPool& operator=(const ThreadPool& other) = delete;

    // Start worker threads.
    PDAL_DLL void go();

    // Disallow the addition of new tasks and wait for all currently running
    // tasks to complete.
    PDAL_DLL void join()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        if (!m_running) return;
        m_running = false;
        lock.unlock();

        m_consumeCv.notify_all();
        for (auto& t : m_threads)
            t.join();
        m_threads.clear();
    }

    // join() and empty the queue of tasks that may have been waiting to run.
    PDAL_DLL void stop()
    {
        join();

        // Effectively clear the queue.
        std::queue<std::function<void()>> q;
        m_tasks.swap(q);
    }

    // Wait for all current tasks to complete.  As opposed to join, tasks may
    // continue to be added while a thread is await()-ing the queue to empty.
    PDAL_DLL void await()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_produceCv.wait(lock, [this]()
        {
            return !m_outstanding && m_tasks.empty();
        });
    }

    // Join and restart.
    PDAL_DLL void cycle()
    { join(); go(); }

    // Change the number of threads.  Current threads will be joined.
    PDAL_DLL void resize(const std::size_t numThreads)
    {
        join();
        m_numThreads = numThreads;
        go();
    }

    // Not thread-safe, pool should be joined before calling.
    const std::vector<std::string>& errors() const
    { return m_errors; }

    // Add a threaded task, blocking until a thread is available.  If join() is
    // called, add() may not be called again until go() is called and completes.
    PDAL_DLL void add(std::function<void()> task)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        if (!m_running)
        {
            throw pdal_error("Attempted to add a task to a stopped ThreadPool");
        }

        m_produceCv.wait(lock, [this]()
        {
            return m_queueSize < 0 || m_tasks.size() < (size_t)m_queueSize;
        });

        m_tasks.emplace(task);

        // Notify worker that a task is available.
        lock.unlock();
        m_consumeCv.notify_all();
    }

    PDAL_DLL std::size_t size() const
    { return m_numThreads; }

    PDAL_DLL std::size_t numThreads() const
    { return m_numThreads; }

private:
    // Worker thread function.  Wait for a task and run it.
    void work();

    int64_t m_queueSize;
    std::size_t m_numThreads;
    bool m_verbose;
    std::vector<std::thread> m_threads;
    std::queue<std::function<void()>> m_tasks;

    std::vector<std::string> m_errors;
    std::mutex m_errorMutex;

    std::size_t m_outstanding = 0;
    bool m_running = false;

    mutable std::mutex m_mutex;
    std::condition_variable m_produceCv;
    std::condition_variable m_consumeCv;
};

} // namespace pdal

