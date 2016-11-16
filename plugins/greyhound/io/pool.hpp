/******************************************************************************
* Copyright (c) 2016, Connor Manning (connor@hobu.co)
*
* Entwine -- Point cloud indexing
*
* Entwine is available under the terms of the LGPL2 license. See COPYING
* for specific license text and more information.
*
******************************************************************************/

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace pdal
{
namespace entwine
{

class Pool
{
public:
    // After numThreads tasks are actively running, and queueSize tasks have
    // been enqueued to wait for an available worker thread, subsequent calls
    // to Pool::add will block until an enqueued task has been popped from the
    // queue.
    Pool(std::size_t numThreads, std::size_t queueSize = 1);
    ~Pool();

    // Start worker threads
    void go();

    // Wait for all currently running tasks to complete.
    void join();

    bool joined() const { return stop(); }

    void cycle() { join(); go(); }

    // Change the number of threads.  Current threads will be joined.
    void resize(std::size_t numThreads);

    // Wait for all current tasks to complete.  As opposed to join, tasks may
    // continue to be added while a thread is await()-ing the queue to empty.
    void await();

    // Not thread-safe, pool should be joined before calling.
    const std::vector<std::string>& errors() const { return m_errors; }

    // Add a threaded task, blocking until a thread is available.  If join() is
    // called, add() may not be called again until go() is called and completes.
    void add(std::function<void()> task);

    std::size_t numThreads() const { return m_numThreads; }

private:
    // Worker thread function.  Wait for a task and run it - or if stop() is
    // called, complete any outstanding task and return.
    void work();

    // Atomically set/get the stop flag.
    bool stop() const;
    void stop(bool val);

    std::size_t m_numThreads;
    std::size_t m_queueSize;
    std::vector<std::thread> m_threads;
    std::queue<std::function<void()>> m_tasks;
    std::atomic_size_t m_running;

    std::vector<std::string> m_errors;
    std::mutex m_errorMutex;

    std::atomic<bool> m_stop;
    std::mutex m_mutex;
    std::condition_variable m_produceCv;
    std::condition_variable m_consumeCv;

    // Disable copy/assignment.
    Pool(const Pool& other);
    Pool& operator=(const Pool& other);
};

} // namespace entwine
} // namespace pdal

