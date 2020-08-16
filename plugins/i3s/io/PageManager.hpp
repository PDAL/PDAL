/******************************************************************************
* Copyright (c) 2020, Hobu Inc. (info@hobu.co)
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

#include <list>
#include <memory>
#include <mutex>
#include <condition_variable>

#include <pdal/JsonFwd.hpp>
#include <pdal/util/ThreadPool.hpp>

namespace pdal
{
namespace i3s
{

using Page = NL::json;
using PagePtr = std::shared_ptr<NL::json>;
using FetchFunction = std::function<std::string(std::string)>;

class PageManager
{
    struct PageEntry
    {
        int index;
        PagePtr page;
    };

public:
    PageManager(int cacheSize, int threads, int indexFactor, FetchFunction fetch);

    void fetchPage(int index);
    PagePtr getPage(int index);

private:
    ThreadPool m_pool;
    size_t m_cacheSize;
    int m_indexFactor;
    FetchFunction m_fetch;
    std::list<PageEntry> m_cache;
    std::mutex m_mutex;
    std::condition_variable m_cv;

    PagePtr getPageLocked(int index);
    void evict();
};

} //namespace i3s
} // namespace pdal 
