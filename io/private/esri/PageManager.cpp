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

#include <nlohmann/json.hpp>

#include "PageManager.hpp"
#include "EsriUtil.hpp"

namespace pdal
{
namespace i3s
{

PageManager::PageManager(int cacheSize, int threads, int indexFactor, i3s::FetchFunction fetch) :
    m_pool(threads, -1), m_cacheSize(cacheSize), m_indexFactor(indexFactor), m_fetch(fetch)
{}

void PageManager::fetchPage(int index)
{
    size_t size;
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        for (const PageEntry& pe : m_cache)
            if (index == pe.index)
                return;

        // Add a pending entry.
        m_cache.push_back({index, PagePtr()});
        size = m_cache.size();
    }

    if (size >= m_cacheSize)
        evict();

    // Before version 2, page indices were the actual first node value, rather than the
    // page index itself. In these cases m_indexFactor is the number of items in a page.
    std::string filename = "nodepages/" + std::to_string(index * m_indexFactor);
    m_pool.add([this, filename, index]()
    {
        std::string s = m_fetch(filename);
        PagePtr p(new Page(i3s::parse(s, "Invalid JSON in file '" + filename + "'.")));
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            for (PageEntry& pe : m_cache)
                if (index == pe.index)
                {
                    pe.page = p;
                    break;
                }
        }
        m_cv.notify_all();
    });
}


PagePtr PageManager::getPage(int index)
{
    // In case someone forgot to call fetchPage() before calling this function or in the
    // case a cached page were to get evicted since the time fetchPage() was called,
    // we loop and try to fetch again if we couldn't get the page that we expected to
    // be around.
    while (true)
    {
        PagePtr p = getPageLocked(index);
        if (p)
            return p;
        fetchPage(index);
    }
}


PagePtr PageManager::getPageLocked(int index)
{
    while (true)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        auto it = m_cache.begin();
        while (it != m_cache.end() && it->index != index)
            ++it;

        // If we don't have the index, just return.
        if (it == m_cache.end())
            return PagePtr();

        // Move to the found item to the end of the list.
        m_cache.splice(m_cache.end(), m_cache, it);

        // If we have the page, return it.
        if (it->page)
            return it->page;

        // Wait until a page comes in and see if it's the one we're interested in.
        m_cv.wait(lock);
    }

    // Should never get here.
    return PagePtr();
}

void PageManager::evict()
{
    // Erase non-pending pages until we're down to few enough pages.
    std::unique_lock<std::mutex> lock(m_mutex);
    auto it = m_cache.begin();
    while (it != m_cache.end() && m_cache.size() > m_cacheSize)
    {
        if (it->page)
            it = m_cache.erase(it);
        else
            ++it;
    }
}

} // namespace i3s
} // namespace pdal
