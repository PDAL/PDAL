/******************************************************************************
 * Copyright (c) 2021, Hobu Inc.
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

#include <unordered_set>

#include "Key.hpp"

namespace pdal
{
namespace copc
{

struct Entry
{
    Entry() :
        m_key(Key::invalid()), m_offset(0), m_byteSize(-1), m_pointCount(-1)
    {}
    Entry(const Key& key) :
        m_key(key), m_offset(0), m_byteSize(-1), m_pointCount(-1)
    {}
    Entry(const Key& key, uint64_t offset, int32_t byteSize, int32_t pointCount) :
        m_key(key), m_offset(offset), m_byteSize(byteSize), m_pointCount(pointCount)
    {}

    bool valid() const
        { return m_key.valid(); }

    bool isDataEntry() const
        { return m_key.valid() && m_pointCount != -1; }

    Key m_key;
    uint64_t m_offset;
    int32_t m_byteSize;
    int32_t m_pointCount;
};

inline std::ostream& operator<<(std::ostream& out, const Entry& e)
{
    out << e.m_key << " - " << e.m_offset << "/" << e.m_byteSize << "/" << e.m_pointCount;
    return out;
}

} // namespace copc
} // namespace pdal

namespace std
{
    template<>
    struct hash<pdal::copc::Entry>
    {
        std::size_t operator()(const pdal::copc::Entry& entry) const noexcept
        {
            return std::hash<pdal::copc::Key>{}(entry.m_key);
        }
    };
}

namespace pdal
{
namespace copc
{

class Hierarchy
{
public:
    Hierarchy() = default;
    Hierarchy(const std::vector<char>& data);

    Entry find(const Key& k) const
    {
        auto it = m_entries.find(Entry(k));
        if (it != m_entries.end())
            return *it;
        return Entry();
    };

    bool insert(const Entry& entry)
    {
        return m_entries.insert(entry).second;
    }

    using iterator = std::unordered_set<Entry>::iterator;
    using const_iterator = std::unordered_set<Entry>::const_iterator;
    iterator begin()
        { return m_entries.begin(); }
    iterator end()
        { return m_entries.end(); }
    const_iterator begin() const
        { return m_entries.begin(); }
    const_iterator end() const
        { return m_entries.end(); }
    size_t size() const
        { return m_entries.size(); }
    point_count_t pointCount() const;

private:
    std::unordered_set<Entry> m_entries;
};
using HierarchyPage = Hierarchy;

inline bool operator==(const Entry& a, const Entry& b)
{
    return a.m_key == b.m_key;
}

} // namespace copc
} // namespace pdal
