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

#include <memory>
#include <unordered_set>

#include "Key.hpp"

namespace pdal
{

class Accessor;
using AccessorPtr = std::unique_ptr<Accessor>;

class Accessor
{
public:
    Accessor() : m_key(Key::invalid())
    {}
    explicit Accessor(const Key& key, int32_t pointCount = 0) : m_key(key), m_pointCount(pointCount)
    {}
    virtual ~Accessor()
    {}

protected:
    Key m_key;
    int32_t m_pointCount;

public:
    Key key() const
        { return m_key; }    
    int32_t pointCount() const
        { return m_pointCount; }
    bool isDataAccessor() const
        { return m_pointCount >= 0; }
    bool valid() const
        { return m_key.valid(); }
    virtual AccessorPtr clone() const
        { return AccessorPtr(new Accessor(*this)); }
};

class EptAccessor : public Accessor
{
public:
    EptAccessor(const Key& key, int32_t pointCount) : Accessor(key, pointCount)
    {}

    point_count_t nodeId() const
        { return m_nodeId; }

    virtual AccessorPtr clone() const
    { return AccessorPtr(new EptAccessor(*this)); }

private:
    point_count_t m_nodeId;
};

/**
class CopcAccessor : public Accessor
{
    uint64_t m_offset;
    uint64_t m_byteSize;
};
**/

inline bool operator<(const AccessorPtr& a, const AccessorPtr& b)
{
    return a->key() < b->key();
}

inline bool operator==(const AccessorPtr& a, const AccessorPtr& b)
{
    return a->key() == b->key();
}


} // namespace pdal

namespace std
{
    template<>
    struct hash<pdal::AccessorPtr>
    {
        std::size_t operator()(const pdal::AccessorPtr& acc) const noexcept
        {
            return std::hash<pdal::Key>{}(acc->key());
        }
    };
}

namespace pdal
{

class Hierarchy
{
public:
    class Iterator
    {
    public:
        Iterator(std::unordered_set<AccessorPtr>::const_iterator i) : p(i)
        {}
        Iterator operator++()
        { ++p; return *this; }
        bool operator!=(const Iterator & other) const
        { return p != other.p; }
        bool operator==(const Iterator & other) const
        { return p == other.p; }
        const Accessor& operator*() const
        { return *(*p); }
    private:
        std::unordered_set<AccessorPtr>::const_iterator p;
    };

    Hierarchy(const std::string& source = "") : m_source(source)
    {}

    Hierarchy(Hierarchy&& other) = default;

    const Accessor& find(const Accessor& acc) const
    {
        return find(acc.key());
    }

    const Accessor& find(const Key& k) const
    {
        static Accessor nullAccessor;

        AccessorPtr acc(new Accessor(k));
        auto it = m_accessors.find(acc);
        if (it == m_accessors.end())
            return nullAccessor;
        const AccessorPtr& found = *it;
        return *found;
    }

    void insert(AccessorPtr&& acc)
    {
        m_accessors.insert(std::move(acc));
    }

    size_t size() const
    {
        return m_accessors.size();
    }

    std::string source() const
    {
        return m_source;
    }

    point_count_t pointCount() const
    {
        point_count_t c = 0;
        for (const AccessorPtr& acc : m_accessors)
            c += acc->pointCount();
        return c;
    }

    Iterator begin() const
    { return Iterator(m_accessors.cbegin()); }
    Iterator end() const
    { return Iterator(m_accessors.cend()); }

private:
    std::string m_source;
    std::unordered_set<AccessorPtr> m_accessors;
};
using HierarchyPage = Hierarchy;

} // namespace pdal

