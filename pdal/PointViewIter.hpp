/******************************************************************************
* Copyright (c) 2014, Hobu Inc.
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

#include <iterator>

#include <pdal/PointView.hpp>

namespace pdal
{

class PointIdxRef
{
private:
    PointView *m_buf;
    PointId m_id;
    bool m_tmp;

public:
    PointIdxRef() : m_buf(NULL), m_id(0), m_tmp(false)
    {}
    PointIdxRef(const PointIdxRef& r) : m_buf(r.m_buf)
    {
        m_id = m_buf->getTemp(r.m_id);
        m_tmp = true;
    }
    // This is the ctor used to make a PointIdxRef from an iterator.
    PointIdxRef(PointView *buf, PointId id) : m_buf(buf), m_id(id), m_tmp(false)
    {}

    ~PointIdxRef()
    {
        if (m_tmp)
            m_buf->freeTemp(m_id);
    }

    PointIdxRef& operator=(const PointIdxRef& r)
    {
        assert(m_buf == NULL || r.m_buf == m_buf);
        if (!m_buf)
        {
            m_buf = r.m_buf;
            m_id = m_buf->getTemp(r.m_id);
            m_tmp = true;
        }
        else
            m_buf->m_index[m_id] = r.m_buf->m_index[r.m_id];
        return *this;
    }

    bool compare(Dimension::Id dim, const PointIdxRef& p) const
        { return m_buf->compare(dim, m_id, p.m_id); }

    void swap(PointIdxRef& p)
    {
        PointId id = m_buf->m_index[m_id];
        m_buf->m_index[m_id] = p.m_buf->m_index[p.m_id];
        p.m_buf->m_index[p.m_id] = id;
    }
};

inline void swap(PointIdxRef && p1, PointIdxRef && p2)
{
    p1.swap(p2);
}

class PointViewIter :
    public std::iterator<std::random_access_iterator_tag, PointIdxRef,
        point_count_t>
{
protected:
    PointView *m_buf;
    PointId m_id;

public:
    typedef std::random_access_iterator_tag iterator_category;
    typedef std::iterator<iterator_category, PointIdxRef>::value_type
            value_type;
    typedef std::iterator<iterator_category, PointIdxRef>::difference_type
            difference_type;
    typedef PointIdxRef reference;
    typedef void * pointer;

    PointViewIter() {}
    PointViewIter(PointView *buf, PointId id) : m_buf(buf), m_id(id)
    {}

    PointViewIter& operator++()
        { ++m_id; return *this; }
    PointViewIter operator++(int)
        { return PointViewIter(m_buf, m_id++); }
    PointViewIter& operator--()
        { --m_id; return *this; }
    PointViewIter operator--(int)
        { return PointViewIter(m_buf, m_id--); }

    PointViewIter operator+(const difference_type& n) const
        { return PointViewIter(m_buf, m_id + n); }
    PointViewIter operator+=(const difference_type& n)
        { m_id += n; return *this; }
    PointViewIter operator-(const difference_type& n) const
        { return PointViewIter(m_buf, m_id - n); }
    PointViewIter operator-=(const difference_type& n)
        { m_id -= n; return *this; }
    difference_type operator-(const PointViewIter& i) const
        { return m_id - i.m_id; }

    bool operator==(const PointViewIter& i)
        { return m_id == i.m_id; }
    bool operator!=(const PointViewIter& i)
        { return m_id != i.m_id; }
    bool operator<=(const PointViewIter& i)
        { return m_id <= i.m_id; }
    bool operator>=(const PointViewIter& i)
        { return m_id >= i.m_id; }
    bool operator<(const PointViewIter& i)
        { return m_id < i.m_id; }
    bool operator>(const PointViewIter& i)
        { return m_id > i.m_id; }

    PointIdxRef operator*() const
        { return PointIdxRef(m_buf, m_id); }
    pointer operator->() const
        { return NULL; }
    PointIdxRef operator[](const difference_type& /*n*/) const
        { return PointIdxRef(m_buf, m_id); }
};

} // namespace pdal

/**
namespace std
{
template<>
inline void iter_swap(pdal::PointViewIter a, pdal::PointViewIter b)
{
    swap(*a, *b);
}
}
**/

