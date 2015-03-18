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

class PointRef
{
private:
    PointView *m_buf;
    PointId m_id;
    bool m_tmp;

public:
    PointRef() : m_buf(NULL), m_id(0), m_tmp(false)
    {}
    PointRef(const PointRef& r) : m_buf(r.m_buf)
    {
        m_id = m_buf->getTemp(r.m_id);
        m_tmp = true;
    }
    // This is the ctor used to make a PointRef from an iterator.
    PointRef(PointView *buf, PointId id) : m_buf(buf), m_id(id), m_tmp(false)
    {}

    ~PointRef()
    {
        if (m_tmp)
            m_buf->freeTemp(m_id);
    }

    PointRef& operator=(const PointRef& r)
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

    bool compare(Dimension::Id::Enum dim, const PointRef& p) const
        { return m_buf->compare(dim, m_id, p.m_id); }

    void swap(PointRef& p)
    {
        PointId id = m_buf->m_index[m_id];
        m_buf->m_index[m_id] = p.m_buf->m_index[p.m_id];
        p.m_buf->m_index[p.m_id] = id;
    }
};

inline void swap(PointRef && p1, PointRef && p2)
{
    p1.swap(p2);
}

class PointViewIter :
    public std::iterator<std::random_access_iterator_tag, PointRef,
        point_count_t>
{
protected:
    PointView *m_buf;
    PointId m_id;

public:
    typedef std::random_access_iterator_tag iterator_category;
    typedef std::iterator<iterator_category, PointRef>::value_type
            value_type;
    typedef std::iterator<iterator_category, PointRef>::difference_type
            difference_type;
    typedef PointRef reference;
    typedef void * pointer;


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
    difference_type operator-(const PointViewIter& i)
        { return m_id - i.m_id; }

    bool operator==(const PointViewIter& i)
        { return m_id == i.m_id; }
    bool operator!=(const PointViewIter& i)
        { return m_id != i.m_id; }
    bool operator>=(const PointViewIter& i)
        { return m_id <= i.m_id; }
    bool operator<(const PointViewIter& i)
        { return m_id < i.m_id; }
    bool operator>(const PointViewIter& i)
        { return m_id > i.m_id; }

    PointRef operator*() const
        { return PointRef(m_buf, m_id); }
    pointer operator->() const
        { return NULL; }
    PointRef operator[](const difference_type& /*n*/) const
        { return PointRef(m_buf, m_id); }
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

