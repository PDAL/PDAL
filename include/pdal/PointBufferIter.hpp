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

#include <pdal/PointBuffer.hpp>

namespace pdal
{

class PointRef
{
private:
    PointBuffer *m_buf;
    PointId m_id;

public:
    PointRef() : m_buf(NULL), m_id(0)
    {}

    PointRef(const PointRef& r) : m_buf(r.m_buf), m_id(r.m_id)
    {}

    PointRef(PointBuffer *buf, PointId id) : m_buf(buf), m_id(id)
    {}

    PointRef& operator=(const PointRef& r)
    {
        assert(m_buf == NULL || r.m_buf == m_buf);
        m_buf = r.m_buf;
        m_buf->m_index[m_id] = m_buf->m_index[r.m_id];
        return *this;
    }


    // Test for sorting.
    bool operator < (const PointRef& p) const
    {
/**
        return m_buf->compare(Dimension::Id::X, m_id, p.m_id);
**/
        double x1 = m_buf->getFieldAs<double>(Dimension::Id::X, m_id);
        double x2 = p.m_buf->getFieldAs<double>(Dimension::Id::X, p.m_id);
        return x1 < x2;
    }

    void swap(PointRef& p)
    {
        PointId id = m_buf->m_index[m_id];
        m_buf->m_index[m_id] = p.m_buf->m_index[p.m_id];
        p.m_buf->m_index[p.m_id] = id;
    }
};

void swap(PointRef && p1, PointRef && p2)
{
    p1.swap(p2);
}


class PointBufferIter :
    public std::iterator<std::random_access_iterator_tag, PointRef,
        point_count_t>
{
protected:
    PointBuffer *m_buf;
    PointId m_id;

public:
    typedef std::random_access_iterator_tag iterator_category;
    typedef typename
        std::iterator<iterator_category, PointRef>::value_type
            value_type;
    typedef typename
        std::iterator<iterator_category, PointRef>::difference_type
            difference_type;
    typedef PointRef reference;
    typedef void * pointer;


    PointBufferIter(PointBuffer *buf, PointId id) : m_buf(buf), m_id(id)
    {}

    PointBufferIter& operator++()
        { ++m_id; return *this; }
    PointBufferIter operator++(int)
        { return PointBufferIter(m_buf, m_id++); }
    PointBufferIter& operator--()
        { --m_id; return *this; }
    PointBufferIter operator--(int)
        { return PointBufferIter(m_buf, m_id--); }
    
    PointBufferIter operator+(const difference_type& n) const
        { return PointBufferIter(m_buf, m_id + n); }
    PointBufferIter operator+=(const difference_type& n)
        { m_id += n; return *this; }
    PointBufferIter operator-(const difference_type& n) const
        { return PointBufferIter(m_buf, m_id - n); }
    PointBufferIter operator-=(const difference_type& n)
        { m_id -= n; return *this; }
    difference_type operator-(const PointBufferIter& i)
        { return m_id - i.m_id; }

    bool operator==(const PointBufferIter& i)
        { return m_id == i.m_id; }
    bool operator!=(const PointBufferIter& i)
        { return m_id != i.m_id; }
    bool operator>=(const PointBufferIter& i)
        { return m_id <= i.m_id; }
    bool operator<(const PointBufferIter& i)
        { return m_id < i.m_id; }
    bool operator>(const PointBufferIter& i)
        { return m_id > i.m_id; }

    PointRef operator*() const
        { return PointRef(m_buf, m_id); }
    pointer operator->() const
        { return NULL; }
    PointRef operator[](const difference_type& n) const
        { return PointRef(m_buf, m_id); }
};

} // namespace pdal

