/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#ifndef INCLUDED_RANGE_HPP
#define INCLUDED_RANGE_HPP

#include <pdal/pdal_internal.hpp>
#include <pdal/Utils.hpp>

namespace pdal
{

template <typename T>
class Bounds;

template <typename T>
class PDAL_DLL Range
{
    friend class Bounds<T>;
private:
    T m_minimum;
    T m_maximum;

public:
    typedef T value_type;

    Range()
    {
        clear();
    }

    Range(T minimum, T maximum)
        : m_minimum(minimum)
        , m_maximum(maximum)
    {
    }

    bool operator==(Range<T> const& rhs) const
    {
        return equal(rhs);
    }

    bool operator!=(Range const& rhs) const
    {
        return !(equal(rhs));
    }

    T getMinimum() const
    {
        return m_minimum;
    }

    void setMinimum(T value)
    {
        m_minimum = value;
    }

    T getMaximum() const
    {
        return m_maximum;
    }

    void setMaximum(T value)
    {
        m_maximum = value;
    }

    bool equal(Range const& other) const
    {
        return Utils::compare_distance(m_minimum, other.m_minimum) &&
            Utils::compare_distance(m_maximum, other.m_maximum);
    }

    bool overlaps(Range const& r) const
    {
        return m_minimum <= r.m_maximum && m_maximum >= r.m_minimum;
    }

    bool contains(Range const& r) const
    {
        return m_minimum <= r.m_minimum && r.m_maximum <= m_maximum;
    }

    bool contains(T v) const
    {
        return m_minimum <= v && v <= m_maximum;
    }

    bool empty(void) const
    {
        return
            Utils::compare_distance(m_minimum,
                (std::numeric_limits<T>::max)()) &&
            Utils::compare_distance(m_maximum,
                (std::numeric_limits<T>::min)());
    }

    void shift(T v)
    {
        m_minimum += v;
        m_maximum += v;
    }

    void scale(T v)
    {
        m_minimum *= v;
        m_maximum *= v;
    }

    void clip(Range const& r)
    {
        if (r.m_minimum > m_minimum)
            m_minimum = r.m_minimum;
        if (r.m_maximum < m_maximum)
            m_maximum = r.m_maximum;
    }

    void grow(T v)
    {
        if (v < m_minimum)
            m_minimum = v;
        if (v > m_maximum)
            m_maximum = v;
    }

    void grow(Range const& r)
    {
        grow(r.m_minimum);
        grow(r.m_maximum);
    }

    void grow(T lo, T hi)
    {
        grow(lo);
        grow(hi);
    }

    void clear()
    {
        m_minimum = (std::numeric_limits<T>::max)();
        m_maximum = (std::numeric_limits<T>::min)();
    }

    T length() const
    {
        return m_maximum - m_minimum;
    }

    void dump() const
    {
        std::cout << *this;
    }
};


template<class T>
std::ostream& operator<<(std::ostream& ostr, const Range<T>& range)
{
    ostr << "[" << range.getMinimum() << ", " << range.getMaximum() << "]";
    return ostr;
}


extern PDAL_DLL std::istream& operator>>(std::istream& istr, Range<double>& range);


} // namespace pdal

#endif
