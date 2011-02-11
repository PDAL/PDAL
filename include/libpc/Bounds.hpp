/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS bounds class
 * Author:   Howard Butler, hobu.inc@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2010, Howard Butler
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

#ifndef INCLUDED_BOUNDS_HPP
#define INCLUDED_BOUNDS_HPP

#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>

#include "libpc/export.hpp"
#include "libpc/Range.hpp"
#include "libpc/Utils.hpp"

namespace libpc
{

template <typename T>
class LIBPC_DLL Bounds
{
public:
    typedef T value_type;
    typedef typename std::vector< Range<T> >::size_type size_type;
    typedef typename std::vector< Range<T> > RangeVec;

private:
    RangeVec m_ranges;

public:
    Bounds<T>()
    {
        m_ranges.resize(0);
    }

    Bounds(Bounds const& other)
        : m_ranges(other.m_ranges)
    {
    }

    Bounds(RangeVec const& ranges)
        :
        m_ranges(ranges)
    {
    }

    Bounds( T minx,
            T miny,
            T minz,
            T maxx,
            T maxy,
            T maxz)
    {
        m_ranges.resize(3);

        m_ranges[0].setMinimum(minx);
        m_ranges[1].setMinimum(miny);
        m_ranges[2].setMinimum(minz);

        m_ranges[0].setMaximum(maxx);
        m_ranges[1].setMaximum(maxy);
        m_ranges[2].setMaximum(maxz);

#ifdef DEBUG
        verify();
#endif

    }

    Bounds( T minx,
            T miny,
            T maxx,
            T maxy)
    {

        m_ranges.resize(2);

        m_ranges[0].setMinimum(minx);
        m_ranges[1].setMinimum(miny);

        m_ranges[0].setMaximum(maxx);
        m_ranges[1].setMaximum(maxy);

#ifdef DEBUG
        verify();
#endif

    }

    ////Bounds( const Point& min, const Point& max)
    ////{
    ////    m_ranges.resize(3);
    ////
    ////    m_ranges[0].minimum = min.GetX();
    ////    m_ranges[1].minimum = min.GetY();
    ////    m_ranges[2].minimum = min.GetZ();
    ////
    ////    m_ranges[0].maximum = max.GetX();
    ////    m_ranges[1].maximum = max.GetY();
    ////    m_ranges[2].maximum = max.GetZ();
    ////
    ////#ifdef DEBUG
    ////    verify();
    ////#endif
    ////}

    T minimum(std::size_t const& index) const
    {
        if (m_ranges.size() <= index)
        {
            // std::ostringstream msg;
            // msg << "Bounds dimensions, " << ranges.size() <<", is less "
            //     << "than the given index, " << index;
            // throw std::runtime_error(msg.str());
            return 0;
        }
        return m_ranges[index].minimum();
    }

    void setMinimum(std::size_t const& index, T v)
    {
        if (m_ranges.size() <= index)
        {
            m_ranges.resize(index + 1);
        }
        m_ranges[index].setMinimum(v);
    }

    T maximum(std::size_t const& index) const
    {
        if (m_ranges.size() <= index)
        {
            // std::ostringstream msg;
            // msg << "Bounds dimensions, " << m_ranges.size() <<", is less "
            //     << "than the given index, " << index;
            // throw std::runtime_error(msg.str());
            return 0;
        }
        return m_ranges[index].maximum();
    }

    void setMaximum(std::size_t const& index, T v)
    {
        if (m_ranges.size() <= index)
        {
            m_ranges.resize(index + 1);
        }
        m_ranges[index].setMaximum(v);
    }

    ////liblas::Point (min)() {
    ////    liblas::Point p;
    ////    try
    ////    {
    ////        p.SetCoordinates(m_ranges[0].minimum, m_ranges[1].minimum, m_ranges[2].minimum);
    ////    }
    ////    catch (std::runtime_error const& e)
    ////    {
    ////        ::boost::ignore_unused_variable_warning(e);
    ////        p.SetCoordinates(m_ranges[0].minimum, m_ranges[1].minimum, 0);
    ////
    ////    }
    ////
    ////    return p;
    ////}
    ////
    ////liblas::Point (max)() {
    ////    liblas::Point p;
    ////    try
    ////    {
    ////        p.SetCoordinates(m_ranges[0].maximum, m_ranges[1].maximum, m_ranges[2].maximum);
    ////    }
    ////    catch (std::runtime_error const& e)
    ////    {
    ////        ::boost::ignore_unused_variable_warning(e);
    ////        p.SetCoordinates(m_ranges[0].maximum, m_ranges[1].maximum, 0);
    ////
    ////    }
    ////    return p;
    ////}

    inline bool operator==(Bounds<T> const& rhs) const
    {
        return equal(rhs);
    }

    inline bool operator!=(Bounds<T> const& rhs) const
    {
        return (!equal(rhs));
    }


    Bounds<T>& operator=(Bounds<T> const& rhs)
    {
        if (&rhs != this)
        {
            m_ranges = rhs.m_ranges;
        }
        return *this;
    }

    /// The vector of Range<T> for the Bounds
    RangeVec const& dims () const
    {
        return m_ranges;
    }

    /// The number of dimensions of the Bounds
    size_type size() const
    {
        return m_ranges.size();
    }

    /// Resize the dimensionality of the Bounds to d
    void resize(size_type d)
    {
        if (m_ranges.size() < d)
        {
            m_ranges.resize(d);
        }
    }

    /// Is this Bounds equal to other?
    bool equal(Bounds<T> const& other) const
    {
        for (size_type i = 0; i < size(); i++)
        {
            if ( m_ranges[i] != other.m_ranges[i] )
                return false;
        }
        return true;
    }

/// Does this Bounds intersect other?
    bool intersects(Bounds const& other) const
    {

        for (size_type i = 0; i < size(); i++)
        {
            if ( m_ranges[i].overlaps(other.m_ranges[i]) )
                return true;
        }

        return false;

    }

    /// Synonym for intersects for now
    bool overlaps(Bounds const& other) const
    {
        return intersects(other);
    }

    /// Does this Bounds contain other?
    bool contains(Bounds const& other) const
    {
        for (size_type i = 0; i < size(); i++)
        {
            // As soon as it is not contains, we're false
            if (! m_ranges[i].contains(other.m_ranges[i]) )
                return false;
        }
        return true;
    }

    /////// Does this Bounds this point other?
    ////bool contains(Point const& point) const
    ////{
    ////    // std::cout << m_ranges[0].length() << std::endl;
    ////    // std::cout << "x contain: " << m_ranges[0].contains(point.GetX())
    ////    //           << " r.x.min: " << m_ranges[0].min
    ////    //           << " r.x.max: " << m_ranges[0].max
    ////    //           << " p.x: " << point.GetX() << std::endl;
    ////    // std::cout << "y contain: " << m_ranges[1].contains(point.GetY())
    ////    //           << " r.y.min: " << m_ranges[1].min
    ////    //           << " r.y.max: " << m_ranges[1].max
    ////    //           << " p.y: " << point.GetY() << std::endl;
    ////    // std::cout << "z contain: " << m_ranges[2].contains(point.GetZ())
    ////    //           << " r.z.min: " << m_ranges[2].min
    ////    //           << " r.z.max: " << m_ranges[2].max
    ////    //           << " p.z: " << point.GetZ() << std::endl;
    ////    if (!m_ranges[0].contains(point.GetX()))
    ////        return false;
    ////    if (!m_ranges[1].contains(point.GetY()))
    ////        return false;
    ////
    ////    // If our z bounds has no length, we'll say it's contained anyway.
    ////    if (!m_ranges[2].contains(point.GetZ()))
    ////    {
    ////        if (detail::compare_distance(m_ranges[2].length(), 0.0))
    ////            return true;
    ////        return false;
    ////    }
    ////    return true;
    ////}

    /// Shift each dimension by a vector of detlas
    void shift(std::vector<T> deltas)
    {
        typedef typename std::vector< T >::size_type size_type;

        size_type i;
        if( size() <= deltas.size())
        {
            std::ostringstream msg;
            msg << "liblas::Bounds::shift: Delta vector size, " << deltas.size()
                << ", is larger than the dimensionality of the bounds, "<< size() << ".";
            throw std::runtime_error(msg.str());
        }
        for (i = 0; i < deltas.size(); ++i)
        {
            m_ranges[i].shift(deltas[i]);
        }
    }

    /// Scale each dimension by a vector of deltas
    void scale(std::vector<T> deltas)
    {
        typedef typename std::vector< T >::size_type size_type;

        size_type i;
        if( size() <= deltas.size())
        {
            std::ostringstream msg;
            msg << "liblas::Bounds::scale: Delta vector size, " << deltas.size()
                << ", is larger than the dimensionality of the bounds, "<< size() << ".";
            throw std::runtime_error(msg.str());
        }
        for (i = 0; i < deltas.size(); ++i)
        {
            m_ranges[i].scale(deltas[i]);
        }
    }

    /// Clip this Bounds to the extent of r
    void clip(Bounds const& r)
    {
        RangeVec ds = r.dims();
        for (size_type i = 0; i < size(); ++i)
        {
            m_ranges[i].clip(ds[i]);
        }
    }

    /// Grow to the union of two liblas::Bounds
    void grow(Bounds const& r)
    {
        RangeVec ds = r.dims();
        for (size_type i = 0; i < size(); ++i)
        {
            m_ranges[i].grow(ds[i]);
        }
    }

    /// Expand the liblas::Bounds to include this point
    ////void grow(Point const& p)
    ////{
    ////    m_ranges[0].grow(p.GetX());
    ////    m_ranges[1].grow(p.GetY());
    ////    m_ranges[2].grow(p.GetZ());
    ////}

    T volume() const
    {
        T output = T();
        for (size_type i = 0; i < size(); i++)
        {
            output = output * m_ranges[i].length();
        }

        return output;
    }

    bool empty() const
    {
        for (size_type i = 0; i < size(); i++)
        {
            if (m_ranges[i].empty())
                return true;
        }
        return false;
    }

    void verify()
    {
        for (size_type d = 0; d < size(); ++d)
        {
            if (minimum(d) > maximum(d) )
            {
                // Check that we're not infinity either way
                if (Utils::compare_distance(minimum(d), std::numeric_limits<T>::max()) ||
                    Utils::compare_distance(maximum(d), -std::numeric_limits<T>::max()))
                {
                    std::ostringstream msg;
                    msg << "liblas::Bounds::verify: Minimum point at dimension " << d
                        << "is greater than maximum point.  Neither point is infinity.";
                    throw std::runtime_error(msg.str());
                }
            }
        }
    }

    ////Bounds<T> project(liblas::SpatialReference const& in_ref, liblas::SpatialReference const& out_ref)
    ////{
    ////    liblas::ReprojectionTransform trans(in_ref, out_ref);
    ////
    ////    liblas::Point minimum = (min)();
    ////    liblas::Point maximum = (max)();
    ////    trans.transform(minimum);
    ////    trans.transform(maximum);
    ////    return Bounds<T>(minimum, maximum);
    ////}
};

template<class T>
std::ostream& operator<<(std::ostream& ostr, const Bounds<T>& bounds)
{
    for (size_t d = 0; d < bounds.size(); ++d)
    {
        const Range<T>& r = bounds.dims()[d];
        ostr << "(" <<  r.minimum() << "," << r.maximum() << ")";
    }
    return ostr;
}

} // namespace libpc

// Needed for C++ DLL exports
#ifdef _MSC_VER
template class LIBPC_DLL libpc::Range<double>;
template class LIBPC_DLL libpc::Bounds<double>;
#endif

#endif
