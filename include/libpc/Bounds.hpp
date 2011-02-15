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

#include <cassert>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>

#include "libpc/export.hpp"
#include "libpc/Vector.hpp"
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

    Bounds(const Vector<T>& minimum, const Vector<T>& maximum)
    {
        assert(minimum.size() == maximum.size());

        m_ranges.resize(minimum.size());
    
        for (std::size_t i=0; i<minimum.size(); i++)
        {
            m_ranges[i].setMinimum(minimum.vN(i));
            m_ranges[i].setMaximum(maximum.vN(i));
        }    

    #ifdef DEBUG
        verify();
    #endif
    }

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

    Vector<T> getMinimum() 
    {
        std::vector<T> vec;
    
        for (std::size_t i=0; i<m_ranges.size(); i++)
        {
            vec.push_back(m_ranges[i].minimum());
        }
    
        return Vector<T>(vec);
    }

    Vector<T> getMaximum() 
    {
        std::vector<T> vec;
    
        for (std::size_t i=0; i<m_ranges.size(); i++)
        {
            vec.push_back(m_ranges[i].maximum());
        }
    
        return Vector<T>(vec);
    }

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

    /// Does this Bounds contain a point?
    bool contains(Vector<T> point) const
    {
        if (point.size() != size())
            return false;

        for (size_type i = 0; i < size(); i++)
        {
            // As soon as it is not contains, we're false
            if (! m_ranges[i].contains(point.vN(i)) )
                return false;
        }
        return true;
    }

    /// Does this Bounds contain other?
    bool contains(Bounds<T> const& other) const
    {
        for (size_type i = 0; i < size(); i++)
        {
            // As soon as it is not contains, we're false
            if (! m_ranges[i].contains(other.m_ranges[i]) )
                return false;
        }
        return true;
    }

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
    void grow(Vector<T> const& point)
    {
        assert(point.size() == size());
        for (size_type i = 0; i < size(); ++i)
        {
            m_ranges[i].grow(point.vN(i));
        }
    }

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
