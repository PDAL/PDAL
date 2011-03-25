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

#include <libpc/libpc.hpp>

#include <cassert>
#include <vector>
#include <sstream>

#include <libpc/Vector.hpp>
#include <libpc/Range.hpp>

namespace libpc
{

template <typename T>
class LIBPC_DLL Bounds
{
public:
    typedef typename std::vector< Range<T> > RangeVector;

private:
    RangeVector m_ranges;

public:
    Bounds<T>()
    {
        m_ranges.resize(0);
    }

    Bounds(Bounds const& other)
        : m_ranges(other.m_ranges)
    {
    }

    Bounds(RangeVector const& ranges)
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

        assert(verify());

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

        assert(verify());
    }

    Bounds(const Vector<T>& minimum, const Vector<T>& maximum)
    {
        assert(minimum.size() == maximum.size());

        m_ranges.resize(minimum.size());
    
        for (std::size_t i=0; i<minimum.size(); i++)
        {
            m_ranges[i].setMinimum(minimum[i]);
            m_ranges[i].setMaximum(maximum[i]);
        }    

        assert(verify());
    }

    T getMinimum(std::size_t const& index) const
    {
        if (m_ranges.size() <= index)
        {
            // std::ostringstream msg;
            // msg << "Bounds dimensions, " << ranges.size() <<", is less "
            //     << "than the given index, " << index;
            // throw std::runtime_error(msg.str());
            return 0;
        }
        return m_ranges[index].getMinimum();
    }

    void setMinimum(std::size_t const& index, T v)
    {
        if (m_ranges.size() <= index)
        {
            m_ranges.resize(index + 1);
        }
        m_ranges[index].setMinimum(v);
    }

    T getMaximum(std::size_t const& index) const
    {
        if (m_ranges.size() <= index)
        {
            // std::ostringstream msg;
            // msg << "Bounds dimensions, " << m_ranges.size() <<", is less "
            //     << "than the given index, " << index;
            // throw std::runtime_error(msg.str());
            return 0;
        }
        return m_ranges[index].getMaximum();
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
            vec.push_back(m_ranges[i].getMinimum());
        }
    
        return Vector<T>(vec);
    }

    Vector<T> getMaximum() 
    {
        std::vector<T> vec;
    
        for (std::size_t i=0; i<m_ranges.size(); i++)
        {
            vec.push_back(m_ranges[i].getMaximum());
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
    RangeVector const& dimensions() const
    {
        return m_ranges;
    }

    /// The number of dimensions of the Bounds
    std::size_t size() const
    {
        return m_ranges.size();
    }

    /// Is this Bounds equal to other?
    bool equal(Bounds<T> const& other) const
    {
        if (size() != other.size())
        {
          return false;
        }
        for (std::size_t i = 0; i < size(); i++)
        {
            if (m_ranges[i] != other.m_ranges[i])
            {
                return false;
            }
        }
        return true;
    }

    /// Synonym for intersects for now
    bool overlaps(Bounds const& other) const
    {
        if (other.size() != size())
        {
            return false;
        }

        for (std::size_t i = 0; i < size(); i++)
        {
            if ( m_ranges[i].overlaps(other.m_ranges[i]) )
                return true;
        }

        return false;
    }

    /// Does this Bounds contain a point?
    bool contains(Vector<T> point) const
    {
        if (point.size() != size())
            return false;

        for (std::size_t i = 0; i < size(); i++)
        {
            // As soon as it is not contains, we're false
            if (! m_ranges[i].contains(point[i]) )
                return false;
        }
        return true;
    }

    /// Does this Bounds contain other?
    bool contains(Bounds<T> const& other) const
    {
        for (std::size_t i = 0; i < size(); i++)
        {
            // As soon as it is not contains, we're false
            if (! m_ranges[i].contains(other.m_ranges[i]) )
                return false;
        }
        return true;
    }

    /// Shift each dimension by a vector of deltas
    void shift(std::vector<T> deltas)
    {
        std::size_t i;
        if( size() != deltas.size())
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
        std::size_t i;
        if( size() != deltas.size())
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
        RangeVector ds = r.dimensions();
        for (std::size_t i = 0; i < size(); ++i)
        {
            m_ranges[i].clip(ds[i]);
        }
    }

    /// Grow to the union of two liblas::Bounds
    void grow(Bounds const& r)
    {
        RangeVector ds = r.dimensions();
        for (std::size_t i = 0; i < size(); ++i)
        {
            m_ranges[i].grow(ds[i]);
        }
    }

    /// Expand the liblas::Bounds to include this point
    void grow(Vector<T> const& point)
    {
        assert(point.size() == size());
        for (std::size_t i = 0; i < size(); ++i)
        {
            m_ranges[i].grow(point[i]);
        }
    }

    T volume() const
    {
        T output = T(1);
        for (std::size_t i = 0; i < size(); i++)
        {
            output = output * m_ranges[i].length();
        }

        return output;
    }

    bool empty() const
    {
        if (size()==0)
        {
          return true;
        }

        for (std::size_t i = 0; i < size(); i++)
        {
            if (m_ranges[i].empty())
            {
                return true;
            }
        }
        return false;
    }

    bool verify()
    {
        for (std::size_t d = 0; d < size(); ++d)
        {
            if (getMinimum(d) > getMaximum(d) )
            {
                // Check that we're not infinity either way
                if (Utils::compare_distance<T>(getMinimum(d), std::numeric_limits<T>::max()) ||
                    Utils::compare_distance<T>(getMaximum(d), -std::numeric_limits<T>::max()))
                {
                    std::ostringstream msg;
                    msg << "liblas::Bounds::verify: Minimum point at dimension " << d
                        << "is greater than maximum point.  Neither point is infinity.";
                    throw std::runtime_error(msg.str());
                }
            }
        }
        return true;
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

    static const Bounds<T>& getDefaultSpatialExtent()
    {
        static T minv(std::numeric_limits<T>::min());
        static T maxv(std::numeric_limits<T>::max());
        static Bounds v(minv,minv,minv,maxv,maxv,maxv);
        return v;
    }
};

template<class T>
std::ostream& operator<<(std::ostream& ostr, const Bounds<T>& bounds)
{
    ostr << "(";
    for (std::size_t d = 0; d < bounds.size(); ++d)
    {
        const Range<T>& r = bounds.dimensions()[d];
        ostr << r;
        if (d!=bounds.size()-1) ostr << ", ";
    }
    ostr << ")";
    return ostr;
}

} // namespace libpc

// Needed for C++ DLL exports
#ifdef LIBPC_COMPILER_MSVC
template class LIBPC_DLL libpc::Range<double>;
template class LIBPC_DLL libpc::Bounds<double>;
#endif

#endif
