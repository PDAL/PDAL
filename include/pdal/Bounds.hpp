/******************************************************************************
 * $Id$
 *
 * Project:  pdal - http://pdal.org - A BSD library for LAS format data.
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

#include <pdal/pdal_internal.hpp>

#include <cassert>
#include <vector>
#include <sstream>

#include <pdal/Vector.hpp>
#include <pdal/Range.hpp>

namespace pdal
{


/*!
    \verbatim embed:rst

    Bounds is for manipulating n-dimensional ranges of data.  Typically
    used for defining the spatial extents of XYZ data, this class can also be
    used for defining bounds of other dimensions.

    \endverbatim
*/

static const double mind = (std::numeric_limits<double>::min)();
static const double maxd = (std::numeric_limits<double>::max)();
class PDAL_DLL BOX3D
{
public:

    BOX3D()
    : minx(maxd)
    , maxx(mind)
    , miny(maxd)
    , maxy(mind)
    , minz(maxd)
    , maxz(mind) {}

    BOX3D( double minx,
           double miny,
           double maxx,
           double maxy)
    : minx(minx)
    , maxx(maxx)
    , miny(miny)
    , maxy(maxy)
    , minz(maxd)
    , maxz(mind)
    {}

    BOX3D( double minx,
           double miny,
           double minz,
           double maxx,
           double maxy,
           double maxz)
    : minx(minx)
    , maxx(maxx)
    , miny(miny)
    , maxy(maxy)
    , minz(minz)
    , maxz(maxz)
    {}

    double minx;
    double maxx;
    double miny;
    double maxy;
    double minz;
    double maxz;

    bool empty() const
    {
        return  Utils::compare_distance(minx, maxd) && Utils::compare_distance(maxx, mind) &&
                Utils::compare_distance(miny, maxd) && Utils::compare_distance(maxy, mind) &&
                Utils::compare_distance(minz, maxd) && Utils::compare_distance(maxz, mind);
    }
    bool contains(double x, double y, double z) const
    {
        return minx <= x && x <= maxx &&
               miny <= y && y <= maxy &&
               minz <= z && z <= maxz;
    }

    bool contains(const BOX3D& other) const
    {
        return minx <= other.minx && other.maxx <= maxx &&
               miny <= other.miny && other.maxy <= maxy &&
               minz <= other.minz && other.maxz <= maxz;
    }
    bool equal(const BOX3D& other) const
    {
        return  Utils::compare_distance(minx, other.minx) && Utils::compare_distance(maxx, other.maxx) &&
                Utils::compare_distance(miny, other.miny) && Utils::compare_distance(maxy, other.maxy) &&
                Utils::compare_distance(minz, other.minz) && Utils::compare_distance(maxz, other.maxz);

    }
    inline bool operator==(BOX3D const& rhs) const
    {
        return equal(rhs);
    }

    /// Inequality operator
    inline bool operator!=(BOX3D const& rhs) const
    {
        return (!equal(rhs));
    }
    void grow(double x, double y, double z=mind)
    {
        if (x < minx) minx = x;
        if (x > maxx) maxx = x;

        if (y < miny) miny = y;
        if (y > maxy) maxy = y;

        if (z < minz) minz = z;
        if (z > maxz) maxz = z;

    }
    void grow(const BOX3D& other)
    {
        if (other.minx < minx) minx = other.minx;
        if (other.maxx > maxx) maxx = other.maxx;

        if (other.miny < miny) miny = other.miny;
        if (other.maxy > maxy) maxy = other.maxy;

        if (other.minz < minz) minz = other.minz;
        if (other.maxz > maxz) maxz = other.maxz;
    }

    void clip(double x, double y, double z)
    {
        if (x > minx) minx = x;
        if (x < maxx) maxx = x;

        if (y > maxy) miny = y;
        if (y < maxy) maxy = y;

        if (z > maxz) minz = z;
        if (z < maxz) maxz = z;
    }
    void clip(const BOX3D& other)
    {
        if (other.minx > minx) minx = other.minx;
        if (other.maxx < maxx) maxx = other.maxx;

        if (other.miny > miny) miny = other.miny;
        if (other.maxy < maxy) maxy = other.maxy;

        if (other.minz < minz) minz = other.minz;
        if (other.maxz > maxz) maxz = other.maxz;
    }

    bool is_z_empty() const
    {
        if (Utils::compare_distance<double>(minz, (std::numeric_limits<double>::max)()) &&
            Utils::compare_distance<double>(maxz, (std::numeric_limits<double>::min)()) )
            return true;
        return false;
    }

    bool overlaps(const BOX3D& other)
    {
        if (is_z_empty())
        {
            return minx <= other.maxx && maxx >= other.minx &&
                   miny <= other.maxy && maxy >= other.miny;
        }

        return minx <= other.maxx && maxx >= other.minx &&
               miny <= other.maxy && maxy >= other.miny &&
               minz <= other.maxz && maxz >= other.minz;
    }

    void clear()
    {
        minx = maxd; miny = maxd; minz = maxd;
        maxx = mind; maxy = maxd; maxz = maxd;
    }

    std::string toBox(uint32_t precision = 8, uint32_t dimensions=2) const
    {
        std::stringstream oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);


        if (dimensions  == 2)
        {
            oss << "box(";
            oss << minx << " " << miny << ", ";
            oss << maxx << " " << maxy << ")";
        }

        else if (dimensions == 3)
        {
            oss << "box3d(";
            oss << minx << " " << miny << " " << minz << ", ";
            oss << maxx << " " << maxy << " " << maxz << ")";
        }
        return oss.str();
    }


    std::string toWKT(uint32_t precision = 8) const
    {
        std::stringstream oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);

        oss << "POLYGON ((";

        oss << minx << " " << miny << ", ";
        oss << minx << " " << maxy << ", ";
        oss << maxx << " " << maxy << ", ";
        oss << maxx << " " << miny << ", ";
        oss << minx << " " << miny;

        // Nothing happens for 3D bounds.
        // else if (m_ranges.size() == 3 || (dimensions != 0 && dimensions == 3))
//         {
//             oss << minx << " " << miny << " " << getMaximum(2) << ", ";
//             oss << minx << " " << maxy << " " << getMaximum(2) << ", ";
//             oss << maxx << " " << maxy << " " << getMaximum(2) << ", ";
//             oss << maxx << " " << miny << " " << getMaximum(2) << ", ";
//             oss << minx << " " << miny << " " << getMaximum(2);
//         }
        oss << "))";
        return oss.str();
    }

    /** @name Default extent
    */
    /// Returns a staticly-allocated Bounds extent that represents infinity
    static const BOX3D& getDefaultSpatialExtent()
    {
        static BOX3D v(mind,mind,mind,maxd,maxd,maxd);
        return v;
    }


};

template <typename T>
class PDAL_DLL Bounds
{
public:
    typedef typename std::vector< Range<T> > RangeVector;

private:
    RangeVector m_ranges;

public:

    /** @name Constructors
    */

    /// Constructs an empty Bounds instance
    /// with no dimensions
    Bounds<T>()
    {
        m_ranges.resize(0);
    }

    /// Copy constructor
    Bounds(Bounds const& other)
        : m_ranges(other.m_ranges)
    {
    }

    /// Constructs a Bounds instance from a vector
    /// of Range
    Bounds(RangeVector const& ranges) : m_ranges(ranges)
    {
    }

    Bounds(size_t numDims)
        { m_ranges.resize(numDims); }

    /// Convenience constructor for typical 3D case
    Bounds(T minx,
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

    /// Convenience constructor for typical 2D case
    Bounds(T minx,
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

    /// Convenience constructor for Vector of minimum and maximum
    /// values.
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

    /// @name Properties

    /*! Gets the minimum value of the Range at the given index
        \param index the Range index to set the minimum value at
        \verbatim embed:rst
        .. note::

            If `index` is greater than :cpp:func:`size()`,
            a default value of ``0.0`` will be returned.
        \endverbatim
    */
    T getMinimum(std::size_t const& index) const
    {
        if (m_ranges.size() <= index)
        {
            return 0;
        }
        return m_ranges[index].getMinimum();
    }


    /// Sets the minimum value of the Range at the given index
    /// @param index the Range index to set the minimum value at
    /// @param v the value to set for the minimum
    void setMinimum(std::size_t const& index, T v)
    {
        if (m_ranges.size() <= index)
        {
            m_ranges.resize(index + 1);
        }
        m_ranges[index].setMinimum(v);
    }

    /*! Gets the maximum value of the Range at the given index
        \param index index the Range index to fetch the maximum value from.
        \verbatim embed:rst
        .. note::

            If `index` is greater than :cpp:func:`size()`,
            a default value of ``0.0`` will be returned.
        \endverbatim
    */
    T getMaximum(std::size_t const& index) const
    {
        if (m_ranges.size() <= index)
        {
            return 0;
        }
        return m_ranges[index].getMaximum();
    }

    /// Sets the maximum value of the Range at the given index
    /// @param index the Range index to set the maximum value at
    /// @param v the value to set for the maximum
    void setMaximum(std::size_t const& index, T v)
    {
        if (m_ranges.size() <= index)
        {
            m_ranges.resize(index + 1);
        }
        m_ranges[index].setMaximum(v);
    }

    /// A Vector of minimum values for each Range of the Bounds
    Vector<T> getMinimum()
    {
        std::vector<T> vec;

        for (std::size_t i=0; i<m_ranges.size(); i++)
        {
            vec.push_back(m_ranges[i].getMinimum());
        }

        return Vector<T>(vec);
    }

    /// A Vector of maximum values for each Range of the Bounds
    Vector<T> getMaximum()
    {
        std::vector<T> vec;

        for (std::size_t i=0; i<m_ranges.size(); i++)
        {
            vec.push_back(m_ranges[i].getMaximum());
        }

        return Vector<T>(vec);
    }

    /// The composite vector of Range<T> for the Bounds
    RangeVector const& dimensions() const
    {
        return m_ranges;
    }

    /// The number of dimensions of the Bounds
    std::size_t size() const
    {
        return m_ranges.size();
    }

    /// Calculate a n-dimensional volume for the Bounds instance

    /// Returns true if the pdal::Bounds<T>::size() is 0 or
    /// all dimensions within the bounds are empty.
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

    template <typename XFORMER>
    void transform(XFORMER xform)
    {
        xform(m_ranges[0].m_minimum, m_ranges[1].m_minimum,
            m_ranges[2].m_minimum);
        xform(m_ranges[0].m_maximum, m_ranges[1].m_maximum,
            m_ranges[2].m_maximum);
    }

    /// @name Equality

    /// Equality operator
    inline bool operator==(Bounds<T> const& rhs) const
    {
        return equal(rhs);
    }

    /// Inequality operator
    inline bool operator!=(Bounds<T> const& rhs) const
    {
        return (!equal(rhs));
    }

    /// Logical equality test -- used by == and != operators.
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

    /// @name Assignment

    /// Assignment operator
    Bounds<T>& operator=(Bounds<T> const& rhs)
    {
        if (&rhs != this)
        {
            m_ranges = rhs.m_ranges;
        }
        return *this;
    }

    /// @name Algebraic operations

//     /// Synonym for intersects for now
    bool overlaps(Bounds const& other) const
    {
        if (other.size() != size())
        {
            return false;
        }
//
        for (std::size_t i = 0; i < size(); i++)
        {
            if (m_ranges[i].overlaps(other.m_ranges[i]))
                return true;
        }
//
        return false;
    }
//
    /// Does this Bounds contain a point?
    bool contains(Vector<T> point) const
    {
        if (point.size() != size())
            return false;
//
        for (std::size_t i = 0; i < size(); i++)
        {
            // As soon as it is not contains, we're false
            if (! m_ranges[i].contains(point[i]))
                return false;
        }
        return true;
    }
//
    /// Does this Bounds contain other?
    bool contains(Bounds<T> const& other) const
    {
        for (std::size_t i = 0; i < size(); i++)
        {
            // As soon as it is not contains, we're false
            if (! m_ranges[i].contains(other.m_ranges[i]))
                return false;
        }
        return true;
    }
//
    /// @name Transformation
//
    /// Shift each dimension by a vector of deltas
    void shift(std::vector<T> deltas)
    {
        std::size_t i;
        if (size() != deltas.size())
        {
            std::ostringstream msg;
            msg << "pdal::Bounds::shift: Delta vector size, " << deltas.size()
                << ", is larger than the dimensionality of the bounds, "<< size() << ".";
            throw pdal::bounds_error(msg.str());
        }
        for (i = 0; i < deltas.size(); ++i)
        {
            m_ranges[i].shift(deltas[i]);
        }
    }
//
    /// Scale each dimension by a vector of deltas
    void scale(std::vector<T> deltas)
    {
        std::size_t i;
        if (size() != deltas.size())
        {
            std::ostringstream msg;
            msg << "pdal::Bounds::scale: Delta vector size, " << deltas.size()
                << ", is larger than the dimensionality of the bounds, "<< size() << ".";
            throw pdal::bounds_error(msg.str());
        }
        for (i = 0; i < deltas.size(); ++i)
        {
            m_ranges[i].scale(deltas[i]);
        }
    }
//
    /// Clip this Bounds to the extent of r
    void clip(Bounds const& r)
    {
        RangeVector ds = r.dimensions();
        for (std::size_t i = 0; i < size(); ++i)
        {
            m_ranges[i].clip(ds[i]);
        }
    }
//
    /// Grow to the union of two pdal::Bounds
    void grow(Bounds const& r)
    {
        RangeVector ds = r.dimensions();
        for (std::size_t i = 0; i < size(); ++i)
        {
            m_ranges[i].grow(ds[i]);
        }
    }
    void grow(double const& x, double const& y, double const& z)
    {

        m_ranges[0].grow(x);
        m_ranges[1].grow(y);
        if (size() == 3)
            m_ranges[2].grow(z);
    }

    /// Expand the pdal::Bounds to include this point
    void grow(Vector<T> const& point)
    {
        assert(point.size() == size());
        for (std::size_t i = 0; i < size(); ++i)
        {
            m_ranges[i].grow(point[i]);
        }
    }

    void clear()
    {
        for (size_t i = 0; i < m_ranges.size(); ++i)
            m_ranges[i].clear();
    }



    /// Verifies that the minimums and maximums of each dimension within the
    /// bounds are not inverted (allows min == +inf and max == -inf, however).
    bool verify()
    {
        for (std::size_t d = 0; d < size(); ++d)
        {
            if (getMinimum(d) > getMaximum(d))
            {

                // Allow infinity bounds
                if (!Utils::compare_distance<T>(getMinimum(d), (std::numeric_limits<T>::max)()) &&
                        !Utils::compare_distance<T>(getMaximum(d), -(std::numeric_limits<T>::min)()))
                {
                    std::ostringstream msg;
                    msg << "pdal::Bounds::verify: Minimum point at dimension " << d
                        << " is greater than maximum point.  Neither point is infinity."
                        << " min: " << getMinimum(d) << " max: " << getMaximum(d);
                    throw pdal::bounds_error(msg.str());
                }
            }
        }
        return true;
    }

    /** @name Default extent
    */
    /// Returns a staticly-allocated Bounds extent that represents infinity
    static const Bounds<T>& getDefaultSpatialExtent()
    {
        static T minv((std::numeric_limits<T>::min)());
        static T maxv((std::numeric_limits<T>::max)());
        static Bounds v(minv,minv,minv,maxv,maxv,maxv);
        return v;
    }

    /** @name Summary and serialization
    */
    /// Returns the Bounds<T> instance as OGC WKT
    /// @param precision the numerical precision to use for the output stream
    /// describing the number of decimal places each point in the WKT will have
    /// @param dimensions override the dimensionality of the WKT. Defaults to
    /// 0, and if not set, dimensionality of the WKT is determined by Bounds<T>::size()
    std::string toWKT(boost::uint32_t precision = 8) const
    {
        std::stringstream oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);

        oss << "POLYGON ((";

        oss << getMinimum(0) << " " << getMinimum(1) << ", ";
        oss << getMinimum(0) << " " << getMaximum(1) << ", ";
        oss << getMaximum(0) << " " << getMaximum(1) << ", ";
        oss << getMaximum(0) << " " << getMinimum(1) << ", ";
        oss << getMinimum(0) << " " << getMinimum(1);

        // Nothing happens for 3D bounds.
        // else if (m_ranges.size() == 3 || (dimensions != 0 && dimensions == 3))
//         {
//             oss << getMinimum(0) << " " << getMinimum(1) << " " << getMaximum(2) << ", ";
//             oss << getMinimum(0) << " " << getMaximum(1) << " " << getMaximum(2) << ", ";
//             oss << getMaximum(0) << " " << getMaximum(1) << " " << getMaximum(2) << ", ";
//             oss << getMaximum(0) << " " << getMinimum(1) << " " << getMaximum(2) << ", ";
//             oss << getMinimum(0) << " " << getMinimum(1) << " " << getMaximum(2);
//         }
        oss << "))";
        return oss.str();
    }

    std::string toBox(boost::uint32_t precision = 8, boost::uint32_t dimensions=2) const
    {
        std::stringstream oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);


        if (dimensions  == 2)
        {
            oss << "box(";
            oss << getMinimum(0) << " " << getMinimum(1) << ", ";
            oss << getMaximum(0) << " " << getMaximum(1) << ")";
        }

        else if (dimensions == 3)
        {
            oss << "box3d(";
            oss << getMinimum(0) << " " << getMinimum(1) << " " << getMinimum(2) << ", ";
            oss << getMaximum(0) << " " << getMaximum(1) << " " << getMaximum(2) << ")";
        }
        return oss.str();
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


extern PDAL_DLL std::istream& operator>>(std::istream& istr, Bounds<double>& bounds);

inline std::ostream& operator<<(std::ostream& ostr, const BOX3D& bounds)
{
    if (bounds.empty())
    {
        ostr << "()";
        return ostr;
    }

    ostr << "(";
    ostr << "[" << bounds.minx << ", " << bounds.maxx << "], "
         << "[" << bounds.miny << ", " << bounds.maxy <<"]";
    if (!bounds.is_z_empty())
         ostr << ", [" <<  bounds.minz << ", " << bounds.maxz << "]";
    ostr << ")";
    return ostr;
}
extern PDAL_DLL std::istream& operator>>(std::istream& istr, BOX3D& bounds);


} // namespace pdal

// Needed for C++ DLL exports
#ifdef PDAL_COMPILER_MSVC
template class PDAL_DLL pdal::Range<double>;
template class PDAL_DLL pdal::Bounds<double>;
#endif

#endif
