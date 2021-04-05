/******************************************************************************
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

#pragma once

#include <cstdint>
#include <sstream>
#include <stdexcept>

#include "pdal_util_export.hpp"

namespace pdal
{

/**
  BOX2D represents a two-dimensional box with double-precision bounds.
*/
class PDAL_DLL BOX2D
{
public:
    struct error : public std::runtime_error
    {
        error(const std::string& err) : std::runtime_error(err)
        {}
    };

    double minx;  ///< Minimum X value.
    double maxx;  ///< Maximum X value.
    double miny;  ///< Minimum Y value.
    double maxy;  ///< Maximum Y value.

    /**
      Construct an "empty" bounds box.
    */
    BOX2D()
        { clear(); }

    /**
      Construct and initialize a bounds box.

      \param minx  Minimum X value.
      \param miny  Minimum Y value.
      \param maxx  Maximum X value.
      \param maxy  Maximum Y value.
    */
    BOX2D(double minx, double miny, double maxx, double maxy) :
        minx(minx), maxx(maxx), miny(miny), maxy(maxy)
    {}

    /**
      Determine whether a bounds box has not had any bounds set.

      \return  Whether the bounds box is empty.
    */
    bool empty() const;

    /**
      Determine whether a bounds box has had any bounds set.

      \return  Whether the bounds box is valid.
    */
    bool valid() const;

    /**
      Clear the bounds box to an empty state.
    */
    void clear();

    /**
      Expand the bounds of the box to include the specified point.

      \param x  X point location.
      \param y  Y point location.
    */
    BOX2D& grow(double x, double y);

    /**
      Expand the bounds of the box in all directions by a specified amount.

      \param dist  Distance by which to expand the box.
    */
    BOX2D& grow(double dist);

    /**
      Determine if a bounds box contains a point.

      \param x  X dimension value.
      \param y  Y dimension value.
      \return  Whether both dimensions are equal to or less than the maximum
        box values and equal to or more than the minimum box values.
    */
    bool contains(double x, double y) const
        { return minx <= x && x <= maxx && miny <= y && y <= maxy; }

    /**
      Determine if the bounds of this box are the same as that of another
      box.  Empty bounds boxes are always equal.

      \param other  Bounds box to check for equality.
      \return \c true if the provided box has equal limits to this box,
        \c false otherwise.
    */
    bool equal(const BOX2D& other) const
    {
        return  minx == other.minx && maxx == other.maxx &&
            miny == other.miny && maxy == other.maxy;
    }

    /**
      Determine if the bounds of this box are the same as that of another
      box.  Empty bounds boxes are always equal.

      \param other  Bounds box to check for equality.
      \return \c true if the provided box has equal limits to this box,
        \c false otherwise.
    */
    bool operator==(BOX2D const& other) const
    {
        return equal(other);
    }

    /**
      Determine if the bounds of this box are different from that of another
      box.  Empty bounds boxes are never unequal.

      \param other  Bounds box to check for inequality.
      \return \c true if the provided box has limits different from this box,
        \c false otherwise.
    */
    bool operator!=(BOX2D const& other) const
    {
        return (!equal(other));
    }

    /**
      Expand this box to contain another box.

      \param other  Box that this box should contain.
    */
    BOX2D& grow(const BOX2D& other)
    {
        if (other.minx < minx) minx = other.minx;
        if (other.maxx > maxx) maxx = other.maxx;

        if (other.miny < miny) miny = other.miny;
        if (other.maxy > maxy) maxy = other.maxy;
        return *this;
    }

    /**
      Clip this bounds box by another so it will be contained by the
      other box.

      \param other  Clipping box for this box.
    */
    void clip(const BOX2D& other)
    {
        if (other.minx > minx) minx = other.minx;
        if (other.maxx < maxx) maxx = other.maxx;

        if (other.miny > miny) miny = other.miny;
        if (other.maxy < maxy) maxy = other.maxy;
    }

    /**
      Determine if another bounds box is contained in this bounds box.
      Equal limits are considered to be contained.

      \param other  Bounds box to check for containment.
      \return  \c true if the provided box is contained in this box,
        \c false otherwise.
    **/
    bool contains(const BOX2D& other) const
    {
        return minx <= other.minx && maxx >= other.maxx &&
            miny <= other.miny && maxy >= other.maxy;
    }

    /**
      Determine if another box overlaps this box.

      \param other  Box to test for overlap.
      \return  Whether the provided box overlaps this box.
    */
    bool overlaps(const BOX2D& other) const
    {
        return minx <= other.maxx && maxx >= other.minx &&
            miny <= other.maxy && maxy >= other.miny;
    }

    /**
      Convert this box to a string suitable for use in SQLite.

      \param precision  Precision for output [default: 8]
      \return  String format of this box.
    */
    std::string toBox(uint32_t precision = 8) const
    {
        std::stringstream oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);

        oss << "box2d(";
            oss << minx << " " << miny << ", ";
            oss << maxx << " " << maxy << ")";
        return oss.str();
    }

    /**
      Convert this box to a well-known text string.

      \param precision  Precision for output [default: 8]
      \return  String format of this box.
    */
    std::string toWKT(uint32_t precision = 8) const
    {
        if (empty())
            return std::string();

        std::stringstream oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);

        oss << "POLYGON ((";
        oss << minx << " " << miny << ", ";
        oss << minx << " " << maxy << ", ";
        oss << maxx << " " << maxy << ", ";
        oss << maxx << " " << miny << ", ";
        oss << minx << " " << miny;
        oss << "))";

        return oss.str();
    }

    /**
      Convert this box to a GeoJSON text string.

      \param precision  Precision for output [default: 8]
      \return  String format of this box.
    */
    std::string toGeoJSON(uint32_t precision = 8) const
    {
        if (empty())
            return std::string();

        std::stringstream oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);
        oss << "{\"bbox\":[" << minx << ", " << miny << ", " <<
            maxx <<  "," << maxy << "]}";
        return oss.str();
    }

    /**
      Return a statically-allocated Bounds extent that represents infinity

      \return  A bounds box with infinite bounds,
    */
    static const BOX2D& getDefaultSpatialExtent();

    /**
      Parse a string as a BOX2D.

      \param s    String representation of the box.
      \param pos  Position in the string at which to start parsing.
                  On return set to parsing end position.
    */
    void parse(const std::string& s, std::string::size_type& pos);
};


/**
  BOX3D represents a three-dimensional box with double-precision bounds.
*/
class PDAL_DLL BOX3D : private BOX2D
{
public:
    struct error : public std::runtime_error
    {
        error(const std::string& err) : std::runtime_error(err)
        {}
    };

    using BOX2D::minx;
    using BOX2D::maxx;
    using BOX2D::miny;
    using BOX2D::maxy;
    double minz;   ///< Minimum Z value.
    double maxz;   ///< Maximum Z value.

    /**
      Clear the bounds box to an empty state.
    */
    BOX3D()
       { clear(); }

    BOX3D(const BOX3D& box) :
        BOX2D(box), minz(box.minz), maxz(box.maxz)
    {}

    BOX3D& operator=(const BOX3D& box) = default;

    explicit BOX3D(const BOX2D& box) :
        BOX2D(box), minz(0), maxz(0)
    {}

    /**
      Construct and initialize a bounds box.

      \param minx  Minimum X value.
      \param miny  Minimum Y value.
      \param minx  Minimum Z value.
      \param maxx  Maximum X value.
      \param maxy  Maximum Y value.
      \param maxz  Maximum Z value.
    */
    BOX3D(double minx, double miny, double minz, double maxx, double maxy,
        double maxz) : BOX2D(minx, miny, maxx, maxy), minz(minz), maxz(maxz)
    {}

    /**
      Determine whether a bounds box has not had any bounds set (is in a state
      as if default-constructed).

      \return  Whether the bounds box is empty.
    */
    bool empty() const;

    /**
      Determine whether a bounds box has had any bounds set.

      \return  \true if the bounds box is not empty
    */
    bool valid() const;

    /**
      Expand the bounds of the box if a value is less than the current
      minimum or greater than the current maximum.  If the bounds box is
      currently empty, both minimum and maximum box bounds will be set to
      the provided value.

      \param x  X dimension value.
      \param y  Y dimension value.
      \param z  Z dimension value.
    */
    BOX3D& grow(double x, double y, double z);

    /**
      Clear the bounds box to an empty state.
    */
    void clear();


    /**
      Determine if a bounds box contains a point.

      \param x  X dimension value.
      \param y  Y dimension value.
      \param z  Z dimension value.
      \return  Whether both dimensions are equal to or less than the maximum
        box values and equal to or more than the minimum box values.
    */
    bool contains(double x, double y, double z) const
    {
        return BOX2D::contains(x, y) && minz <= z && z <= maxz;
    }

    /**
      Determine if another bounds box is contained in this bounds box.
      Equal limits are considered to be contained.

      \param other  Bounds box to check for containment.
      \return  \c true if the provided box is contained in this box,
        \c false otherwise.
    **/
    bool contains(const BOX3D& other) const
    {
        return BOX2D::contains(other) &&
            minz <= other.minz && other.maxz <= maxz;
    }

    /**
      Determine if the bounds of this box are the same as that of another
      box.  Empty bounds boxes are always equal.

      \param other  Bounds box to check for equality.
      \return \c true if the provided box has equal limits to this box,
        \c false otherwise.
    */
    bool equal(const BOX3D& other) const
    {
        return  BOX2D::contains(other) &&
            minz == other.minz && maxz == other.maxz;
    }

    /**
      Determine if the bounds of this box are the same as that of another
      box.  Empty bounds boxes are always equal.

      \param other  Bounds box to check for equality.
      \return \c true if the provided box has equal limits to this box,
        \c false otherwise.
    */
    bool operator==(BOX3D const& rhs) const
    {
        return equal(rhs);
    }

    /**
      Determine if the bounds of this box are different from that of another
      box.  Empty bounds boxes are never unequal.

      \param other  Bounds box to check for inequality.
      \return \c true if the provided box has limits different from this box,
        \c false otherwise.
    */
    bool operator!=(BOX3D const& rhs) const
    {
        return (!equal(rhs));
    }

    /**
      Expand this box to contain another box.

      \param other  Box that this box should contain.
    */
    BOX3D& grow(const BOX3D& other)
    {
        BOX2D::grow(other);
        if (other.minz < minz) minz = other.minz;
        if (other.maxz > maxz) maxz = other.maxz;
        return *this;
    }

    /**
      Expand this box by a specified amount.

      \param dist  Distance by which box should be expanded.
    */
    BOX3D& grow(double dist)
    {
        BOX2D::grow(dist);
        minz -= dist;
        maxz += dist;
        return *this;
    }

    /**
      Clip this bounds box by another so it will be contained by the
      other box.

      \param other  Clipping box for this box.
    */
    void clip(const BOX3D& other)
    {
        BOX2D::clip(other);
        if (other.minz < minz) minz = other.minz;
        if (other.maxz > maxz) maxz = other.maxz;
    }

    /**
      Determine if another box overlaps this box.

      \param other  Box to test for overlap.
      \return  Whether the provided box overlaps this box.
    */
    bool overlaps(const BOX3D& other) const
    {
        return BOX2D::overlaps(other) &&
           minz <= other.maxz && maxz >= other.minz;
    }

    /**
      Convert this box to 2-dimensional bounding box.

      \return  Bounding box with Z dimension stripped.
    */
    BOX2D to2d() const
    {
        return *this;
    }

    /**
      Convert this box to a string suitable for use in SQLite.

      \param precision  Precision for output [default: 8]
      \return  String format of this box.
    */
    std::string toBox(uint32_t precision = 8) const
    {
        std::stringstream oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);

        oss << "box3d(" << minx << " " << miny << " " << minz << ", " <<
            maxx << " " << maxy << " " << maxz << ")";
        return oss.str();
    }

    /**
      Convert this box to a well-known text string.

      \param precision  Precision for output [default: 8]
      \return  String format of this box.
    */
    std::string toWKT(uint32_t precision = 8) const
    {
        if (empty())
            return std::string();

        std::stringstream oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);

        oss << "POLYHEDRON Z ( ";

        oss << "((" << minx << " " << miny << " " << minz << ", " <<
                       maxx << " " << miny << " " << minz << ", " <<
                       maxx << " " << maxy << " " << minz << ", " <<
                       minx << " " << maxy << " " << minz << ", " <<
                       minx << " " << miny << " " << minz << ", " <<
               ")), ";
        oss << "((" << minx << " " << miny << " " << minz << ", " <<
                       maxx << " " << miny << " " << minz << ", " <<
                       maxx << " " << miny << " " << maxz << ", " <<
                       minx << " " << miny << " " << maxz << ", " <<
                       minx << " " << miny << " " << minz << ", " <<
               ")), ";
        oss << "((" << maxx << " " << miny << " " << minz << ", " <<
                       maxx << " " << maxy << " " << minz << ", " <<
                       maxx << " " << maxy << " " << maxz << ", " <<
                       maxx << " " << miny << " " << maxz << ", " <<
                       maxx << " " << miny << " " << minz << ", " <<
               ")), ";
        oss << "((" << maxx << " " << maxy << " " << minz << ", " <<
                       minx << " " << maxy << " " << minz << ", " <<
                       minx << " " << maxy << " " << maxz << ", " <<
                       maxx << " " << maxy << " " << maxz << ", " <<
                       maxx << " " << maxy << " " << minz << ", " <<
               ")), ";
        oss << "((" << minx << " " << maxy << " " << minz << ", " <<
                       minx << " " << miny << " " << minz << ", " <<
                       minx << " " << miny << " " << maxz << ", " <<
                       minx << " " << maxy << " " << maxz << ", " <<
                       minx << " " << maxy << " " << minz << ", " <<
               ")), ";
        oss << "((" << minx << " " << miny << " " << maxz << ", " <<
                       maxx << " " << miny << " " << maxz << ", " <<
                       maxx << " " << maxy << " " << maxz << ", " <<
                       minx << " " << maxy << " " << maxz << ", " <<
                       minx << " " << miny << " " << maxz << ", " <<
               "))";

        oss << " )";

        return oss.str();
    }

    /**
      Return a statically-allocated Bounds extent that represents infinity

      \return  A bounds box with infinite bounds,
    */
    static const BOX3D& getDefaultSpatialExtent();

    /**
      Parse a string as a BOX3D.

      \param s    String representation of the box.
      \param pos  Position in the string at which to start parsing.
                  On return set to parsing end position.
    */
    void parse(const std::string& s, std::string::size_type& pos);
};

/**
  Wrapper for BOX3D and BOX2D to allow extraction as either.  Typically used
  to facilitate streaming either a BOX2D or BOX3D
*/
class PDAL_DLL Bounds
{
public:
    struct error : public std::runtime_error
    {
        error(const std::string& err) : std::runtime_error(err)
        {}
    };

    Bounds()
    {}

    explicit Bounds(const BOX3D& box);
    explicit Bounds(const BOX2D& box);

    BOX3D to3d() const;
    BOX2D to2d() const;
    bool is3d() const;
    void reset(const BOX3D& box);
    void reset(const BOX2D& box);
    void grow(double x, double y);
    void grow(double x, double y, double z);
    void parse(const std::string& s, std::string::size_type& pos);

    friend PDAL_DLL std::istream& operator >> (std::istream& in,
        Bounds& bounds);
    friend PDAL_DLL std::ostream& operator << (std::ostream& out,
        const Bounds& bounds);

private:
    BOX3D m_box;

    void set(const BOX3D& box);
    void set(const BOX2D& box);
};

/**
  Write a 2D bounds box to a stream in a format used by PDAL options.

  \param ostr  Stream to write to.
  \param bounds  Box to write.
*/
inline std::ostream& operator << (std::ostream& ostr, const BOX2D& bounds)
{
    if (bounds.empty())
    {
        ostr << "()";
        return ostr;
    }

    auto savedPrec = ostr.precision();
    ostr.precision(16); // or..?
    ostr << "(";
    ostr << "[" << bounds.minx << ", " << bounds.maxx << "], " <<
            "[" << bounds.miny << ", " << bounds.maxy << "]";
    ostr << ")";
    ostr.precision(savedPrec);
    return ostr;
}

/**
  Write a 3D bounds box to a stream in a format used by PDAL options.

  \param ostr  Stream to write to.
  \param bounds  Box to write.
*/
inline std::ostream& operator << (std::ostream& ostr, const BOX3D& bounds)
{
    if (bounds.empty())
    {
        ostr << "()";
        return ostr;
    }

    auto savedPrec = ostr.precision();
    ostr.precision(16); // or..?
    ostr << "(";
    ostr << "[" << bounds.minx << ", " << bounds.maxx << "], " <<
            "[" << bounds.miny << ", " << bounds.maxy << "], " <<
            "[" << bounds.minz << ", " << bounds.maxz << "]";
    ostr << ")";
    ostr.precision(savedPrec);
    return ostr;
}

/**
  Read a 2D bounds box from a stream in a format provided by PDAL options.

  \param istr  Stream to read from.
  \param bounds  Bounds box to populate.
*/
extern PDAL_DLL std::istream& operator>>(std::istream& istr, BOX2D& bounds);

/**
  Read a 3D bounds box from a stream in a format provided by PDAL options.

  \param istr  Stream to read from.
  \param bounds  Bounds box to populate.
*/
extern PDAL_DLL std::istream& operator>>(std::istream& istr, BOX3D& bounds);

PDAL_DLL std::istream& operator >> (std::istream& in, Bounds& bounds);
PDAL_DLL std::ostream& operator << (std::ostream& out, const Bounds& bounds);

} // namespace pdal
