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

#ifndef PDAL_DLL
#if defined(_WIN32)
#   define PDAL_DLL   __declspec(dllexport)
#else
#  if defined(USE_GCC_VISIBILITY_FLAG)
#    define PDAL_DLL     __attribute__ ((visibility("default")))
#  else
#    define PDAL_DLL
#  endif
#endif
#endif

namespace pdal
{

/*!
    \verbatim embed:rst

    Bounds is for manipulating n-dimensional ranges of data.  Typically
    used for defining the spatial extents of XYZ data, this class can also be
    used for defining bounds of other dimensions.

    \endverbatim
*/

class PDAL_DLL BOX2D
{
protected:
    static const double LOWEST;
    static const double HIGHEST;

public:
    double minx;
    double maxx;
    double miny;
    double maxy;

    BOX2D()
        { clear(); }

    BOX2D(const BOX2D& box) :
        minx(box.minx), maxx(box.maxx), miny(box.miny), maxy(box.maxy)
    {}

    BOX2D(double minx, double miny, double maxx, double maxy) :
        minx(minx), maxx(maxx), miny(miny), maxy(maxy)
    {}

    bool empty() const;
    void clear();
    void grow(double x, double y);

    bool contains(double x, double y) const
        { return minx <= x && x <= maxx && miny <= y && y <= maxy; }

    bool contains(const BOX2D& other) const
    {
        return minx <= other.minx && other.maxx <= maxx &&
            miny <= other.miny && other.maxy <= maxy;
    }

    bool equal(const BOX2D& other) const
    {
        return  minx == other.minx && maxx == other.maxx &&
            miny == other.miny && maxy == other.maxy;
    }

    bool operator==(BOX2D const& rhs) const
    {
        return equal(rhs);
    }

    bool operator!=(BOX2D const& rhs) const
    {
        return (!equal(rhs));
    }

    void grow(const BOX2D& other)
    {
        if (other.minx < minx) minx = other.minx;
        if (other.maxx > maxx) maxx = other.maxx;

        if (other.miny < miny) miny = other.miny;
        if (other.maxy > maxy) maxy = other.maxy;
    }

    void clip(double x, double y)
    {
        if (x > minx) minx = x;
        if (x < maxx) maxx = x;

        if (y > maxy) miny = y;
        if (y < maxy) maxy = y;
    }

    void clip(const BOX2D& other)
    {
        if (other.minx > minx) minx = other.minx;
        if (other.maxx < maxx) maxx = other.maxx;

        if (other.miny > miny) miny = other.miny;
        if (other.maxy < maxy) maxy = other.maxy;
    }

    bool overlaps(const BOX2D& other)
    {
        return minx <= other.maxx && maxx >= other.minx &&
            miny <= other.maxy && maxy >= other.miny;
    }

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
        oss << "))";

        return oss.str();
    }

    /// Returns a staticly-allocated Bounds extent that represents infinity
    static const BOX2D& getDefaultSpatialExtent();
};

class PDAL_DLL BOX3D : private BOX2D
{
public:
    using BOX2D::minx;
    using BOX2D::maxx;
    using BOX2D::miny;
    using BOX2D::maxy;
    double minz;
    double maxz;

    BOX3D()
       { clear(); }

    BOX3D(const BOX3D& box) :
        BOX2D(box), minz(box.minz), maxz(box.maxz)
    {}

    BOX3D(double minx, double miny, double minz, double maxx, double maxy,
        double maxz) : BOX2D(minx, miny, maxx, maxy), minz(minz), maxz(maxz)
    {}


    bool empty() const;
    void grow(double x, double y, double z);
    void clear();

    bool contains(double x, double y, double z) const
    {
        return BOX2D::contains(x, y) && minz <= z && z <= maxz;
    }

    bool contains(const BOX3D& other) const
    {
        return BOX2D::contains(other) &&
            minz <= other.minz && other.maxz <= maxz;
    }

    bool equal(const BOX3D& other) const
    {
        return  BOX2D::contains(other) &&
            minz == other.minz && maxz == other.maxz;
    }

    bool operator==(BOX3D const& rhs) const
    {
        return equal(rhs);
    }

    bool operator!=(BOX3D const& rhs) const
    {
        return (!equal(rhs));
    }

    void grow(const BOX3D& other)
    {
        BOX2D::grow(other);
        if (other.minz < minz) minz = other.minz;
        if (other.maxz > maxz) maxz = other.maxz;
    }

    void clip(double x, double y, double z)
    {
        BOX2D::clip(x, y);
        if (z > maxz) minz = z;
        if (z < maxz) maxz = z;
    }

    void clip(const BOX3D& other)
    {
        BOX2D::clip(other);
        if (other.minz < minz) minz = other.minz;
        if (other.maxz > maxz) maxz = other.maxz;
    }

    bool overlaps(const BOX3D& other)
    {
        return BOX2D::overlaps(other) &&
           minz <= other.maxz && maxz >= other.minz;
    }

    BOX2D to2d() const
    {
        return *this;
    }
    std::string toBox(uint32_t precision = 8) const
    {
        std::stringstream oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);

        oss << "box3d(" << minx << " " << miny << " " << minz << ", " <<
            maxx << " " << maxy << " " << maxz << ")";
        return oss.str();
    }

    std::string toWKT(uint32_t precision = 8) const
    {
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

    /// Returns a staticly-allocated Bounds extent that represents infinity
    static const BOX3D& getDefaultSpatialExtent();
};

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

extern PDAL_DLL std::istream& operator>>(std::istream& istr, BOX2D& bounds);
extern PDAL_DLL std::istream& operator>>(std::istream& istr, BOX3D& bounds);

} // namespace pdal
