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


class PDAL_DLL BOX3D
{
private:
    static const double LOWEST;
    static const double HIGHEST;
    
public:

    BOX3D()
       { clear(); }

    BOX3D(const BOX3D& box) :
        minx(box.minx), maxx(box.maxx), miny(box.miny), maxy(box.maxy),
        minz(box.minz), maxz(box.maxz)
    {}

    // Note: a "2D" bbox is explicitly constructed to have invalid Z values,
    //   which might lead to some problems when used in conjuction with a "3D"
    //   bbox. See, for example, issue #897.
    // Note: ctor body not inlined due to MSVC, see #900
    BOX3D(double minx, double miny, double maxx, double maxy) ;

    BOX3D(double minx, double miny, double minz, double maxx, double maxy,
            double maxz) :
        minx(minx), maxx(maxx), miny(miny), maxy(maxy), minz(minz), maxz(maxz)
    {}

    double minx;
    double maxx;
    double miny;
    double maxy;
    double minz;
    double maxz;

    bool empty() const;

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
        return  minx == other.minx && maxx == other.maxx &&
            miny == other.miny && maxy == other.maxy &&
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

    void grow(double x, double y);

    void grow(double x, double y, double z);

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

    bool is_z_empty() const;

    bool overlaps(const BOX3D& other)
    {
        if (is_z_empty())
            return minx <= other.maxx && maxx >= other.minx &&
                   miny <= other.maxy && maxy >= other.miny;
        return minx <= other.maxx && maxx >= other.minx &&
               miny <= other.maxy && maxy >= other.miny &&
               minz <= other.maxz && maxz >= other.minz;
    }

    void clear();

    std::string toBox(uint32_t precision = 8, uint32_t dimensions = 2) const
    {
        std::stringstream oss;

        oss.precision(precision);
        oss.setf(std::ios_base::fixed, std::ios_base::floatfield);

        if (dimensions == 2)
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
        oss << "))";

        // Nothing happens for 3D bounds.

        return oss.str();
    }

    /// Returns a staticly-allocated Bounds extent that represents infinity
    static const BOX3D& getDefaultSpatialExtent();


};

inline std::ostream& operator << (std::ostream& ostr, const BOX3D& bounds)
{
    if (bounds.empty())
    {
        ostr << "()";
        return ostr;
    }

    const int savedPrec = ostr.precision();
    ostr.precision(16); // or..?
    ostr << "(";
    ostr << "[" << bounds.minx << ", " << bounds.maxx << "], "
         << "[" << bounds.miny << ", " << bounds.maxy <<"]";
    if (!bounds.is_z_empty())
         ostr << ", [" <<  bounds.minz << ", " << bounds.maxz << "]";
    ostr << ")";
    ostr.precision(savedPrec);
    return ostr;
}

extern PDAL_DLL std::istream& operator>>(std::istream& istr, BOX3D& bounds);

} // namespace pdal
