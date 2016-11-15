/******************************************************************************
* Copyright (c) 2016, Howard Butler (howard@hobu.co)
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

#include <pdal/pdal_internal.hpp>
#include <iosfwd>
#include <limits>

namespace pdal
{


namespace cropfilter
{

class PDAL_DLL Point2D
{
public:
    double x;
    double y;
    Point2D(double x_, double y_) : x(x_), y(y_) {};
    Point2D() { clear(); }
    Point2D(const Point2D& point) : x(point.x), y(point.y) {};

    Point2D to2d() const
    {
        return *this;
    }
    bool empty() const;

    void clear();

};

class PDAL_DLL Point3D : private Point2D
{
public:
    using Point2D::x;
    using Point2D::y;
    double z;
    Point3D(double x_, double y_, double z_) : Point2D(x_, y_), z(z_) {};
    Point3D() { clear(); }
    Point3D(const Point3D& point) :
        Point2D(point.x, point.y), z (point.z)
    {}
    explicit Point3D(const Point2D& point) :
        Point2D(point), z(0)
    {}

    Point2D to2d() const;
    Point3D to3d() const;
    bool empty() const;

    void clear();
};

class PDAL_DLL Point
{
public:
    Point()
    {};

    Point(const Point3D& box);
    Point(const Point2D& box);

    Point3D to3d() const;
    Point2D to2d() const;
    bool is3d() const;
    bool empty() const;

    friend PDAL_DLL std::istream& operator >> (std::istream& in,
        Point& point);
    friend PDAL_DLL std::ostream& operator << (std::ostream& out,
        const Point& point);

private:
    Point3D m_point;

    void set(const Point3D& box);
    void set(const Point2D& box);
};

inline std::ostream& operator << (std::ostream& ostr, const cropfilter::Point2D& point)
{
    auto savedPrec = ostr.precision();
    ostr.precision(16);
    ostr << "(";
    ostr << point.x << ", " << point.y;
    ostr << ")";
    ostr.precision(savedPrec);
    return ostr;
}

inline std::ostream& operator << (std::ostream& ostr, const cropfilter::Point3D& point)
{

    auto savedPrec = ostr.precision();
    ostr.precision(16);
    ostr << "(";
    ostr << point.x << ", " << point.y << ", " << point.z;
    ostr << ")";
    ostr.precision(savedPrec);
    return ostr;
}

/*
  Read a Point2D from a stream in a format provided by PDAL options.

  \param istr  Stream to read from.
  \param point Point box to populate.
*/
extern PDAL_DLL std::istream& operator>>(std::istream& istr, cropfilter::Point2D& point);

/**
  Read a Point3D from a stream in a format provided by PDAL options.

  \param istr  Stream to read from.
  \param bounds  Point box to populate.
*/
extern PDAL_DLL std::istream& operator>>(std::istream& istr, cropfilter::Point3D& point);

PDAL_DLL std::istream& operator >> (std::istream& in, cropfilter::Point& point);
// PDAL_DLL std::ostream& operator << (std::ostream& out, cropfilter::Point& point);


} // namespace cropfilter
} // namespace pdal
