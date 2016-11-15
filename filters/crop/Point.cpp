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

#include "Point.hpp"

namespace pdal
{

namespace
{

const double LOWEST = (std::numeric_limits<double>::lowest)();
const double HIGHEST = (std::numeric_limits<double>::max)();

}

template <typename PREDICATE>
void eat(std::istream& in, PREDICATE p)
{
    while (p((char)in.get()))
        ;
    if (in.eof())
        in.clear(in.rdstate() & ~std::ios::failbit);
    else
        in.unget();
}

bool eat(std::istream& in, char c)
{
    if ((char)in.get() == c)
        return true;
    in.unget();
    return false;
}

void readxy(std::istream& istr, double& x, double& y)
{
    eat(istr, isspace);
    if (!eat(istr,'('))
        istr.setstate(std::ios_base::failbit);

    eat(istr, isspace);
    istr >> x;

    eat(istr, isspace);
    if (!eat(istr,','))
        istr.setstate(std::ios_base::failbit);

    eat(istr, isspace);
    istr >> y;

    eat(istr, isspace);
    if (!eat(istr,')'))
        istr.setstate(std::ios_base::failbit);
}

void readxyz(std::istream& istr, double& x, double& y, double& z)
{
    eat(istr, isspace);
    if (!eat(istr,'('))
        istr.setstate(std::ios_base::failbit);

    eat(istr, isspace);
    istr >> x;

    eat(istr, isspace);
    if (!eat(istr,','))
        istr.setstate(std::ios_base::failbit);

    eat(istr, isspace);
    istr >> y;

    eat(istr, isspace);
    if (!eat(istr,','))
        istr.setstate(std::ios_base::failbit);

    eat(istr, isspace);
    istr >> z;

    eat(istr, isspace);
    if (!eat(istr,')'))
        istr.setstate(std::ios_base::failbit);
}



namespace cropfilter
{

bool Point2D::empty() const
{
    return  x == LOWEST && y == LOWEST;
}

void Point2D::clear()
{
    x = LOWEST; y = LOWEST;
}

void Point3D::clear()
{
    Point2D::clear();
    z = LOWEST;
}

bool Point3D::empty() const
{
    return  x == LOWEST && y == LOWEST && z == LOWEST;
}

Point::Point(const Point3D& point) : m_point(point)
{}


Point::Point(const Point2D& point) : m_point(point)
{
    m_point.z = LOWEST;
}

bool Point::is3d() const
{
    return (m_point.z != LOWEST );
}


void Point::set(const Point3D& point)
{
    m_point = point;
}


void Point::set(const Point2D& point)
{
    m_point = Point3D(point);
    m_point.z = LOWEST;
}

Point2D Point::to2d() const
{
    return m_point.to2d();
}

Point2D Point3D::to2d() const
{
    return (*this);
}

Point3D Point::to3d() const
{
    if (m_point.x == LOWEST && m_point.y == LOWEST)
        return Point3D();
    return m_point;
}


std::istream& operator>>(std::istream& istr, cropfilter::Point2D& point)
{
    char left_paren = (char)istr.get();
    if (!istr.good())
    {
        istr.setstate(std::ios_base::failbit);
        return istr;
    }
    const char right_paren = (char)istr.get();

    if (left_paren == '(' && right_paren == ')')
    {
        point = cropfilter::Point2D();
        return istr;
    }
    istr.unget();
    istr.unget(); // ()

    double x, y;

    readxy(istr, x, y);

    if (istr.good())
    {
        point.x = x;
        point.y = y;
    }
    return istr;
}

std::istream& operator>>(std::istream& istr, cropfilter::Point3D& point)
{
    char left_paren = (char)istr.get();
    if (!istr.good())
    {
        istr.setstate(std::ios_base::failbit);
        return istr;
    }
    const char right_paren = (char)istr.get();

    if (left_paren == '(' && right_paren == ')')
    {
        cropfilter::Point3D output;
        point = output;
        return istr;
    }
    istr.unget();
    istr.unget(); // ()

    double x, y, z;

    readxyz(istr, x, y, z);
    if (istr.good())
    {
        point.x = x;
        point.y = y;
        point.z = z;
    }
    return istr;
}




std::istream& operator>>(std::istream& in, cropfilter::Point& point)
{
    std::streampos start = in.tellg();
    cropfilter::Point3D b3d;
    in >> b3d;
    if (in.fail())
    {
        in.clear();
        in.seekg(start);
        cropfilter::Point2D b2d;
        in >> b2d;
        if (!in.fail())
            point.set(b2d);
    }
    else
        point.set(b3d);
    return in;
}

std::ostream& operator<<(std::ostream& out, const cropfilter::Point& point)
{
    if (point.is3d())
        out << point.to3d();
    else
        out << point.to2d();
    return out;
}



} //namespace cropfilter

} //namespace pdal

