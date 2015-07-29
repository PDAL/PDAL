/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <iostream>
#include <limits>
#include <vector>

#include <pdal/util/Bounds.hpp>

namespace
{

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

void readpair(std::istream& istr, double& low, double& high)
{
    eat(istr, isspace);
    if (!eat(istr,'['))
        istr.setstate(std::ios_base::failbit);

    eat(istr, isspace);
    istr >> low;

    eat(istr, isspace);
    if (!eat(istr,','))
        istr.setstate(std::ios_base::failbit);

    eat(istr, isspace);
    istr >> high;

    if (!eat(istr,']'))
        istr.setstate(std::ios_base::failbit);
}

} // unnamed namespace

namespace pdal
{

const double BOX2D::LOWEST = (std::numeric_limits<double>::lowest)();
const double BOX2D::HIGHEST = (std::numeric_limits<double>::max)();
    
void BOX2D::clear()
{
    minx = HIGHEST; miny = HIGHEST;
    maxx = LOWEST; maxy = LOWEST;
}

void BOX3D::clear()
{
    BOX2D::clear();
    minz = HIGHEST;
    maxz = LOWEST;
}

bool BOX2D::empty() const
{
    return  minx == HIGHEST && maxx == LOWEST &&
        miny == HIGHEST && maxy == LOWEST;
}

bool BOX3D::empty() const
{
    return  BOX2D::empty() && minz == HIGHEST && maxz == LOWEST;
}

void BOX2D::grow(double x, double y)
{
    if (x < minx) minx = x;
    if (x > maxx) maxx = x;

    if (y < miny) miny = y;
    if (y > maxy) maxy = y;
}

void BOX3D::grow(double x, double y, double z)
{
    BOX2D::grow(x, y);
    if (z < minz) minz = z;
    if (z > maxz) maxz = z;
}

const BOX2D& BOX2D::getDefaultSpatialExtent()
{
    static BOX2D v(LOWEST, LOWEST, HIGHEST, HIGHEST);
    return v;
}    


const BOX3D& BOX3D::getDefaultSpatialExtent()
{
    static BOX3D v(LOWEST, LOWEST, LOWEST, HIGHEST, HIGHEST, HIGHEST);
    return v;
}    


std::istream& operator>>(std::istream& istr, BOX2D& bounds)
{
    //ABELL - Not sure the point of this.  I get that one can have an "empty"
    // BOX2D, but when would it be useful to create one from a string?
    // A really dirty way to check for an empty bounds object right off
    // the bat
    char left_paren = (char)istr.get();
    if (!istr.good())
    {
        istr.setstate(std::ios_base::failbit);
        return istr;
    }
    const char right_paren = (char)istr.get();

    if (left_paren == '(' && right_paren == ')')
    {
        bounds = BOX2D();
        return istr;
    }
    istr.unget();
    istr.unget(); // ()

    std::vector<double> v;

    eat(istr, isspace);
    if (!eat(istr,'('))
        istr.setstate(std::ios_base::failbit);

    bool done = false;
    for (int i = 0; i < 2; ++i)
    {
        double low, high;

        readpair(istr, low, high);

        eat(istr, isspace);
        if (!eat(istr, i == 1 ? ')' : ','))
            istr.setstate(std::ios_base::failbit);
        v.push_back(low);
        v.push_back(high);
    }

    if (istr.good())
    {
        bounds.minx = v[0];
        bounds.maxx = v[1];
        bounds.miny = v[2];
        bounds.maxy = v[3];
    }
    return istr;
}

std::istream& operator>>(std::istream& istr, BOX3D& bounds)
{
    //ABELL - Not sure the point of this.  I get that one can have an "empty"
    // BOX3D, but when would it be useful to create one from a string?
    // A really dirty way to check for an empty bounds object right off
    // the bat
    char left_paren = (char)istr.get();
    if (!istr.good())
    {
        istr.setstate(std::ios_base::failbit);
        return istr;
    }
    const char right_paren = (char)istr.get();

    if (left_paren == '(' && right_paren == ')')
    {
        BOX3D output;
        bounds = output;
        return istr;
    }
    istr.unget();
    istr.unget(); // ()

    std::vector<double> v;

    eat(istr, isspace);
    if (!eat(istr,'('))
        istr.setstate(std::ios_base::failbit);

    bool done = false;
    for (int i = 0; i < 3; ++i)
    {
        double low, high;

        readpair(istr, low, high);

        eat(istr, isspace);
        if (!eat(istr, i == 2 ? ')' : ','))
            istr.setstate(std::ios_base::failbit);
        v.push_back(low);
        v.push_back(high);
    }

    if (istr.good())
    {
        bounds.minx = v[0];
        bounds.maxx = v[1];
        bounds.miny = v[2];
        bounds.maxy = v[3];
        bounds.minz = v[4];
        bounds.maxz = v[5];
    }
    return istr;
}

} // namespace pdal
