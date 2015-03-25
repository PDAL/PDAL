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

#pragma GCC diagnostic ignored "-Wfloat-equal"

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

} // namespace

namespace pdal
{

const double BOX3D::LOWEST = (std::numeric_limits<double>::lowest)();
const double BOX3D::HIGHEST = (std::numeric_limits<double>::max)();
    
BOX3D::BOX3D(double minx, double miny, double maxx, double maxy) :
    minx(minx), maxx(maxx), miny(miny), maxy(maxy),
    minz(HIGHEST), maxz(LOWEST)
{}

void BOX3D::clear()
{
    minx = HIGHEST; miny = HIGHEST; minz = HIGHEST;
    maxx = LOWEST; maxy = LOWEST; maxz = LOWEST;
}

bool BOX3D::empty() const
{
    return  minx == HIGHEST && maxx == LOWEST &&
            miny == HIGHEST && maxy == LOWEST &&
            minz == HIGHEST && maxz == LOWEST;
}

void BOX3D::grow(double x, double y)
{
    grow(x, y, LOWEST);
}

void BOX3D::grow(double x, double y, double z)
{
    if (x < minx) minx = x;
    if (x > maxx) maxx = x;

    if (y < miny) miny = y;
    if (y > maxy) maxy = y;

    if (z < minz) minz = z;
    if (z > maxz) maxz = z;
}

bool BOX3D::is_z_empty() const
{
    return ((minz == HIGHEST) && (maxz == LOWEST));
}

const BOX3D& BOX3D::getDefaultSpatialExtent()
{
    static BOX3D v(LOWEST, LOWEST, LOWEST, HIGHEST, HIGHEST, HIGHEST);
    return v;
}    

std::istream& operator>>(std::istream& istr, BOX3D& bounds)
{
    istr.get();
    if (istr.eof())
    {
        BOX3D output;
        bounds = output;
        return istr;
    }

    if (!istr.good())
    {
        istr.setstate(std::ios_base::failbit);
        return istr;
    }

    istr.unget();

    // A really dirty way to check for an empty bounds object right off
    // the bat
    const char left_paren = (char)istr.get();
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
    {
        istr.setstate(std::ios_base::failbit);
        return istr;
    }

    bool done = false;
    while (!done)
    {
        eat(istr, isspace);
        double low, high;

        if (!eat(istr,'['))
        {
            istr.setstate(std::ios_base::failbit);
            return istr;
        }

        eat(istr, isspace);
        istr >> low;
        eat(istr, isspace);
        if (!eat(istr,','))
        {
            istr.setstate(std::ios_base::failbit);
            return istr;
        }

        eat(istr, isspace);
        istr >> high;

        if (!eat(istr,']'))
        {
            istr.setstate(std::ios_base::failbit);
            return istr;
        }

        eat(istr, isspace);
        if (eat(istr,','))
            done = false;
        else if (eat(istr,')'))
            done = true;
        else
        {
            istr.setstate(std::ios_base::failbit);
            return istr;
        }
        v.push_back(low);
        v.push_back(high);
    }

    BOX3D xxx;
    if (v.size() == 4)
    {
        xxx.minx = v[0];
        xxx.maxx = v[1];
        xxx.miny = v[2];
        xxx.maxy = v[3];
    }
    else if (v.size() == 6)
    {
        xxx.minx = v[0];
        xxx.maxx = v[1];
        xxx.miny = v[2];
        xxx.maxy = v[3];
        xxx.minz = v[4];
        xxx.maxz = v[5];
    }
    else
        istr.setstate(std::ios_base::failbit);
    eat(istr, isspace);
    bounds = xxx;
    return istr;
}

} // namespace pdal
