/******************************************************************************
 * Copyright (c) 2014, Bradley J Chambers (brad.chambers@gmail.com)
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
#include <map>

#include "MortonOrderFilter.hpp"

namespace pdal
{


Options MortonOrderFilter::getDefaultOptions()
{
    Options options;
    return options;
}


//This used to be a lambda, but the VS compiler exploded, I guess.
typedef std::pair<double, double> Coord;
namespace
{
bool less_msb(const int& x, const int& y)
{
    return x < y && x < (x ^ y);
};

class CmpZOrder
{
public:
    bool operator()(const Coord& c1, const Coord& c2) const
    {
        int a[2] = {(int)(c1.first * INT_MAX), (int)(c1.second * INT_MAX)};
        int b[2] = {(int)(c2.first * INT_MAX), (int)(c2.second * INT_MAX)};

        int j = 0;
        int x = 0;

        for (int k = 0; k < 2; k++)
        {
            int y = a[k] ^ b[k];
            if (less_msb(x, y))
            {
                j = k;
                x = y;
            }
        }
        return (a[j] - b[j]) < 0;
    };
};
}

PointBufferSet MortonOrderFilter::run(PointBufferPtr buf)
{
    PointBufferSet pbSet;
    if (!buf->size())
        return pbSet;
    CmpZOrder compare;
    std::multimap<Coord, PointId, CmpZOrder> sorted(compare);

    BOX3D const& buffer_bounds = buf->calculateBounds();
    double xrange = buffer_bounds.maxx - buffer_bounds.minx;
    double yrange = buffer_bounds.maxy - buffer_bounds.miny;

    for (PointId idx = 0; idx < buf->size(); idx++)
    {
        double xpos = (buf->getFieldAs<double>(Dimension::Id::X, idx) - buffer_bounds.minx) / xrange;
        double ypos = (buf->getFieldAs<double>(Dimension::Id::Y, idx) - buffer_bounds.miny) / yrange;
        Coord loc(xpos, ypos);
        sorted.insert(std::make_pair(loc, idx));
    }

    PointBufferPtr outbuf = buf->makeNew();
    std::multimap<Coord, PointId, CmpZOrder>::iterator pos;
    for (pos = sorted.begin(); pos != sorted.end(); ++pos)
    {
        outbuf->appendPoint(*buf, pos->second);
    }
    pbSet.insert(outbuf);

    return pbSet;
}

} // pdal
