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

#pragma once

#include <pdal/Filter.hpp>

namespace pdal
{

class PDAL_DLL SplitterFilter : public pdal::Filter
{
private:
    //This used to be a lambda, but the VS compiler exploded, I guess.
    typedef std::pair<int, int> Coord;
    class CoordCompare
    {
    public:
        bool operator () (const Coord& c1, const Coord& c2) const
        {
            return c1.first < c2.first ? true :
                c1.first > c2.first ? false :
                c1.second < c2.second ? true :
                false;
        };
    };

public:
    SplitterFilter();
    using PointAdder = std::function<void(PointRef&, int, int)>;

    std::string getName() const;
    void setOrigin(double xOrigin, double yOrigin);
    void processPoint(PointRef& p, PointAdder adder);
    PointViewPtr view(const Coord& c);

    // Return the bounds of a tile.
    BOX2D bounds(const Coord& c) const;
    // Return the buffered bounds of a tile.
    BOX2D bufferedBounds(const Coord& c) const;
    // Return the coordinate extent of tiles.  Note that the return type
    // will contain integer-valued doubles.
    BOX2D extent() const;

private:
    double m_length;
    double m_xOrigin;
    double m_yOrigin;
    double m_buffer;
    std::map<Coord, PointViewPtr, CoordCompare> m_viewMap;

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual PointViewSet run(PointViewPtr view);
    bool squareContains(int xpos, int ypos, double x, double y) const;

    SplitterFilter& operator=(const SplitterFilter&); // not implemented
    SplitterFilter(const SplitterFilter&); // not implemented
};

} // namespace pdal
