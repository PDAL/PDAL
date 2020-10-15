/******************************************************************************
 * Copyright (c) 2013, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "SplitterFilter.hpp"

#include <cmath>
#include <iostream>
#include <limits>

#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.splitter",
    "Split data based on a X/Y box length.",
    "http://pdal.io/stages/filters.splitter.html"
};

CREATE_STATIC_STAGE(SplitterFilter, s_info)

SplitterFilter::SplitterFilter() : m_viewMap(CoordCompare())
{}

std::string SplitterFilter::getName() const { return s_info.name; }


PointViewPtr SplitterFilter::view(const Coord& coord)
{
    auto vi = m_viewMap.find(coord);
    if (vi == m_viewMap.end())
        return nullptr;
    return vi->second;
}


BOX2D SplitterFilter::bounds(const Coord& coord) const
{
    const int& xpos = coord.first;
    const int& ypos = coord.second;

    double minx = m_xOrigin + xpos * m_length;
    double maxx = minx + m_length;
    double miny = m_yOrigin + ypos * m_length;
    double maxy = miny + m_length;

    return BOX2D(minx, miny, maxx, maxy);
}


BOX2D SplitterFilter::bufferedBounds(const Coord& coord) const
{
    const int& xpos = coord.first;
    const int& ypos = coord.second;

    double minx = m_xOrigin + xpos * m_length - m_buffer;
    double maxx = minx + m_length + 2 * m_buffer;
    double miny = m_yOrigin + ypos * m_length - m_buffer;
    double maxy = miny + m_length + 2 * m_buffer;

    return BOX2D(minx, miny, maxx, maxy);
}


BOX2D SplitterFilter::extent() const
{
    BOX2D box;

    for (auto& i : m_viewMap)
    {
        const Coord& c = i.first;
        box.grow(c.first, c.second);
    }
    return box;
}


void SplitterFilter::addArgs(ProgramArgs& args)
{
    args.add("length", "Edge length of cell", m_length, 1000.0);
    args.add("origin_x", "X origin for a cell", m_xOrigin,
        std::numeric_limits<double>::quiet_NaN());
    args.add("origin_y", "Y origin for a cell", m_yOrigin,
        std::numeric_limits<double>::quiet_NaN());
    args.add("buffer", "Size of buffer (overlap) to include around each tile.",
        m_buffer, 0.0);
}


void SplitterFilter::initialize()
{
    if (m_buffer >= m_length / 2)
    {
        std::stringstream oss;
        oss << "Buffer (" << m_buffer <<
            ") must be less than half of length (" << m_length << ")";
        throwError(oss.str());
    }
}


void SplitterFilter::setOrigin(double xOrigin, double yOrigin)
{
    m_xOrigin = xOrigin;
    m_yOrigin = yOrigin;
}


PointViewSet SplitterFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;
    if (!inView->size())
        return viewSet;

    auto addPoint = [this, &inView](PointRef& point, int xpos, int ypos) {
        Coord loc(xpos, ypos);
        PointViewPtr& outView = m_viewMap[loc];
        if (!outView)
            outView = inView->makeNew();
        outView->appendPoint(*inView.get(), point.pointId());
    };

    // Use the location of the first point as the origin, unless specified.
    // (!= test == isnan(), which doesn't exist on windows)
    if (m_xOrigin != m_xOrigin)
        setOrigin(inView->getFieldAs<double>(Dimension::Id::X, 0), m_yOrigin);
    if (m_yOrigin != m_yOrigin)
        setOrigin(m_xOrigin, inView->getFieldAs<double>(Dimension::Id::Y, 0));
    // Overlay a grid of squares on the points (m_length sides).  Each square
    // corresponds to a new point buffer.  Place the points falling in the
    // each square in the corresponding point buffer.
    PointRef point(*inView, 0);
    for (PointId idx = 0; idx < inView->size(); idx++)
    {
        point.setPointId(idx);
        processPoint(point, addPoint);
    }

    // Pull the buffers out of the map and stick them in the standard
    // output set.
    for (auto bi = m_viewMap.begin(); bi != m_viewMap.end(); ++bi)
        viewSet.insert(bi->second);
    return viewSet;
}


void SplitterFilter::processPoint(PointRef& point, PointAdder adder)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double dx = x - m_xOrigin;
    int xpos = static_cast<int>(dx / m_length);
    if (dx < 0)
        xpos--;

    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double dy = y - m_yOrigin;
    int ypos = static_cast<int>(dy / m_length);
    if (dy < 0)
        ypos--;

    adder(point, xpos, ypos);

    // We check in initialize() to make sure that the buffer value isn't more
    // than have the cell edge length.
    if (m_buffer > 0.0) {
        if (squareContains(xpos - 1, ypos, x, y))
            adder(point, xpos - 1, ypos);
        else if (squareContains(xpos + 1, ypos, x, y))
            adder(point, xpos + 1, ypos);

        if (squareContains(xpos, ypos - 1, x, y))
            adder(point, xpos, ypos - 1);
        else if (squareContains(xpos, ypos + 1, x, y))
            adder(point, xpos, ypos + 1);

        if (squareContains(xpos - 1, ypos - 1, x, y))
            adder(point, xpos - 1, ypos - 1);
        else if (squareContains(xpos - 1, ypos + 1, x, y))
            adder(point, xpos - 1, ypos + 1);
        else if (squareContains(xpos + 1, ypos - 1, x, y))
            adder(point, xpos + 1, ypos - 1);
        else if (squareContains(xpos + 1, ypos + 1, x, y))
            adder(point, xpos + 1, ypos + 1);
    }
}


bool SplitterFilter::squareContains(int xpos, int ypos,
    double x, double y) const
{
    double minx = m_xOrigin + xpos * m_length - m_buffer;
    double maxx = minx + m_length + 2 * m_buffer;
    double miny = m_yOrigin + ypos * m_length - m_buffer;
    double maxy = miny + m_length + 2 * m_buffer;

    return minx < x && x < maxx && miny < y && y < maxy;
}

} // pdal
