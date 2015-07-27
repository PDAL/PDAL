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

#include <pdal/pdal_macros.hpp>

#include <cmath>
#include <iostream>
#include <limits>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.splitter",
    "Split data based on a X/Y box length.",
    "http://pdal.io/stages/filters.splitter.html" );

CREATE_STATIC_PLUGIN(1, 0, SplitterFilter, Filter, s_info)

std::string SplitterFilter::getName() const { return s_info.name; }

void SplitterFilter::processOptions(const Options& options)
{
    m_length = options.getValueOrDefault<double>("length", 1000.0);
    m_xOrigin = options.getValueOrDefault<double>("origin_x",
        std::numeric_limits<double>::quiet_NaN());
    m_yOrigin = options.getValueOrDefault<double>("origin_y",
        std::numeric_limits<double>::quiet_NaN());
}


Options SplitterFilter::getDefaultOptions()
{
    Options options;
    Option length("length", 1000.0, "Splitter length");
    options.add(length);

    return options;
}


//This used to be a lambda, but the VS compiler exploded, I guess.
typedef std::pair<int, int> Coord;
namespace
{
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
}

PointViewSet SplitterFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;
    if (!inView->size())
        return viewSet;

    CoordCompare compare;
    std::map<Coord, PointViewPtr, CoordCompare> viewMap(compare);

    // Use the location of the first point as the origin, unless specified.
    // (!= test == isnan(), which doesn't exist on windows)
    if (m_xOrigin != m_xOrigin)
        m_xOrigin = inView->getFieldAs<double>(Dimension::Id::X, 0);
    if (m_yOrigin != m_yOrigin)
        m_yOrigin = inView->getFieldAs<double>(Dimension::Id::Y, 0);
    // Overlay a grid of squares on the points (m_length sides).  Each square
    // corresponds to a new point buffer.  Place the points falling in the
    // each square in the corresponding point buffer.
    for (PointId idx = 0; idx < inView->size(); idx++)
    {
        double x = inView->getFieldAs<double>(Dimension::Id::X, idx);
        int xpos = (x - m_xOrigin) / m_length;
        double y = inView->getFieldAs<double>(Dimension::Id::Y, idx);
        int ypos = (y - m_yOrigin) / m_length;

        Coord loc(xpos, ypos);
        PointViewPtr& outView = viewMap[loc];
        if (!outView)
            outView = inView->makeNew();
        outView->appendPoint(*inView.get(), idx);
    }

    // Pull the buffers out of the map and stick them in the standard
    // output set, setting the bounds as we go.
    for (auto bi = viewMap.begin(); bi != viewMap.end(); ++bi)
        viewSet.insert(bi->second);
    return viewSet;
}

} // pdal
