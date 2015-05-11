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

#include "GridderFilter.hpp"

#include <pdal/pdal_macros.hpp>

#include <iostream>
#include <limits>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.gridder",
    "Grid data based on a number of grid cells and a global bounding box",
    "http://pdal.io/stages/filters.gridder.html" );

CREATE_STATIC_PLUGIN(1, 0, GridderFilter, Filter, s_info)

std::string GridderFilter::getName() const { return s_info.name; }

void GridderFilter::processOptions(const Options& options)
{
    num_x = options.getValueOrDefault<uint32_t>("num_x", 16);
    num_y = options.getValueOrDefault<uint32_t>("num_y", 16);
    min_x = options.getValueOrDefault<double>("min_x", 0);
    max_x = options.getValueOrDefault<double>("max_x", 0);
    min_y = options.getValueOrDefault<double>("min_y", 0);
    max_y = options.getValueOrDefault<double>("max_y", 0);
}

Options GridderFilter::getDefaultOptions()
{
    Options options;
    Option num_x("num_x", 0, "Number of grid cells in X");
    options.add(num_x);
    Option num_y("num_y", 0, "Number of grid cells in Y");
    options.add(num_y);
    Option min_x("min_x", 0, "Minimum X of grid");
    options.add(min_x);
    Option max_x("max_x", 0, "Maximum X of grid");
    options.add(max_x);
    Option min_y("min_y", 0, "Minimum Y of grid");
    options.add(min_y);
    Option max_y("max_y", 0, "Maximum Y of grid");
    options.add(max_y);

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

PointViewSet GridderFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;
    if (!inView->size())
        return viewSet;

    CoordCompare compare;
    std::map<Coord, PointViewPtr, CoordCompare> viewMap(compare);

    double factor_x = num_x / (max_x - min_x);
    double factor_y = num_y / (max_y - min_y);

    // Overlay a grid of squares with defined spatial extents given the
    // min_x, min_y, max_x, max_y and num_cells
    // Place the points falling in each grid cells in the corresponding point buffer.
    for (PointId idx = 0; idx < inView->size(); idx++)
    {
        int xpos = (inView->getFieldAs<double>(Dimension::Id::X, idx) - min_x) * factor_x;
        int ypos = (inView->getFieldAs<double>(Dimension::Id::Y, idx) - min_y) * factor_y;
        Coord loc(xpos, ypos);
        PointViewPtr& outView = viewMap[loc];
        if (!outView)
        {
            outView = inView->makeNew();
        }
        outView->appendPoint(*inView.get(), idx);
    }

    // Pull the buffers out of the map and stick them in the standard
    // output set, setting the bounds as we go.
    for (auto bi = viewMap.begin(); bi != viewMap.end(); ++bi)
        viewSet.insert(bi->second);
    return viewSet;
}

} // pdal
