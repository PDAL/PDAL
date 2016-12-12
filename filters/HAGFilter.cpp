/******************************************************************************
* Copyright (c) 2016, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "HAGFilter.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/pdal_macros.hpp>

#include <string>
#include <vector>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.hag", "HAG Filter",
               "http://pdal.io/stages/filters.hag.html");

CREATE_STATIC_PLUGIN(1, 0, HAGFilter, Filter, s_info)

std::string HAGFilter::getName() const
{
    return s_info.name;
}

void HAGFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::HeightAboveGround);
}

void HAGFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    if (!layout->hasDim(Dimension::Id::Classification))
        throw pdal_error("HAGFilter: missing Classification dimension in input PointView");
}

void HAGFilter::filter(PointView& view)
{
    PointViewPtr gView = view.makeNew();
    PointViewPtr ngView = view.makeNew();
    std::vector<PointId> gIdx, ngIdx;

    // First pass: Separate into ground and non-ground views.
    for (PointId i = 0; i < view.size(); ++i)
    {
        double c = view.getFieldAs<double>(Dimension::Id::Classification, i);
        if (c == 2)
        {
            gView->appendPoint(view, i);
            gIdx.push_back(i);
        }
        else
        {
            ngView->appendPoint(view, i);
            ngIdx.push_back(i);
        }
    }

    // Bail if there weren't any points classified as ground.
    if (gView->size() == 0)
        throw pdal_error("HAGFilter: the input PointView does not appear to have any points classified as ground");

    // Build the 2D KD-tree.
    KD2Index kdi(*gView);
    kdi.build();

    // Second pass: Find Z difference between non-ground points and the nearest 
    // neighbor (2D) in the ground view.
    for (PointId i = 0; i < ngView->size(); ++i)
    {
        PointRef point = ngView->point(i);
        double z0 = point.getFieldAs<double>(Dimension::Id::Z);
        auto ids = kdi.neighbors(point, 1);
        double z1 = gView->getFieldAs<double>(Dimension::Id::Z, ids[0]);
        view.setField(Dimension::Id::HeightAboveGround, ngIdx[i], z0 - z1);
    }

    // Final pass: Ensure that all ground points have height value pegged at 0.
    for (auto const& i : gIdx)
        view.setField(Dimension::Id::HeightAboveGround, i, 0.0);
}


} // namespace pdal
