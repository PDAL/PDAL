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

#include "HagNnFilter.hpp"

#include <pdal/KDIndex.hpp>

#include <string>
#include <vector>
#include <cmath>

namespace pdal
{

namespace
{

double neighbor_interp_ground(PointViewPtr gView, const PointIdList& ids,
    const std::vector<double>& sqr_dists, double maxDistance2, double zDefault)
{
    double weights = 0;
    double z_accumulator = 0;

    for (size_t j = 0; j < ids.size(); ++j)
    {
        auto z = gView->getFieldAs<double>(Dimension::Id::Z, ids[j]);
        double sqr_dist = sqr_dists[j];
        if (maxDistance2 > 0 && sqr_dist > maxDistance2)
            break;
        else {
            double weight = 1 / sqr_dist;
            weights += weight;
            z_accumulator += weight * z;
        }
    }

    if (weights)
        return z_accumulator / weights;
    return zDefault;
}

} // unnamed namespace


static StaticPluginInfo const s_info
{
    "filters.hag_nn",
    "Computes height above ground using nearest-neighbor ground-classified "
        "returns.",
    "http://pdal.io/stages/filters.hag_nn.html"
};

CREATE_STATIC_STAGE(HagNnFilter, s_info)

std::string HagNnFilter::getName() const
{
    return s_info.name;
}


HagNnFilter::HagNnFilter()
{}


void HagNnFilter::addArgs(ProgramArgs& args)
{
    args.add("count", "The number of points to fetch to determine the "
        "ground point [Default: 1].", m_count, point_count_t(1));
    args.add("max_distance", "Ground points beyond this distance will not "
        "influence nearest neighbor interpolation of height above ground."
        "[Default: None]", m_maxDistance);
    args.add("allow_extrapolation", "If true and count > 1, allow "
        "extrapolation [Default: true].", m_allowExtrapolation, true);
}


void HagNnFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::HeightAboveGround);
}


void HagNnFilter::prepared(PointTableRef table)
{
    if (m_count == 0)
        throwError("Option 'count' must be a positive integer.");

    const PointLayoutPtr layout(table.layout());
    if (!layout->hasDim(Dimension::Id::Classification))
        throwError("Missing Classification dimension in input PointView.");
}


void HagNnFilter::filter(PointView& view)
{
    using namespace pdal::Dimension;

    PointViewPtr gView = view.makeNew();
    PointViewPtr ngView = view.makeNew();

    // Separate into ground and non-ground views.
    for (PointId i = 0; i < view.size(); ++i)
    {
        if (view.getFieldAs<uint8_t>(Id::Classification, i) ==
            ClassLabel::Ground)
        {
            view.setField(Id::HeightAboveGround, i, 0);
            gView->appendPoint(view, i);
        }
        else
            ngView->appendPoint(view, i);
    }
    BOX2D gBounds;
    gView->calculateBounds(gBounds);

    // Bail if there weren't any points classified as ground.
    if (gView->size() == 0) {
        log()->get(LogLevel::Error) << "Input PointView does not have "
            "any points classified as ground.\n";
        return;
    }

    // Build the 2D KD-tree.
    const KD2Index& kdi = gView->build2dIndex();

    double maxDistance2 = std::pow(m_maxDistance, 2.0);
    // Find Z difference between non-ground points and the nearest
    // neighbor (2D) in the ground view or between non-ground points and the
    // locally-computed surface.
    for (PointId i = 0; i < ngView->size(); ++i)
    {
        PointRef point = ngView->point(i);

        // Non-ground view point for which we're trying to calc HAG
        double x0 = point.getFieldAs<double>(Id::X);
        double y0 = point.getFieldAs<double>(Id::Y);
        double z0 = point.getFieldAs<double>(Id::Z);

        PointIdList ids(m_count);
        std::vector<double> sqr_dists(m_count);
        kdi.knnSearch(x0, y0, m_count, &ids, &sqr_dists);

        // Closest ground point.
        double x = gView->getFieldAs<double>(Id::X, ids[0]);
        double y = gView->getFieldAs<double>(Id::Y, ids[0]);
        double z = gView->getFieldAs<double>(Id::Z, ids[0]);

        double z1;
        // If the close ground point is at the same X/Y as the non-ground
        // point, we're done.  Also, if there's only one ground point, we
        // just use that.
        if ((x0 == x && y0 == y) || ids.size() == 1)
        {
            z1 = z;
        }
        // If the non-ground point is outside the bounds of all the
        // ground points and we're not doing extrapolation, just return
        // its current Z, which will give a HAG of 0.
        else if (!gBounds.contains(x0, y0) && !m_allowExtrapolation)
        {
            z1 = z0;
        }
        else
        {
            z1 = neighbor_interp_ground(gView, ids, sqr_dists,
                maxDistance2, z0);
        }
        ngView->setField(Dimension::Id::HeightAboveGround, i, z0 - z1);
    }
}

} // namespace pdal
