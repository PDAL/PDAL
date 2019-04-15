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

#include <string>
#include <vector>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.hag",
    "Computes height above ground using ground-classified returns.",
    "http://pdal.io/stages/filters.hag.html"
};

CREATE_STATIC_STAGE(HAGFilter, s_info)

std::string HAGFilter::getName() const
{
    return s_info.name;
}

void HAGFilter::addArgs(ProgramArgs& args)
{
    args.add("count", "The number of points to fetch to determine the ground point [default: 1].",
            m_count, point_count_t(1));
    args.add("max_distance", "The maximum distance to the farthest nearest neighbor before the height above ground is not calculated [default: 0 (disabled)]", m_max_distance);
    args.add("allow_extrapolation", "If true and count > 1, allow extrapolation [default: true].",
            m_allow_extrapolation, true);
}

void HAGFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::HeightAboveGround);
}

void HAGFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    if (!layout->hasDim(Dimension::Id::Classification))
        throwError("Missing Classification dimension in input PointView.");
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
        throwError("Input PointView does not have any points classified "
            "as ground");

    // Build the 2D KD-tree.
    KD2Index& kdi = gView->build2dIndex();

    // Second pass: Find Z difference between non-ground points and the nearest
    // neighbor (2D) in the ground view.
    for (PointId i = 0; i < ngView->size(); ++i)
    {
        PointRef point = ngView->point(i);
        double x0 = point.getFieldAs<double>(Dimension::Id::X);
        double y0 = point.getFieldAs<double>(Dimension::Id::Y);
        double z0 = point.getFieldAs<double>(Dimension::Id::Z);
        auto ids = kdi.neighbors(point, m_count);
        double z1 = std::numeric_limits<double>::quiet_NaN();
        assert(ids.size() > 0);
        if (ids.size() == 1) {
            z1 = gView->getFieldAs<double>(Dimension::Id::Z, ids[0]);
        } else {
            auto min_x = std::numeric_limits<double>::max();
            auto max_x = std::numeric_limits<double>::min();
            auto min_y = std::numeric_limits<double>::max();
            auto max_y = std::numeric_limits<double>::min();
            double weights = 0;
            double z_accumulator = 0;
            bool exact_match = false;
            for (unsigned long j = 0; j < ids.size(); ++j) {
                auto x = gView->getFieldAs<double>(Dimension::Id::X, ids[j]);
                auto y = gView->getFieldAs<double>(Dimension::Id::Y, ids[j]);
                auto z = gView->getFieldAs<double>(Dimension::Id::Z, ids[j]);
                auto distance = std::sqrt(std::pow(x - x0, 2) + std::pow(y - y0, 2));
                if (distance == 0) {
                    exact_match = true;
                    z1 = z;
                    break;
                }
                if (m_max_distance > 0 && distance > m_max_distance) {
                    break;
                } else {
                    auto weight = 1 / std::pow(distance, 2);
                    weights += weight;
                    z_accumulator += weight * z;
                }
                if (x > max_x) {
                    max_x = x;
                }
                if (x < min_x) {
                    min_x = x;
                }
                if (y > max_y) {
                    max_y = y;
                }
                if (y < min_y) {
                    min_y = y;
                }
            }
            if (exact_match || m_allow_extrapolation || (x0 > min_x && x0 < max_x && y0 > min_y && y0 < max_y)) {
                z1 = z_accumulator / weights;
            }
        }
        view.setField(Dimension::Id::HeightAboveGround, ngIdx[i], z0 - z1);
    }

    // Final pass: Ensure that all ground points have height value pegged at 0.
    for (auto const& i : gIdx)
        view.setField(Dimension::Id::HeightAboveGround, i, 0.0);
}


} // namespace pdal
