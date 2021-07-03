/******************************************************************************
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

#include "HagDelaunayFilter.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/private/MathUtils.hpp>

#include "private/delaunator.hpp"

#include <string>
#include <vector>
#include <cmath>

namespace pdal
{

namespace
{

// The non-ground point (x0, y0) is in exactly 0 or 1 of the triangles of
// the ground triangulation, so when we find a triangle containing the point,
// return the interpolated z.
// (I suppose the point could be on a edge of two triangles, but the
//  result is the same, so this is still good.)
double delaunay_interp_ground(double x0, double y0, PointViewPtr gView,
    const PointIdList& ids)
{
    using namespace pdal::Dimension;

    // Delaunay-based interpolation
    std::vector<double> neighbors;

    for (size_t j = 0; j < ids.size(); ++j)
    {
        neighbors.push_back(gView->getFieldAs<double>(Id::X, ids[j]));
        neighbors.push_back(gView->getFieldAs<double>(Id::Y, ids[j]));
    }

    delaunator::Delaunator triangulation(neighbors);
    const std::vector<size_t>& triangles(triangulation.triangles);

    for (size_t j = 0; j < triangles.size(); j += 3)
    {
        auto ai = triangles[j+0];
        auto bi = triangles[j+1];
        auto ci = triangles[j+2];
        double ax = gView->getFieldAs<double>(Id::X, ids[ai]);
        double ay = gView->getFieldAs<double>(Id::Y, ids[ai]);
        double az = gView->getFieldAs<double>(Id::Z, ids[ai]);

        double bx = gView->getFieldAs<double>(Id::X, ids[bi]);
        double by = gView->getFieldAs<double>(Id::Y, ids[bi]);
        double bz = gView->getFieldAs<double>(Id::Z, ids[bi]);

        double cx = gView->getFieldAs<double>(Id::X, ids[ci]);
        double cy = gView->getFieldAs<double>(Id::Y, ids[ci]);
        double cz = gView->getFieldAs<double>(Id::Z, ids[ci]);

        // Returns infinity unless the point x0/y0 is in the triangle.
        double z1 = math::barycentricInterpolation(ax, ay, az, bx, by, bz,
                cx, cy, cz, x0, y0);
        if (z1 != std::numeric_limits<double>::infinity())
            return z1;
    }
    // If the non ground point was outside the triangulation of ground
    // points, just use the Z coordinate of the closest
    // ground point.
    return gView->getFieldAs<double>(Id::Z, ids[0]);
}

} // unnamed namespace


static StaticPluginInfo const s_info
{
    "filters.hag_delaunay",
    "Computes height above ground using delaunay interpolation of "
        "ground returns.",
    "http://pdal.io/stages/filters.hag_delaunay.html"
};

CREATE_STATIC_STAGE(HagDelaunayFilter, s_info)

std::string HagDelaunayFilter::getName() const
{
    return s_info.name;
}


HagDelaunayFilter::HagDelaunayFilter()
{}


void HagDelaunayFilter::addArgs(ProgramArgs& args)
{
    args.add("count", "The number of points to fetch to determine the "
        "ground point [Default: 10].", m_count, point_count_t(10));
    args.add("allow_extrapolation", "Allow extrapolation for points "
        "outside of the local triangulations. [Default: true].",
        m_allowExtrapolation, true);
}


void HagDelaunayFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::HeightAboveGround);
}


void HagDelaunayFilter::prepared(PointTableRef table)
{
    if (m_count < 3)
        throwError("Option 'count' must be at least 3.");

    const PointLayoutPtr layout(table.layout());
    if (!layout->hasDim(Dimension::Id::Classification))
        throwError("Missing Classification dimension in input PointView.");
}


void HagDelaunayFilter::filter(PointView& view)
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
    if (gView->size() == 0)
        log()->get(LogLevel::Error) << "Input PointView does not have any "
            "points classified as ground.\n";

    // Build the 2D KD-tree.
    const KD2Index& kdi = gView->build2dIndex();

    // Find Z difference between non-ground points and the nearest
    // neighbor (2D) in the ground view or between non-ground points and the
    // locally-computed surface (Delaunay triangultion of the neighborhood).
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
            z1 = delaunay_interp_ground(x0, y0, gView, ids);
        }
        ngView->setField(Dimension::Id::HeightAboveGround, i, z0 - z1);
    }
}

} // namespace pdal
