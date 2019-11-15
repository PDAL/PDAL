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

#include "private/delaunator.hpp"

#include <string>
#include <vector>
#include <cmath>

namespace pdal
{

namespace
{

// https://en.wikipedia.org/wiki/Barycentric_coordinate_system
// http://blackpawn.com/texts/pointinpoly/default.html
//
// If x/y is in the triangle, we'll return a valid distance.
// If not, we return infinity.  If the determinant is 0, the input points
// aren't a triangle (they're collinear).
double distance_along_z(double x1, double y1, double z1,
    double x2, double y2, double z2,
    double x3, double y3, double z3,
    double x, double y)
{
    double z = std::numeric_limits<double>::infinity();

    double detT = ((y2-y3) * (x1-x3)) + ((x3-x2) * (y1-y3));

    //ABELL - should probably check something close to 0, rather than
    // exactly 0.
    if (detT != 0.0)
    {
        // Compute the barycentric coordinates of x,y (relative to
        // x1/y1, x2/y2, x3/y3).  Essentially the wieght that each
        // corner of the triangle contributes to the point in question.

        // Another way to think about this is that we're making a basis
        // for the system with the basis vectors being two sides of
        // the triangle.  You can rearrange the z calculation below in
        // terms of lambda1 and lambda2 to see this.  Also note that
        // since lambda1 and lambda2 are coefficients of the basis vectors,
        // any values outside of the range [0,1] are necessarily out of the
        // triangle.
        double lambda1 = ((y2-y3) * (x-x3) + (x3-x2) * (y-y3)) / detT;
        double lambda2 = ((y3-y1) * (x-x3) + (x1-x3) * (y-y3)) / detT;
        if (lambda1 >= 0 && lambda1 <= 1 && lambda2 >= 0 && lambda2 <= 1)
        {
            double sum = lambda1 + lambda2;
            if (sum <= 1)
            {
                double lambda3 = 1 - sum;
                z = (lambda1 * z1) + (lambda2 * z2) + (lambda3 * z3);
            }
        }
    }
    return z;
}


// The non-ground point (x0, y0) is in exactly 0 or 1 of the triangles of
// the ground triangulation, so when we find a triangle containing the point,
// return the interpolated z.
// (I supposed the point could be on a edge of two triangles, but the
//  result is the same, so this is still good.)
double delaunay_interp_ground(double x0, double y0, PointViewPtr gView,
    const std::vector<size_t>& triangles, const PointIdList& ids)
{
    using namespace pdal::Dimension;

    double z1 = std::numeric_limits<double>::infinity();
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

        z1 = distance_along_z(ax, ay, az, bx, by, bz,
                cx, cy, cz, x0, y0);
        if (z1 != std::numeric_limits<double>::infinity())
            break;

    }
    return z1;
}

} // unnamed namespace


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


HAGFilter::HAGFilter()
{}


void HAGFilter::addArgs(ProgramArgs& args)
{
    args.add("count", "The number of points to fetch to determine the "
        "ground point [default: 1].", m_count, point_count_t(1));
    args.add("max_distance", "The maximum distance to the farthest nearest "
        "neighbor before the height above ground is not calculated "
        "[default: 0 (disabled)]", m_maxDistance);
    args.add("allow_extrapolation", "If true and count > 1, allow "
        "extrapolation [default: true].", m_allowExtrapolation, true);
    args.add("delaunay", "Construct local Delaunay fans and infer heights "
        "from them [default: false].", m_delaunay, false);
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
    using namespace pdal::Dimension;

    double maxDistance2 = std::pow(m_maxDistance, 2);
    PointViewPtr gView = view.makeNew();
    PointViewPtr ngView = view.makeNew();

    // First pass: Separate into ground and non-ground views.
    for (PointId i = 0; i < view.size(); ++i)
    {
        if (view.getFieldAs<short>(Id::Classification, i) == 2)
        {
            view.setField(Id::HeightAboveGround, i, 0);
            gView->appendPoint(view, i);
        }
        else
            ngView->appendPoint(view, i);
    }

    // Bail if there weren't any points classified as ground.
    if (gView->size() == 0)
        throwError("Input PointView does not have any points classified "
            "as ground");

    // Build the 2D KD-tree.
    const KD2Index& kdi = gView->build2dIndex();

    // Second pass: Find Z difference between non-ground points and the nearest
    // neighbor (2D) in the ground view or between non-ground points and the
    // locally-computed surface (Delaunay triangultion of the neighborhood).
    for (PointId i = 0; i < ngView->size(); ++i)
    {
        PointRef point = ngView->point(i);
        double x0 = point.getFieldAs<double>(Id::X);
        double y0 = point.getFieldAs<double>(Id::Y);
        double z0 = point.getFieldAs<double>(Id::Z);
        PointIdList ids = kdi.neighbors(point, m_count);

        double z1 = std::numeric_limits<double>::quiet_NaN();
        assert(ids.size() > 0);
        if (ids.size() == 1)
        {
            z1 = gView->getFieldAs<double>(Id::Z, ids[0]);
        }
        else if (m_delaunay == false)
        {
            // Nearest-Neighbor-based interpolation
            auto min_x = (std::numeric_limits<double>::max)();
            auto max_x = (std::numeric_limits<double>::lowest)();
            auto min_y = (std::numeric_limits<double>::max)();
            auto max_y = (std::numeric_limits<double>::lowest)();

            double weights = 0;
            double z_accumulator = 0;
            bool exact_match = false;
            for (size_t j = 0; j < ids.size(); ++j)
            {
                auto x = gView->getFieldAs<double>(Id::X, ids[j]);
                auto y = gView->getFieldAs<double>(Id::Y, ids[j]);
                auto z = gView->getFieldAs<double>(Id::Z, ids[j]);
                auto distance2 = std::pow(x - x0, 2) + std::pow(y - y0, 2);
                if (distance2 == 0)
                {
                    //ABELL - This doesn't make sense.  Z1 is set, but if
                    // exact match is set, it gets overwritten outside of
                    // the for loop.
                    exact_match = true;
                    z1 = z;
                    break;
                }
                if (maxDistance2 > 0 && distance2 > maxDistance2) {
                    break;
                } else {
                    auto weight = 1 / distance2;
                    weights += weight;
                    z_accumulator += weight * z;
                }
                max_x = (std::max)(x, max_x);
                min_x = (std::min)(x, min_x);
                max_y = (std::max)(y, max_y);
                min_y = (std::min)(y, min_y);
            }
            if (exact_match || m_allowExtrapolation ||
                (x0 > min_x && x0 < max_x && y0 > min_y && y0 < max_y))
            {
                z1 = z_accumulator / weights;
            }
        }
        else if (m_delaunay == true)
        {
            // Delaunay-based interpolation
            std::vector<double> neighbors;

            for (size_t j = 0; j < ids.size(); ++j)
            {
                neighbors.push_back(gView->getFieldAs<double>(Id::X, ids[j]));
                neighbors.push_back(gView->getFieldAs<double>(Id::Y, ids[j]));
            }

            delaunator::Delaunator triangulation(neighbors);
            z1 = delaunay_interp_ground(x0, y0, gView,
                triangulation.triangles, ids);

            // If the non ground point was outside the triangulation of ground
            // points, just use the Z coordinate of the closest
            // ground point.
            if (z1 == std::numeric_limits<double>::infinity())
                z1 = gView->getFieldAs<double>(Id::Z, ids[0]);
        }
        ngView->setField(Dimension::Id::HeightAboveGround, i, z0 - z1);
    }
}

} // namespace pdal
