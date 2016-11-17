/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include "CropFilter.hpp"

#include <iomanip>

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Polygon.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/KDIndex.hpp>

#include <sstream>
#include <cstdarg>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.crop",
    "Filter points inside or outside a bounding box or a polygon",
    "http://pdal.io/stages/filters.crop.html" );

CREATE_STATIC_PLUGIN(1, 0, CropFilter, Filter, s_info)

std::string CropFilter::getName() const { return s_info.name; }

CropFilter::CropFilter() : pdal::Filter()
{
    m_cropOutside = false;
}


void CropFilter::addArgs(ProgramArgs& args)
{
    args.add("outside", "Whether we keep points inside or outside of the "
        "bounding region", m_cropOutside);
    args.add("a_srs", "Spatial reference for bounding region", m_assignedSrs);
    args.add("bounds", "Point box for cropped points", m_bounds);
    args.add("point", "Crop within 'distance' from a 2D or 3D point", m_points).
        setErrorText("Invalid point specification must be in the form \"(1.00, 1.00)\""
                "or \"(1.00, 1.00, 1.00)\"");
    args.add("distance", "Crop with this distance from 2D or 3D 'point'", m_distance);
    args.add("polygon", "Bounding polying for cropped points", m_polys).
        setErrorText("Invalid polygon specification.  "
            "Must be valid GeoJSON/WKT");
}


void CropFilter::initialize()
{

    // Set geometry from polygons.
    if (m_polys.size())
    {
        m_geoms.clear();
        for (Polygon& poly : m_polys)
        {
            GeomPkg g;

            // Throws if invalid.
            poly.valid();
            if (!m_assignedSrs.empty())
                poly.setSpatialReference(m_assignedSrs);
            g.m_geom = poly;
            m_geoms.push_back(g);
        }
    }

}


void CropFilter::ready(PointTableRef table)
{
    for (auto& geom : m_geoms)
    {
        // If we already overrode the SRS, use that instead
        if (m_assignedSrs.empty())
            geom.m_geom.setSpatialReference(table.anySpatialReference());
    }
}


bool CropFilter::processOne(PointRef& point)
{
    for (auto& geom : m_geoms)
        if (!crop(point, geom))
            return false;

    for (auto& box : m_bounds)
        if (!crop(point, box.to2d()))
            return false;

    return true;
}


PointViewSet CropFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;
    SpatialReference srs = view->spatialReference();

    for (auto& geom : m_geoms)
    {
        // If this is the first time through or the SRS has changed,
        // prepare the crop polygon.
        if (srs != m_lastSrs)
        {
            geom.m_geom = geom.m_geom.transform(srs);
        }

        PointViewPtr outView = view->makeNew();
        crop(geom, *view, *outView);
        viewSet.insert(outView);
    }
    m_lastSrs = srs;

    for (auto& box : m_bounds)
    {
        PointViewPtr outView = view->makeNew();
        crop(box.to2d(), *view, *outView);
        viewSet.insert(outView);
    }

    for (auto& point: m_points)
    {
        PointViewPtr outView = view->makeNew();
        crop(point, m_distance, *view, *outView);
        viewSet.insert(outView);
    }

    return viewSet;
}


bool CropFilter::crop(PointRef& point, const BOX2D& box)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);

    // Return true if we're keeping a point.
    return (m_cropOutside != box.contains(x, y));
}


void CropFilter::crop(const BOX2D& box, PointView& input, PointView& output)
{
    PointRef point = input.point(0);
    for (PointId idx = 0; idx < input.size(); ++idx)
    {
        point.setPointId(idx);
        if (crop(point, box))
            output.appendPoint(input, idx);
    }
}

bool CropFilter::crop(PointRef& point, const GeomPkg& g)
{
    bool covers = g.m_geom.covers(point);
    bool keep = (m_cropOutside != covers);
    return keep;
}

void CropFilter::crop(const GeomPkg& g, PointView& input, PointView& output)
{
    PointRef point = input.point(0);
    for (PointId idx = 0; idx < input.size(); ++idx)
    {
        point.setPointId(idx);
        bool covers = g.m_geom.covers(point);
        bool keep = (m_cropOutside != covers);
        if (keep)
            output.appendPoint(input, idx);
    }
}

void CropFilter::crop(const cropfilter::Point& point, double distance, PointView& input, PointView& output)
{

    bool bIs3D = point.is3d();

    if (bIs3D)
    {
        KD3Index index(input);
        index.build();
        std::vector<PointId> points = index.radius(point.x, point.y, point.z, m_distance);
        for (PointId idx = 0; idx < points.size(); ++idx)
        {
            if (!m_cropOutside)
                output.appendPoint(input, idx);
        }
    }

    else
    {
        KD2Index index(input);
        index.build();
        std::vector<PointId> points = index.radius(point.x, point.y, m_distance);

        for (PointId idx = 0; idx < points.size(); ++idx)
        {
            if (!m_cropOutside)
                output.appendPoint(input, idx);
        }

    }
}


} // namespace pdal
