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

#include <pdal/GDALUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Polygon.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <filters/private/crop/Point.hpp>

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

CropFilter::CropFilter() : m_cropOutside(false)
{}

CropFilter::~CropFilter()
{}

void CropFilter::addArgs(ProgramArgs& args)
{
    args.add("outside", "Whether we keep points inside or outside of the "
        "bounding region", m_cropOutside);
    args.add("a_srs", "Spatial reference for bounding region", m_assignedSrs);
    args.add("bounds", "Point box for cropped points", m_bounds);
    args.add("point", "Center of circular/spherical crop region.  Use with "
        "'distance'.", m_centers).setErrorText("Invalid point specification.  "
            "Must be valid GeoJSON/WKT. "
            "Ex: \"(1.00, 1.00)\" or \"(1.00, 1.00, 1.00)\"");
    args.add("distance", "Crop with this distance from 2D or 3D 'point'",
        m_distance);
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
            // Throws if invalid.
            poly.valid();
            m_geoms.push_back(poly);
        }
    }
    m_distance2 = m_distance * m_distance;
}


void CropFilter::ready(PointTableRef table)
{
    // If the user didn't provide an SRS, take one from the table.
    if (m_assignedSrs.empty())
        m_assignedSrs = table.anySpatialReference();
    for (auto& geom : m_geoms)
        geom.setSpatialReference(m_assignedSrs);
}


bool CropFilter::processOne(PointRef& point)
{
    for (auto& geom : m_geoms)
        if (!crop(point, geom))
            return false;

    for (auto& box : m_bounds)
        if (!crop(point, box.to2d()))
            return false;

    for (auto& center: m_centers)
        if (!crop(point, center))
            return false;

    return true;
}


void CropFilter::spatialReferenceChanged(const SpatialReference& srs)
{
    transform(srs);
}


void CropFilter::transform(const SpatialReference& srs)
{
    // If we don't have any SRS, do nothing.
    for (auto& geom : m_geoms)
    {
        try
        {
            geom = geom.transform(srs);
        }
        catch (pdal_error& err)
        {
            throwError(err.what());
        }
    }

    if (srs.empty() && m_assignedSrs.empty())
        return;
    if (srs.empty() || m_assignedSrs.empty())
        throwError("Unable to transform crop geometry to point "
            "coordinate system.");

    for (auto& box : m_bounds)
    {
        BOX3D b3d = box.to3d();
        gdal::reprojectBounds(b3d, m_assignedSrs.getWKT(), srs.getWKT());
        box = b3d;
    }
    for (auto& point : m_centers)
    {
        gdal::reprojectPoint(point.x, point.y, point.z,
            m_assignedSrs.getWKT(), srs.getWKT());
    }
    m_assignedSrs = srs;
}


PointViewSet CropFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;

    transform(view->spatialReference());
    for (auto& geom : m_geoms)
    {
        PointViewPtr outView = view->makeNew();
        crop(geom, *view, *outView);
        viewSet.insert(outView);
    }

    for (auto& box : m_bounds)
    {
        PointViewPtr outView = view->makeNew();
        crop(box.to2d(), *view, *outView);
        viewSet.insert(outView);
    }

    for (auto& point: m_centers)
    {
        PointViewPtr outView = view->makeNew();
        crop(point, *view, *outView);
        viewSet.insert(outView);
    }

    return viewSet;
}


bool CropFilter::crop(const PointRef& point, const BOX2D& box)
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
        if (m_cropOutside != crop(point, box))
            output.appendPoint(input, idx);
    }
}


bool CropFilter::crop(const PointRef& point, const Polygon& g)
{
    return (m_cropOutside != g.covers(point));
}


void CropFilter::crop(const Polygon& g, PointView& input, PointView& output)
{
    PointRef point = input.point(0);
    for (PointId idx = 0; idx < input.size(); ++idx)
    {
        point.setPointId(idx);
        if (crop(point, g))
            output.appendPoint(input, idx);
    }
}


bool CropFilter::crop(const PointRef& point, const cropfilter::Point& center)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    x -= center.x;
    y -= center.y;
    if (x > m_distance || y > m_distance)
        return (m_cropOutside);

    bool inside;
    if (center.is3d())
    {
        double z = point.getFieldAs<double>(Dimension::Id::Z);
        z -= center.z;
        if (z > m_distance)
            return (m_cropOutside);
        inside = (x * x + y * y + z * z < m_distance2);
    }
    else
        inside = (x * x + y * y < m_distance2);
    return (m_cropOutside != inside);
}


void CropFilter::crop(const cropfilter::Point& center, PointView& input,
    PointView& output)
{
    PointRef point = input.point(0);
    for (PointId idx = 0; idx < input.size(); ++idx)
    {
        point.setPointId(idx);
        if (crop(point, center))
            output.appendPoint(input, idx);
    }
}

} // namespace pdal
