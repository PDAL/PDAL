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

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Polygon.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>

#include "private/Point.hpp"
#include "private/pnp/GridPnp.hpp"

#include <sstream>
#include <cstdarg>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.crop",
    "Filter points inside or outside a bounding box or a polygon",
    "http://pdal.io/stages/filters.crop.html"
};

CREATE_STATIC_STAGE(CropFilter, s_info)

struct CropArgs
{
    bool m_cropOutside;
    SpatialReference m_assignedSrs;
    std::vector<Bounds> m_bounds;
    std::vector<filter::Point> m_centers;
    double m_distance;
    std::vector<Polygon> m_polys;
};

CropFilter::ViewGeom::ViewGeom(const Polygon& poly) : m_poly(poly)
{}

CropFilter::ViewGeom::ViewGeom(ViewGeom&& vg) :
    m_poly(std::move(vg.m_poly)), m_gridPnps(std::move(vg.m_gridPnps))
{}

std::string CropFilter::getName() const { return s_info.name; }

CropFilter::CropFilter() : m_args(new CropArgs)
{}


CropFilter::~CropFilter()
{}


void CropFilter::addArgs(ProgramArgs& args)
{
    args.add("outside", "Whether we keep points inside or outside of the "
        "bounding region", m_args->m_cropOutside);
    args.add("a_srs", "Spatial reference for bounding region",
        m_args->m_assignedSrs);
    args.add("bounds", "Point box for cropped points", m_args->m_bounds);
    args.add("point", "Center of circular/spherical crop region.  Use with "
        "'distance'.", m_args->m_centers).
        setErrorText("Invalid point specification.  Must be valid "
            "GeoJSON/WKT. Ex: \"(1.00, 1.00)\" or \"(1.00, 1.00, 1.00)\"");
    args.add("distance", "Crop with this distance from 2D or 3D 'point'",
        m_args->m_distance);
    args.add("polygon", "Bounding polying for cropped points", m_args->m_polys).
        setErrorText("Invalid polygon specification.  "
            "Must be valid GeoJSON/WKT");
}


void CropFilter::initialize()
{
    // Set geometry from polygons.
    if (m_args->m_polys.size())
    {
        m_geoms.clear();
        for (Polygon& poly : m_args->m_polys)
        {
            // Throws if invalid.
            poly.valid();
            m_geoms.emplace_back(poly);
        }
    }

    m_boxes.clear();
    for (auto& bound : m_args->m_bounds)
        m_boxes.push_back(bound);

    m_distance2 = m_args->m_distance * m_args->m_distance;
}


void CropFilter::ready(PointTableRef table)
{
    // If the user didn't provide an SRS, take one from the table.
    if (m_args->m_assignedSrs.empty())
    {
        m_args->m_assignedSrs = table.anySpatialReference();
        if (!table.spatialReferenceUnique())
            log()->get(LogLevel::Warning) << "Can't determine spatial "
                "reference for provided bounds.  Consider using 'a_srs' "
                "option.\n";
    }
    for (auto& geom : m_geoms)
        geom.m_poly.setSpatialReference(m_args->m_assignedSrs);
}


bool CropFilter::processOne(PointRef& point)
{
    for (auto& g : m_geoms)
        for (auto& gridPnp : g.m_gridPnps)
            if (crop(point, *gridPnp))
                return true;

    for (auto& box : m_boxes)
        if (box.is3d())
        {
            if (crop(point, box.to3d()))
                return true;
        }
        else
        {
            if (crop(point, box.to2d()))
                return true;
        }


    for (auto& center: m_args->m_centers)
        if (crop(point, center))
            return true;

    return false;
}


void CropFilter::spatialReferenceChanged(const SpatialReference& srs)
{
    transform(srs);
}


void CropFilter::transform(const SpatialReference& srs)
{
    for (auto& geom : m_geoms)
    {
        auto ok = geom.m_poly.transform(srs);
        if (!ok)
            throwError(ok.what());
        geom.m_gridPnps.clear();
        std::vector<Polygon> polys = geom.m_poly.polygons();
        for (auto& p : polys)
        {
            std::unique_ptr<GridPnp> gridPnp(new GridPnp(
                p.exteriorRing(), p.interiorRings()));
            geom.m_gridPnps.push_back(std::move(gridPnp));
        }
    }

    // If we don't have any SRS, do nothing.
    if (srs.empty() && m_args->m_assignedSrs.empty())
        return;

    // Note that we should never have assigned SRS empty here since
    // if it is missing we assign it from the point data.
    assert(!m_args->m_assignedSrs.empty());
    if (srs.empty() || m_args->m_assignedSrs.empty())
        throwError("Unable to transform crop geometry to point coordinate system.");

    for (auto& box : m_boxes)
    {
        if (!gdal::reprojectBounds(box, m_args->m_assignedSrs.getWKT(),
            srs.getWKT()))
            throwError("Unable to reproject bounds.");
    }
    for (auto& point : m_args->m_centers)
    {
        point.setSpatialReference(m_args->m_assignedSrs);
        auto ok = point.transform(srs);
        if (!ok)
            throwError(ok.what());
    }
    // Set the assigned SRS for the points/bounds to the one we've
    // transformed to.
    m_args->m_assignedSrs = srs;
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

    for (auto& box : m_boxes)
    {
        PointViewPtr outView = view->makeNew();
        crop(box, *view, *outView);
        viewSet.insert(outView);
    }

    for (auto& point: m_args->m_centers)
    {
        PointViewPtr outView = view->makeNew();
        crop(point, *view, *outView);
        viewSet.insert(outView);
    }

    return viewSet;
}

bool CropFilter::crop(const PointRef& point, const BOX3D& box)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);

    // Return true if we're keeping a point.
    return (m_args->m_cropOutside != box.contains(x, y, z));
}

bool CropFilter::crop(const PointRef& point, const BOX2D& box)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);

    // Return true if we're keeping a point.
    return (m_args->m_cropOutside != box.contains(x, y));
}

void CropFilter::crop(const Bounds& box, PointView& input, PointView& output)
{
    bool is3d = box.is3d();
    if (is3d)
        crop(box.to3d(), input, output);
    else
        crop(box.to2d(), input, output);

}

void CropFilter::crop(const BOX3D& box, PointView& input, PointView& output)
{
    PointRef point = input.point(0);
    for (PointId idx = 0; idx < input.size(); ++idx)
    {
        point.setPointId(idx);
        if (crop(point, box))
            output.appendPoint(input, idx);
    }
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


bool CropFilter::crop(const PointRef& point, GridPnp& g)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    return (m_args->m_cropOutside != g.inside(x, y));
}


void CropFilter::crop(const ViewGeom& g, PointView& input, PointView& output)
{
    PointRef point = input.point(0);
    for (auto& gridPnp : g.m_gridPnps)
    {
        for (PointId idx = 0; idx < input.size(); ++idx)
        {
            point.setPointId(idx);
            if (crop(point, const_cast<GridPnp&>(*gridPnp)))
                output.appendPoint(input, idx);
        }
    }
}


bool CropFilter::crop(const PointRef& point, const filter::Point& center)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);

    x = std::abs(x - center.x());
    y = std::abs(y - center.y());
    if (x > m_args->m_distance || y > m_args->m_distance)
        return (m_args->m_cropOutside);

    bool inside;
    if (center.is3d())
    {
        double z = point.getFieldAs<double>(Dimension::Id::Z);
        z = std::abs(z - center.z());
        if (z > m_args->m_distance)
            return (m_args->m_cropOutside);
        inside = (x * x + y * y + z * z < m_distance2);
    }
    else
        inside = (x * x + y * y < m_distance2);
    return (m_args->m_cropOutside != inside);
}


void CropFilter::crop(const filter::Point& center, PointView& input,
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
