/******************************************************************************
* Copyright (c) 2022, Howard Butler (info@hobu.co)
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

#include "GeomDistanceFilter.hpp"

#include <string>
#include <vector>

#include <pdal/Geometry.hpp>
#include <pdal/Polygon.hpp>
#include <pdal/private/OGRSpec.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>

#include <nlohmann/json.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.geomdistance",
    "Compute the distance for points to a given geometry",
    "http://pdal.io/stages/filters.geomdistance.html"
};

CREATE_STATIC_STAGE(GeomDistanceFilter, s_info)


struct GeomDistanceArgs
{
    Dimension::Id m_dim;
    std::string m_dimName;
    pdal::Geometry m_geometry;
    bool m_doRingMode;
    OGRSpec m_ogr;

};


GeomDistanceFilter::GeomDistanceFilter() : m_args(new GeomDistanceArgs)
{}

GeomDistanceFilter::~GeomDistanceFilter()
{}

std::string GeomDistanceFilter::getName() const
{
    return s_info.name;
}

void GeomDistanceFilter::addDimensions(PointLayoutPtr layout)
{
    m_args->m_dim = layout->registerOrAssignDim(m_args->m_dimName,
            Dimension::Type::Double);
}

void GeomDistanceFilter::initialize()
{
    gdal::registerDrivers();
}

void GeomDistanceFilter::addArgs(ProgramArgs& args)
{
    args.add("geometry", "Geometries to test", m_args->m_geometry).
        setErrorText("Invalid polygon specification.  "
            "Must be valid GeoJSON/WKT");
    args.add("dimension", "Dimension to create to place distance values", m_args->m_dimName, "distance");
    args.add("ring", "Compare edges (demote polygons to linearrings)", m_args->m_doRingMode, false);
    args.add("ogr", "OGR filter geometries", m_args->m_ogr);


}


void GeomDistanceFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    m_args->m_dim = layout->findDim(m_args->m_dimName);
    if (m_args->m_dim == Dimension::Id::Unknown)
        throwError("Missing dimension with name '" + m_args->m_dimName +
            "'in input PointView.");

}

void GeomDistanceFilter::ready(PointTableRef table)
{
    if (!m_args->m_ogr.empty())
        m_args->m_geometry = m_args->m_ogr.getPolygons()[0];

    if (m_args->m_doRingMode)
        m_args->m_geometry = m_args->m_geometry.getRing();

    if (!m_args->m_geometry.getOGRHandle())
        throwError("Candidate polygon in filters.geomdistance was NULL!");
}


void GeomDistanceFilter::filter(PointView& view)
{
    PointRef point = view.point(0);
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
}

bool GeomDistanceFilter::processOne(PointRef& point)
{
    static std::vector<double> data;

    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);


    double distance = m_args->m_geometry.distance(x, y, z);
    point.setField(m_args->m_dim, distance);

    return true;
}


} // namespace pdal
