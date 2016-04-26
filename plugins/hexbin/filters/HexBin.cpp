/******************************************************************************
* Copyright (c) 2013, Andrew Bell (andrew.bell.ia@gmail.com)
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

#include "HexBin.hpp"

#include <hexer/HexIter.hpp>
#include <pdal/Polygon.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_macros.hpp>

using namespace hexer;

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.hexbin",
    "Tessellate the point's X/Y domain and determine point density and/or point boundary.",
    "http://pdal.io/stages/filters.hexbin.html" );

CREATE_SHARED_PLUGIN(1, 0, HexBin, Filter, s_info)

void HexBin::processOptions(const Options& options)
{
    m_sampleSize = options.getValueOrDefault<uint32_t>("sample_size", 5000);
    m_density = options.getValueOrDefault<uint32_t>("threshold", 15);
    m_outputTesselation = options.getValueOrDefault<bool>("output_tesselation", false);

    if (options.hasOption("edge_length"))
        m_edgeLength = options.getValueOrDefault<double>("edge_length", 0.0);
    else
        // Backward compatability.
        m_edgeLength = options.getValueOrDefault<double>("edge_size", 0.0);
}


void HexBin::ready(PointTableRef table)
{
    m_count = 0;
    if (m_edgeLength == 0.0)  // 0 can always be represented exactly.
    {
        m_grid.reset(new HexGrid(m_density));
        m_grid->setSampleSize(m_sampleSize);
    }
    else
        m_grid.reset(new HexGrid(m_edgeLength * sqrt(3), m_density));
}


void HexBin::filter(PointView& view)
{
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        double x = view.getFieldAs<double>(pdal::Dimension::Id::X, idx);
        double y = view.getFieldAs<double>(pdal::Dimension::Id::Y, idx);
        m_grid->addPoint(x, y);
    }
    m_count += view.size();
}


void HexBin::done(PointTableRef table)
{
    m_grid->processSample();
    m_grid->findShapes();
    m_grid->findParentPaths();

    std::ostringstream offsets;
    offsets << "MULTIPOINT (";
    for (int i = 0; i < 6; ++i)
    {
        hexer::Point p = m_grid->offset(i);
        offsets << p.m_x << " " << p.m_y;
        if (i != 5)
            offsets << ", ";
    }
    offsets << ")";

    m_metadata.add("edge_length", m_edgeLength, "The edge length of the "
        "hexagon to use in situations where you do not want to estimate "
        "based on a sample");
    m_metadata.add("estimated_edge", m_grid->height(),
        "Estimated computed edge distance");
    m_metadata.add("threshold", m_grid->denseLimit(),
        "Minimum number of points inside a hexagon to be considered full");
    m_metadata.add("sample_size", m_sampleSize, "Number of samples to use "
        "when estimating hexagon edge size. Specify 0.0 or omit options "
        "for edge_size if you want to compute one.");
    m_metadata.add("hex_offsets", offsets.str(), "Offset of hex corners from "
        "hex centers.");

    uint32_t precision  = m_options.getValueOrDefault<uint32_t>("precision", 8);
    std::ostringstream polygon;
    polygon.setf(std::ios_base::fixed, std::ios_base::floatfield);
    polygon.precision(precision);
    m_grid->toWKT(polygon);

    if (m_outputTesselation)
    {
        MetadataNode hexes = m_metadata.add("hexagons");
        for (HexIter hi = m_grid->hexBegin(); hi != m_grid->hexEnd(); ++hi)
        {
            HexInfo h = *hi;

            MetadataNode hex = hexes.addList("hexagon");
            hex.add("density", h.density());

            hex.add("gridpos", Utils::toString(h.xgrid()) + " " +
                Utils::toString((h.ygrid())));
            std::ostringstream oss;
            // Using stream limits precision (default 6)
            oss << "POINT (" << h.x() << " " << h.y() << ")";
            hex.add("center", oss.str());
        }
        m_metadata.add("hex_boundary", polygon.str(),
            "Boundary MULTIPOLYGON of domain");
    }


    /***
      We want to make these bumps on edges go away, which means that
      we want to elimnate both B and C.  If we take a line from A -> C,
      we need the tolerance to eliminate B.  After that we're left with
      the triangle ACD and we want to eliminate C.  The perpendicular
      distance from AD to C is the hexagon height / 2, so we set the
      tolerance a little larger than that.  This is larger than the
      perpendicular distance needed to eliminate B in ABC, so should
      serve for both cases.

         B ______  C
          /      \
       A /        \ D

    ***/
    double tolerance = 1.1 * m_grid->height() / 2;

    double cull =
        m_options.getValueOrDefault<double>("hole_cull_area_tolerance",
            6 * tolerance * tolerance);

    SpatialReference srs(table.anySpatialReference());
    pdal::Polygon p(polygon.str(), srs);
    pdal::Polygon density_p(polygon.str(), srs);

    // If the SRS was geographic, use relevant
    // UTM for area and density computation
    if (srs.isGeographic())
    {
        // Compute a UTM polygon
        BOX3D box = p.bounds();
        int zone = SpatialReference::calculateZone(box.minx, box.miny);

        auto makezone = [] (int zone) -> std::string
        {

            std::ostringstream z;

            // Use WGS84 UTM zones
            z << "EPSG:327" << abs(zone);
            return z.str();
        };

        SpatialReference utm(makezone(zone));
        density_p = p.transform(utm);
    }
    pdal::Polygon smooth = p.simplify(tolerance, cull);
    std::string smooth_text = smooth.wkt(precision);

    m_metadata.add("boundary", smooth_text, "Approximated MULTIPOLYGON of domain");
    double area = density_p.area();

//    double density = (double) m_grid->densePointCount() / area ;
    double density = (double) m_count/ area ;
    m_metadata.add("density", density, "Number of points per square unit");
    m_metadata.add("area", area, "Area in square units of tessellated polygon");
}

} // namespace pdal
