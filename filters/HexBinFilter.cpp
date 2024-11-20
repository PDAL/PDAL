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

#include "HexBinFilter.hpp"

#include "private/hexer/HexGrid.hpp"
#include "private/hexer/H3grid.hpp"

#include "../kernels/private/density/OGR.hpp"
#include <pdal/Polygon.hpp>

using namespace hexer;

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.hexbin",
    "Tessellate the point's X/Y domain and determine point density and/or point boundary.",
    "http://pdal.io/stages/filters.hexbin.html" );

CREATE_STATIC_STAGE(HexBin, s_info)

HexBin::HexBin()
{}


HexBin::~HexBin()
{}

std::string HexBin::getName() const
{
    return s_info.name;
}


hexer::BaseGrid *HexBin::grid() const
{
    return m_grid.get();
}


void HexBin::addArgs(ProgramArgs& args)
{
    args.add("sample_size", "Sample size for auto-edge length calculation",
        m_sampleSize, 5000U);
    args.add("threshold", "Required cell density", m_density, 15);
    args.add("output_tesselation", "Write tesselation to output metadata",
        m_outputTesselation);
    args.add("edge_size", "Synonym for 'edge_length' (deprecated)",
        m_edgeLength);
    args.add("edge_length", "Length of hex edge", m_edgeLength);
    args.add("precision", "Output precision", m_precision, 8U);
    m_cullArg = &args.add("hole_cull_area_tolerance", "Tolerance area to "
        "apply to holes before cull", m_cullArea);
    args.add("smooth", "Smooth boundary output", m_doSmooth, true);
    args.add("preserve_topology", "Preserve topology when smoothing",
        m_preserve_topology, true);
    args.add("density", "Emit a density tessellation to a specified OGR-compatible output file. "
        "Defaults to GeoJSON unless 'ogrdriver' option is set.", m_DensityOutput, "");
    args.add("boundary", "Emit a boundary tessellation to a specified OGR-compatible output file. "
        "Defaults to GeoJSON unless 'ogrdriver' option is set.", m_boundaryOutput, "");
    args.add("h3_grid", "Create a grid using H3 (https://h3geo.org/docs) Hexagons",
        m_isH3, false);
    args.add("h3_resolution", "H3 grid resolution: 0 (coarsest) - 15 (finest). See "
        "https://h3geo.org/docs/core-library/restable", m_h3Res, -1);
    args.add("ogrdriver", "GDAL OGR vector driver for writing with 'density' or 'boundary' "
        "options.", m_driver, "GeoJSON");
}


void HexBin::initialize()
{
    if (m_isH3)
    {
        if (m_edgeLength) {
            if (m_h3Res == -1)
                throwError("'edge_length' not implemented for H3 processing. "
                    "Set 'h3_resolution' option to specify cell size.");
            else
                log()->get(LogLevel::Warning) << "'edge_length' not implemented "
                    "for H3 processing. Using 'h3_resolution'\n";
        }
        if (m_cullArea || !m_preserve_topology)
            throwError("Smoothing not implemented for H3 processing. "
                "'preserve_topology' and 'hole_cull_area_tolerance' "
                "options are ignored.");
    }
    if (!m_isH3 && (m_h3Res != -1))
    {
        if (!m_edgeLength)
            throwError("'h3_resolution' not implemented for standard "
                "processing. Set 'edge_length' option to specify cell size.");
        else
            log()->get(LogLevel::Warning) << "'h3_resolution' not implemented "
                "for standard processing. Using 'edge_length'\n"; 
    }
}


void HexBin::ready(PointTableRef table)
{
    m_count = 0;
    if (m_isH3)
    {
        if (m_h3Res == -1)
        {
            m_grid.reset(new H3Grid(m_density));
            m_grid->setSampleSize(m_sampleSize);
        }
        else
            m_grid.reset(new H3Grid(m_h3Res, m_density));
    }
    else
    {
        if (m_edgeLength == 0.0)
        {
            m_grid.reset(new HexGrid(m_density));
            m_grid->setSampleSize(m_sampleSize);
        }
        else
            m_grid.reset(new HexGrid(m_edgeLength * sqrt(3), m_density));
    }
}


void HexBin::filter(PointView& view)
{
    PointRef p(view, 0);

    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        p.setPointId(idx);
        processOne(p);
    }
}


bool HexBin::processOne(PointRef& point)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    m_grid->addXY(x, y);
    m_count++;
    return true;
}


void HexBin::spatialReferenceChanged(const SpatialReference& srs)
{
    m_srs = srs;
    if (!m_grid->checkSRS(m_srs)) {
        std::ostringstream oss;
        oss << "Cannot find H3 hexbin locations with spatial reference: ("
            << m_srs.getProj4() << ")! Input must be EPSG:4326";
        throwError(oss.str());
    }
}


void HexBin::done(PointTableRef table)
{
    if (m_grid->sampling())
    {
        std::ostringstream oss;
        oss << "Sampling for hexbin auto-edge length calculation failed! ";
        if (m_sampleSize > m_count)
            oss << "Decrease sample size: sample size of " << m_sampleSize 
                << " with " << m_count << " points.";
        throwError(oss.str());
    }

    try
    {
        m_grid->findShapes();
        m_grid->findParentPaths();
    }
    catch (hexer::hexer_error& e)
    {
        m_metadata.add("error", e.what(),
            "Hexer threw an error and was unable to compute a boundary");
        m_metadata.add("boundary", "MULTIPOLYGON EMPTY",
            "Empty polygon -- unable to compute boundary");
        return;
    }

    m_metadata.add("threshold", m_grid->denseLimit(),
        "Minimum number of points inside a hexagon to be considered full");
    m_metadata.add("sample_size", m_sampleSize, "Number of samples to use "
        "when estimating hexagon edge size. Specify 0.0 or omit options "
        "for edge_size if you want to compute one.");

    Utils::OStringStreamClassicLocale polygon;
    polygon.setf(std::ios_base::fixed, std::ios_base::floatfield);
    polygon.precision(m_precision);
    m_grid->toWKT(polygon);

    if (m_outputTesselation)
    {
        m_metadata.add("hex_boundary", polygon.str(),
            "Boundary MULTIPOLYGON of domain");
    }

    // density and boundary writing with OGR does not support polygon smoothing
    if (m_DensityOutput.size())
    {
        OGR writer(m_DensityOutput, m_srs.getWKT(), m_driver, "hexbins");
        writer.writeDensity(*m_grid);
    }
    if (m_boundaryOutput.size())
    {
        OGR writer(m_boundaryOutput, m_srs.getWKT(), m_driver, "hexbins");
        writer.writeBoundary(*m_grid); 
    }

    pdal::Polygon p(polygon.str(), m_srs);

    // If the SRS was geographic, use relevant
    // UTM for area and density computation
    Polygon density_p(p);
    if (m_srs.isGeographic())
    {
        // Compute a UTM polygon
        BOX3D box = p.bounds();
        int zone = SpatialReference::calculateZone(box.minx, box.miny);
        if (!density_p.transform(SpatialReference::wgs84FromZone(zone)))
            density_p = Polygon();
    }

    double area = density_p.area();
    double density = m_count / area;
    if (std::isinf(density))
    {
        density = -1.0;
        area = -1.0;
    }

    m_metadata.add("density", density,
        "Number of points per square unit (total area)");
    m_metadata.add("area", area, "Area in square units of tessellated polygon");
    m_metadata.add("avg_pt_spacing", std::sqrt(1 / density),
        "Avg point spacing (x/y units)");

    int n(0);
    point_count_t totalCount(0);
    for (auto& [coord, count] : m_grid->getHexes())
    {
        if (m_grid->isDense(coord))
        {
            totalCount += count;
            n++;
        }
    }

    // what's the purpose of this? rename it?
    double hexArea(((3 * SQRT_3)/2.0) * (m_grid->height() * m_grid->height()));
    double avg_density = (n * hexArea) / totalCount;
    m_metadata.add("avg_pt_per_sq_unit", avg_density, "Area / point count "
        "(ignore contrary metadata item name. This is '(n * hexArea) / totalCount')");

    if (!m_isH3)
    {
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
        if (m_doSmooth)
        {
            double tolerance = 1.1 * m_grid->height() / 2;
            double cull = m_cullArg->set() ?
                m_cullArea : (6 * tolerance * tolerance);
            p.simplify(tolerance, cull, m_preserve_topology);
        }

        Utils::OStringStreamClassicLocale offsets;
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
        m_metadata.add("hex_offsets", offsets.str(), "Offset of hex corners from "
            "hex centers.");
    }
    else
    {
        m_metadata.add("h3_resolution", m_grid->getRes(), "The H3 resolution level "
            "of the grid. See https://h3geo.org/docs/core-library/restable "
            "for more information" );
    }
    
    m_metadata.add("boundary", p.wkt(m_precision),
        "Approximated MULTIPOLYGON of domain");
    m_metadata.addWithType("boundary_json", p.json(), "json",
        "Approximated MULTIPOLYGON of domain");
}

} // namespace pdal
