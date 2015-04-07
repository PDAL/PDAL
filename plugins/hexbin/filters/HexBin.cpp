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
#include <pdal/StageFactory.hpp>

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
    m_metadata.add("threshold", m_density, "Minimum number of points inside "
        "a hexagon to be considered full");
    m_metadata.add("sample_size", m_sampleSize, "Number of samples to use "
        "when estimating hexagon edge size. Specify 0.0 or omit options "
        "for edge_size if you want to compute one.");
    m_metadata.add("hex_offsets", offsets.str(), "Offset of hex corners from "
        "hex centers.");
    if (m_outputTesselation)
    {

        MetadataNode hexes = m_metadata.add("hexagons");
        for (HexIter hi = m_grid->hexBegin(); hi != m_grid->hexEnd(); ++hi)
        {
            using namespace boost;

            HexInfo h = *hi;

            MetadataNode hex = hexes.addList("hexagon");
            hex.add("density", h.density());

            hex.add("gridpos", lexical_cast<std::string>(h.xgrid()) + " " +
                lexical_cast<std::string>(h.ygrid()));
            std::ostringstream oss;
            // Using stream limits precision (default 6)
            oss << "POINT (" << h.x() << " " << h.y() << ")";
            hex.add("center", oss.str());
        }
    }

    std::ostringstream polygon;
    polygon.setf(std::ios_base::fixed, std::ios_base::floatfield);
    polygon.precision(m_options.getValueOrDefault<uint32_t>("precision", 8));
    m_grid->toWKT(polygon);
    m_metadata.add("boundary", polygon.str(),
        "Boundary MULTIPOLYGON of domain");
}

} // namespace pdal
