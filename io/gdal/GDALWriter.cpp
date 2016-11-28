/******************************************************************************
* Copyright (c) 2016, Hobu Inc. <info@hobu.co>
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
*     * Neither the name of Hobu, Inc. nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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

#include "GDALWriter.hpp"

#include <sstream>

#include <pdal/GDALUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/pdal_macros.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
        "writers.gdal",
        "Write a point cloud as a GDAL raster.",
        "http://pdal.io/stages/writers.gdal.html");


CREATE_STATIC_PLUGIN(1, 0, GDALWriter, Writer, s_info);

std::string GDALWriter::getName() const
{
    return s_info.name;
}


void GDALWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("resolution", "Cell edge size, in units of X/Y",
        m_edgeLength).setPositional();
    args.add("radius", "Radius from cell center to use to locate influencing "
        "points", m_radius).setPositional();
    args.add("gdaldriver", "GDAL writer driver name", m_drivername, "GTiff");
    args.add("gdalopts", "GDAL driver options (name=value,name=value...)",
        m_options);
    args.add("output_type", "Statistics produced ('min', 'max', 'mean', "
        "'idw', 'count', 'stdev' or 'all')", m_outputTypeString, {"all"} );
    args.add("window_size", "Cell distance for fallback interpolation",
        m_windowSize);
    args.add("nodata", "No data value", m_noData, -9999.0);
    args.add("dimension", "Dimension to use", m_interpDimString, "Z");
}


void GDALWriter::initialize()
{
    for (auto& ts : m_outputTypeString)
    {
       Utils::trim(ts);
        if (ts == "all")
        {
            m_outputTypes = ~0;
            break;
        }
        if (ts == "min")
            m_outputTypes |= GDALGrid::statMin;
        else if (ts == "max")
            m_outputTypes |= GDALGrid::statMax;
        else if (ts == "count")
            m_outputTypes |= GDALGrid::statCount;
        else if (ts == "mean")
            m_outputTypes |= GDALGrid::statMean;
        else if (ts == "idw")
            m_outputTypes |= GDALGrid::statIdw;
        else if (ts == "stdev")
            m_outputTypes |= GDALGrid::statStdDev;
        else
        {
            std::ostringstream oss;
            oss << "Invalid writers.gdal output type: '" << ts << "'.";
            throw pdal_error(oss.str());
        }
    }

    gdal::registerDrivers();
}


void GDALWriter::prepared(PointTableRef table)
{
    m_interpDim = table.layout()->findDim(m_interpDimString);
    if (m_interpDim == Dimension::Id::Unknown)
    {
        std::ostringstream oss;

        oss << getName() << ": specified dimension '" << m_interpDimString <<
            "' does not exist.";
        throw pdal_error(oss.str());
    }
}


void GDALWriter::ready(PointTableRef table)
{
    if (!table.spatialReferenceUnique())
    {
        std::ostringstream oss;

        oss << getName() << ": Can't write output with multiple spatial "
            "references.";
        throw pdal_error(oss.str());
    }
}


void GDALWriter::write(const PointViewPtr view)
{
    view->calculateBounds(m_bounds);
    size_t width = ((m_bounds.maxx - m_bounds.minx) / m_edgeLength) + 1;
    size_t height = ((m_bounds.maxy - m_bounds.miny) / m_edgeLength) + 1;
    m_grid.reset(new GDALGrid(width, height, m_edgeLength, m_radius, m_noData,
        m_outputTypes, m_windowSize));

    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        double x = view->getFieldAs<double>(Dimension::Id::X, idx) -
            m_bounds.minx;
        double y = view->getFieldAs<double>(Dimension::Id::Y, idx) -
            m_bounds.miny;
        double z = view->getFieldAs<double>(m_interpDim, idx);

        m_grid->addPoint(x, y, z);
   }
}


void GDALWriter::done(PointTableRef table)
{
    std::array<double, 6> pixelToPos;

    pixelToPos[0] = m_bounds.minx;
    pixelToPos[1] = m_edgeLength;
    pixelToPos[2] = 0;
    pixelToPos[3] = m_bounds.miny + (m_edgeLength * m_grid->height());
    pixelToPos[4] = 0;
    pixelToPos[5] = -m_edgeLength;
    gdal::Raster raster(m_filename, m_drivername, table.spatialReference(),
        pixelToPos);

    m_grid->finalize();

    gdal::GDALError err = raster.open(m_grid->width(), m_grid->height(),
        m_grid->numBands(), Dimension::Type::Double, m_grid->noData());
    if (err != gdal::GDALError::None)
        throw pdal_error(raster.errorMsg());
    int bandNum = 1;
    uint8_t *buf;
    buf = m_grid->data("min");
    if (buf)
        raster.writeBand(buf, bandNum++, "min");
    buf = m_grid->data("max");
    if (buf)
        raster.writeBand(buf, bandNum++, "max");
    buf = m_grid->data("mean");
    if (buf)
        raster.writeBand(buf, bandNum++, "mean");
    buf = m_grid->data("idw");
    if (buf)
        raster.writeBand(buf, bandNum++, "idw");
    buf = m_grid->data("count");
    if (buf)
        raster.writeBand(buf, bandNum++, "count");
    buf = m_grid->data("stdev");
    if (buf)
        raster.writeBand(buf, bandNum++, "stdev");

    getMetadata().addList("filename", m_filename);
}

} // namespace pdal

