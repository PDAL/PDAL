/******************************************************************************
* Copyright (c) 2015, Howard Butler <howard@hobu.co>
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

#include "GDALReader.hpp"

#include <sstream>

#include <pdal/GDALUtils.hpp>
#include <pdal/PointView.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "readers.gdal",
    "Read GDAL rasters as point clouds.",
    "http://pdal.io/stages/reader.gdal.html",
    { "tif", "tiff", "jpeg", "jpg", "png" }

};

CREATE_STATIC_STAGE(GDALReader, s_info)


std::string GDALReader::getName() const
{
    return s_info.name;
}


GDALReader::GDALReader()
    : m_index(0)
{}


void GDALReader::initialize()
{
    gdal::registerDrivers();
    m_raster.reset(new gdal::Raster(m_filename));

    m_raster->open();
    try
    {
        setSpatialReference(m_raster->getSpatialRef());
    }
    catch (...)
    {
        log()->get(LogLevel::Error) << "Could not create an SRS" << std::endl;
    }

    m_count = m_raster->width() * m_raster->height();
    m_bandTypes = m_raster->getPDALDimensionTypes();
    m_raster->close();

}


QuickInfo GDALReader::inspect()
{
    QuickInfo qi;
    std::unique_ptr<PointLayout> layout(new PointLayout());

    initialize();
    addDimensions(layout.get());

    m_raster = std::unique_ptr<gdal::Raster>(new gdal::Raster(m_filename));
    if (m_raster->open() == gdal::GDALError::CantOpen)
        throwError("Couldn't open raster file '" + m_filename + "'.");

    qi.m_pointCount = m_raster->width() * m_raster->height();

    auto p = std::find(m_bandIds.begin(), m_bandIds.end(), Dimension::Id::Z);

    int nBand(1);
    if (p != m_bandIds.end())
    {
        nBand = (int) std::distance(m_bandIds.begin(), p);
    }

    qi.m_bounds = m_raster->bounds(nBand);
    qi.m_srs = m_raster->getSpatialRef();
    qi.m_valid = true;

    return qi;
}


void GDALReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(pdal::Dimension::Id::X);
    layout->registerDim(pdal::Dimension::Id::Y);
    std::vector<std::string> dimNames;
    if (m_header.size())
    {
        dimNames = Utils::split(m_header, ',');
        if (dimNames.size() != (size_t)m_raster->bandCount())
            throwError("Dimension names are not the same count as raster bands!");
    }

    for (int i = 0; i < m_raster->bandCount(); ++i)
    {
        std::ostringstream oss;
        oss << "band-" << (i + 1);
        std::string name(dimNames.empty() ? oss.str() : dimNames[i]);
        Dimension::Id id = layout->registerOrAssignDim(name, m_bandTypes[i]);
        m_bandIds.push_back(id);
    }
}

void GDALReader::addArgs(ProgramArgs& args)
{
    args.add("header", "A comma-separated list of dimension IDs to map raster bands to dimension id", m_header);
}


void GDALReader::ready(PointTableRef table)
{
    m_index = 0;
    if (m_raster->open() == gdal::GDALError::CantOpen)
        throwError("Couldn't open raster file '" + m_filename + "'.");
    m_row = 0;
    m_col = 0;
}


point_count_t GDALReader::read(PointViewPtr view, point_count_t numPts)
{
    PointId idx = view->size();
    point_count_t cnt = 0;
    PointRef point(*view, idx);
    while (cnt < numPts)
    {
        point.setPointId(idx);
        if (!processOne(point))
            break;
        cnt++;
        idx++;
    }
    return cnt;
}


bool GDALReader::processOne(PointRef& point)
{

    std::array<double, 2> coords;
    if (m_row == m_raster->height() &&
        m_col == m_raster->width())
        return false; // done

    if (m_col == m_raster->width())
    {
        m_col = 0;
        m_row ++;
    }

    m_raster->pixelToCoord(m_col, m_row, coords);
    double x = coords[0];
    double y = coords[1];
    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);

    std::vector<double> data;
    if (m_raster->read(x, y, data) != gdal::GDALError::None)
        return false;

    for (int b = 0; b < m_raster->bandCount(); ++b)
    {
        Dimension::Id id = m_bandIds[b];
        double v = data[b];
        point.setField(id, v);
    }

    m_col++;


    return true;
}

} // namespace pdal

