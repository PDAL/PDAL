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
#include <algorithm>


#include <pdal/PointView.hpp>
#include <pdal/GlobalEnvironment.hpp>
#include <pdal/pdal_macros.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
        "readers.gdal",
        "Read GDAL rasters as point clouds.",
        "http://pdal.io/stages/reader.gdal.html");


CREATE_STATIC_PLUGIN(1, 0, GDALReader, Reader, s_info);


std::string GDALReader::getName() const
{
    return s_info.name;
}


GDALReader::GDALReader()
    : m_index(0)
{}


void GDALReader::initialize()
{
    GlobalEnvironment::get().wakeGDALDrivers();
    m_raster.reset(new gdal::Raster(m_filename));

    m_raster->open();
    setSpatialReference(m_raster->getSpatialRef());
    m_count = m_raster->m_raster_x_size * m_raster->m_raster_y_size;
    m_raster->close();
}


QuickInfo GDALReader::inspect()
{
    QuickInfo qi;
    std::unique_ptr<PointLayout> layout(new PointLayout());

    addDimensions(layout.get());
    initialize();

    m_raster = std::unique_ptr<gdal::Raster>(new gdal::Raster(m_filename));
    m_raster->open();

    qi.m_pointCount = m_raster->m_raster_x_size * m_raster->m_raster_y_size;
    // qi.m_bounds = ???;
    qi.m_srs = m_raster->getSpatialRef();
    qi.m_valid = true;

    return qi;
}


void GDALReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(pdal::Dimension::Id::X);
    layout->registerDim(pdal::Dimension::Id::Y);
    for (int i = 0; i < m_raster->m_band_count; ++i)
    {
        std::ostringstream oss;
        oss << "band-" << (i + 1);
        layout->registerOrAssignDim(oss.str(), Dimension::Type::Double);
    }
}


void GDALReader::ready(PointTableRef table)
{
    m_index = 0;
    m_raster->open();
}


point_count_t GDALReader::read(PointViewPtr view, point_count_t num)
{
    point_count_t count = std::min(num, m_count - m_index);
    PointId nextId = view->size();

    std::array<double, 2> coords;
    for (int row = 0; row < m_raster->m_raster_y_size; ++row)
    {
        for (int col = 0; col < m_raster->m_raster_x_size; ++col)
        {
            m_raster->pixelToCoord(col, row, coords);
            view->setField(Dimension::Id::X, nextId, coords[0]);
            view->setField(Dimension::Id::Y, nextId, coords[1]);
            nextId++;
        }
    }

    std::vector<uint8_t> band;
    std::vector<Dimension::Type::Enum> band_types =
        m_raster->getPDALDimensionTypes();

    for (int b = 0; b < m_raster->m_band_count; ++b)
    {
        // Bands count from 1
        m_raster->readBand(band, b + 1);
        std::stringstream oss;
        oss << "band-" << (b + 1);
        log()->get(LogLevel::Info) << "Read band '" << oss.str() << "'" <<
            std::endl;

        Dimension::Id::Enum d = view->layout()->findDim(oss.str());
        size_t dimSize = Dimension::size(band_types[b]);
        uint8_t* p = band.data();
        for (point_count_t i = 0; i < count; ++i)
        {
            view->setField(d, band_types[b], i, p);
            p = p + dimSize;
        }
    }

    return view->size();
}

} // namespace pdal

