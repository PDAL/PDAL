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
    m_raster->close();
}


QuickInfo GDALReader::inspect()
{
    QuickInfo qi;
    std::unique_ptr<PointLayout> layout(new PointLayout());

    addDimensions(layout.get());
    initialize();

    m_raster = std::unique_ptr<gdal::Raster>(new gdal::Raster(m_filename));
    if (m_raster->open() == gdal::GDALError::CantOpen)
        throwError("Couldn't open raster file '" + m_filename + "'.");

    qi.m_pointCount = m_raster->width() * m_raster->height();
    // qi.m_bounds = ???;
    qi.m_srs = m_raster->getSpatialRef();
    qi.m_valid = true;

    return qi;
}


void GDALReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(pdal::Dimension::Id::X);
    layout->registerDim(pdal::Dimension::Id::Y);
    for (int i = 0; i < m_raster->bandCount(); ++i)
    {
        std::ostringstream oss;
        oss << "band-" << (i + 1);
        layout->registerOrAssignDim(oss.str(), Dimension::Type::Double);
    }
}


void GDALReader::ready(PointTableRef table)
{
    m_index = 0;
    if (m_raster->open() == gdal::GDALError::CantOpen)
        throwError("Couldn't open raster file '" + m_filename + "'.");
}


point_count_t GDALReader::read(PointViewPtr view, point_count_t num)
{
    point_count_t count = std::min(num, m_count - m_index);
    PointId nextId = view->size();

    std::array<double, 2> coords;
    for (int row = 0; row < m_raster->height(); ++row)
    {
        for (int col = 0; col < m_raster->width(); ++col)
        {
            m_raster->pixelToCoord(col, row, coords);
            view->setField(Dimension::Id::X, nextId, coords[0]);
            view->setField(Dimension::Id::Y, nextId, coords[1]);
            nextId++;
        }
    }

    std::vector<uint8_t> band;
    std::vector<Dimension::Type> band_types =
        m_raster->getPDALDimensionTypes();

    for (int b = 0; b < m_raster->bandCount(); ++b)
    {
        // Bands count from 1
        switch (band_types[b])
        {
        case Dimension::Type::Signed8:
            readBandData<int8_t>(b + 1, view, count);
            break;
        case Dimension::Type::Unsigned8:
            readBandData<uint8_t>(b + 1, view, count);
            break;
        case Dimension::Type::Signed16:
            readBandData<int16_t>(b + 1, view, count);
            break;
        case Dimension::Type::Unsigned16:
            readBandData<uint16_t>(b + 1, view, count);
            break;
        case Dimension::Type::Signed32:
            readBandData<int32_t>(b + 1, view, count);
            break;
        case Dimension::Type::Unsigned32:
            readBandData<uint32_t>(b + 1, view, count);
            break;
        case Dimension::Type::Signed64:
            readBandData<int64_t>(b + 1, view, count);
            break;
        case Dimension::Type::Unsigned64:
            readBandData<uint64_t>(b + 1, view, count);
            break;
        case Dimension::Type::Float:
            readBandData<float>(b + 1, view, count);
            break;
        case Dimension::Type::Double:
            readBandData<double>(b + 1, view, count);
            break;
        case Dimension::Type::None:
            break;
        }
    }
    return view->size();
}


template<typename T>
void GDALReader::readBandData(int band, PointViewPtr view, point_count_t count)
{
    std::vector<T> buf;

    m_raster->readBand(buf, band);
    std::stringstream oss;
    oss << "band-" << (band);
    log()->get(LogLevel::Info) << "Read band '" << oss.str() << "'" <<
       std::endl;
    Dimension::Id d = view->layout()->findDim(oss.str());
    for (point_count_t i = 0; i < count; ++i)
        view->setField(d, i, buf[i]);
}

} // namespace pdal

