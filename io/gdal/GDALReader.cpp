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
    GlobalEnvironment::get().initializeGDAL(log());
    if (!m_raster)
        m_raster = std::unique_ptr<gdal::Raster>(new gdal::Raster(m_filename));

    m_raster->open();
    setSpatialReference(m_raster->getSpatialRef());
    m_count = m_raster->m_raster_x_size * m_raster->m_raster_y_size;

}

void GDALReader::processOptions(const Options& options)
{

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
//     qi.m_bounds = m_lasHeader.getBounds();
    qi.m_srs = m_raster->getSpatialRef();
    qi.m_valid = true;


    return qi;
}


void GDALReader::addDimensions(PointLayoutPtr layout)
{
    int nBands = m_raster->m_band_count;

    layout->registerDim(pdal::Dimension::Id::X);
    layout->registerDim(pdal::Dimension::Id::Y);
    for (int i=0; i < nBands; ++i)
    {
        std::ostringstream oss;
        oss << "band-" << i+1;
        layout->registerOrAssignDim(oss.str(), Dimension::Type::Double);
    }
}


void GDALReader::ready(PointTableRef table)
{
    m_index = 0;


}

template<typename T>
double convert(uint8_t*p)
{
    double output;

    T t;
    std::copy(p, p + sizeof(t), (uint8_t*)&t);
    output = static_cast<double>(t);
    return output;
}

point_count_t GDALReader::read(PointViewPtr view, point_count_t num)
{
    point_count_t count = std::min(num, m_count - m_index);
    PointId nextId = view->size();

    std::array<double, 2> coords;
    for (int row = 0; row < m_raster->m_raster_y_size; ++row)
    {
        for(int col = 0; col < m_raster->m_raster_x_size; ++col)
        {
            m_raster->pixelToCoord(row, col, coords);
            view->setField(Dimension::Id::X, nextId, coords[0]);
            view->setField(Dimension::Id::Y, nextId, coords[1]);
            nextId++;
        }
    }

    std::vector<uint8_t> band;
    std::vector<pdal::Dimension::Type::Enum> band_types = m_raster->getPDALDimensionTypes();
    for (int b = 0; b < m_raster->m_band_count; ++b)
    {
        // Bands count from 1
        // read up the band's bytes into an array,
        // convert them from GDAL native types into
        // doubles, and stuff them into the view
        m_raster->readBand(band, b+1);
        std::stringstream oss;
        oss << "band-" << b+1;
        log()->get(LogLevel::Info) << "Read band '" << oss.str() <<"'" << std::endl;
        pdal::Dimension::Id::Enum d = view->layout()->findProprietaryDim(oss.str());
        size_t dimSize = pdal::Dimension::size(band_types[b]); // count from 0

        uint8_t* p = band.data();
        for (point_count_t i = 0; i < count; ++i)
        {

            if (band_types[b] == pdal::Dimension::Type::Float ||
                band_types[b] == pdal::Dimension::Type::Double )
            {
                if (dimSize == 4)
                    view->setField(d, i, convert<float>(p));
                else
                    view->setField(d, i, convert<double>(p));
            }
            else
            {
                if (dimSize == 1)
                {
                    view->setField(d, i, convert<uint8_t>(p));
                }
                else if (dimSize == 2)
                {
                    view->setField(d, i, convert<uint16_t>(p));
                }
                else if (dimSize == 4)
                {
                    view->setField(d, i, convert<uint32_t>(p));
                }
            }
            p = p + dimSize;
        }
    }

    return view->size();
}


}
