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

#include <pdal/PointView.hpp>
#include <pdal/private/gdal/Raster.hpp>
#include <pdal/util/Utils.hpp>

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
    : m_blockReader(*this)
{}

GDALReader::~GDALReader()
{
    m_raster.reset();
}


void GDALReader::initialize()
{
    m_raster.reset(new gdal::Raster(m_filename));
    if (m_raster->open() == gdal::GDALError::CantOpen)
        throwError("Couldn't open raster file '" + m_filename + "'.");

    m_raster->open();
    setSpatialReference(m_raster->getSpatialRef());

    m_width = m_raster->width();
    m_height = m_raster->height();
    m_bandTypes = m_raster->getPDALDimensionTypes();
    m_metadata.add(m_raster->getMetadata());
    m_blockReader.initialize();

    m_dimNames.clear();
    if (m_header.size())
    {
        m_dimNames = Utils::split(m_header, ',');
        if (m_dimNames.size() != m_bandTypes.size())
            throwError("Dimension names are not the same count as "
                "raster bands.");
    }
    else
    {
        for (size_t i = 0; i < m_bandTypes.size(); ++i)
            m_dimNames.push_back("band_" + std::to_string(i + 1));
    }

    int zBand = 1;
    for (size_t i = 0; i < m_bandIds.size(); ++i)
        if (m_bandIds[i] == Dimension::Id::Z)
        {
            zBand = i + 1;
            break;
        }

    // Bounds is only used in inspect.  We calculate it here so that
    // the raster can be released.
    m_bounds = m_raster->bounds(zBand);

    m_raster.reset();
}

QuickInfo GDALReader::inspect()
{
    QuickInfo qi;

    initialize();

    qi.m_pointCount = m_width * m_height;
    qi.m_srs = getSpatialReference();
    qi.m_bounds = m_bounds;
    qi.m_valid = true;
    qi.m_dimNames = m_dimNames;

    return qi;
}


void GDALReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(pdal::Dimension::Id::X);
    layout->registerDim(pdal::Dimension::Id::Y);

    for (size_t i = 0; i < m_bandTypes.size(); ++i)
    {
        const std::string& name = m_dimNames[i];
        Dimension::Type type = m_bandTypes[i];
        m_bandIds.push_back(layout->registerOrAssignDim(name, type));
    }
}


void GDALReader::addArgs(ProgramArgs& args)
{
    args.add("header", "A comma-separated list of dimension IDs to map "
        "raster bands to dimension id", m_header);
    args.add("memorycopy", "Load the given raster file "
        "entirely to memory", m_useMemoryCopy, false).setHidden();
}


void GDALReader::ready(PointTableRef table)
{
    m_raster.reset(new gdal::Raster(m_filename));
    if (m_raster->open() == gdal::GDALError::CantOpen)
        throwError("Couldn't open raster file '" + m_filename + "'.");

    if (m_useMemoryCopy)
    {
        gdal::Raster *r = m_raster->memoryCopy();
        if (r)
            m_raster.reset(r);
        else
            log()->get(LogLevel::Warning) << "Couldn't create raster memory "
                "copy.  Using standard interface.";
    }

    m_blockReader.initialize();
}

point_count_t GDALReader::read(PointViewPtr view, point_count_t numPts)
{
    point_count_t cnt = 0;
    while (cnt < numPts)
    {
        point_count_t processed = m_blockReader.processBlock(view);
        if (processed == 0)
            break;
        cnt += processed;
    }
    return cnt;
}

bool GDALReader::processOne(PointRef& point)
{
    return m_blockReader.processOne(point);
}


void GDALReader::done(PointTableRef table)
{
    m_raster->close();
}

GDALReader::BlockReader::BlockReader(GDALReader& reader): m_reader(reader) {}

void GDALReader::BlockReader::initialize()
{
    m_blockCol = 0;
    m_blockRow = 0;
    m_reader.m_raster->getBlockSize(0, m_blockWidth, m_blockHeight);
    m_numBlocksX = (m_reader.m_width + m_blockWidth - 1) / m_blockWidth;
    m_numBlocksY = (m_reader.m_height + m_blockHeight - 1) / m_blockHeight;
    m_needsRead = true;
    m_colInBlock = 0;
    m_rowInBlock = 0;
}

bool GDALReader::BlockReader::readBlock()
{
    m_needsRead = false;
    if (m_blockRow >= m_numBlocksY)
    {
        return false; // done
    }

    m_currentBlock.m_blockCol = m_blockCol;
    m_currentBlock.m_blockRow = m_blockRow;
    m_currentBlock.m_data.resize(m_reader.m_raster->bandCount());

    int readCol = m_blockCol * m_blockWidth;
    int readRow = m_blockRow * m_blockHeight;

    for (int band = 0; band < m_reader.m_raster->bandCount(); ++band)
    {
        if (m_reader.m_raster->read(
                band, readCol, readRow, m_blockWidth, m_blockHeight,
                m_currentBlock.m_data.at(band)) != gdal::GDALError::None)
        {
            return false;
        }
    }

    m_blockCol++;
    if (m_blockCol >= m_numBlocksX)
    {
        m_blockCol = 0;
        m_blockRow++;
    }

    return true;
}

point_count_t GDALReader::BlockReader::processBlock(PointViewPtr view)
{
    if (!readBlock())
    {
        return 0;
    }

    point_count_t cnt = 0;

    int readCol = m_currentBlock.m_blockCol * m_blockWidth;
    int readRow = m_currentBlock.m_blockRow * m_blockHeight;

    for (int rowInBlock = 0; rowInBlock < m_blockHeight; ++rowInBlock)
    {
        int row = rowInBlock + readRow;
        // We need to check for invalid indices because block sizes don't
        // have to divide the raster size evenly
        if (row >= m_reader.m_height)
            break;

        int rowOffset = rowInBlock * m_blockWidth;
        for (int colInBlock = 0; colInBlock < m_blockWidth; ++colInBlock)
        {
            int col = colInBlock + readCol;
            // We need to check for invalid indices because block sizes don't
            // have to divide the raster size evenly
            if (col >= m_reader.m_width)
                break;

            PointRef point = view->point(view->size());
            std::array<double, 2> coords;
            m_reader.m_raster->pixelToCoord(col, row, coords);
            point.setField(Dimension::Id::X, coords[0]);
            point.setField(Dimension::Id::Y, coords[1]);
            for (int band = 0; band < m_currentBlock.m_data.size(); ++band)
            {
                Dimension::Id id = m_reader.m_bandIds[band];
                point.setField(id, m_currentBlock.m_data.at(band).at(
                                       rowOffset + colInBlock));
            }
            cnt++;
        }
    }

    return cnt;
}

bool GDALReader::BlockReader::processOne(PointRef& point)
{
    if (m_needsRead)
    {
        if (!readBlock())
        {
            return false; // done
        }
    }

    int sample = m_currentBlock.m_blockCol * m_blockWidth + m_colInBlock;
    int line = m_currentBlock.m_blockRow * m_blockHeight + m_rowInBlock;

    std::array<double, 2> coords;
    m_reader.m_raster->pixelToCoord(sample, line, coords);
    point.setField(Dimension::Id::X, coords[0]);
    point.setField(Dimension::Id::Y, coords[1]);
    for (int band = 0; band < m_currentBlock.m_data.size(); ++band)
    {
        Dimension::Id id = m_reader.m_bandIds[band];
        point.setField(id, m_currentBlock.m_data.at(band).at(
                               (m_rowInBlock * m_blockWidth) + m_colInBlock));
    }

    m_colInBlock++;
    // Need to check if col in block or col in raster is out of bounds
    if (m_colInBlock >= m_blockWidth || sample + 1 >= m_reader.m_width)
    {
        m_colInBlock = 0;
        m_rowInBlock++;
        // Need to check if row in block or row in raster is out of bounds
        if (m_rowInBlock >= m_blockHeight || line + 1 >= m_reader.m_height)
        {
            // end of block, need to read a new block
            m_rowInBlock = 0;
            m_needsRead = true;
        }
    }

    return true;
}

} // namespace pdal

