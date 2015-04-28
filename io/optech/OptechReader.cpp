/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski <pete.gadomski@gmail.com>
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

#define _USE_MATH_DEFINES
#include "OptechReader.hpp"

#include <cmath>
#include <cstring>
#include <sstream>

#include <pdal/SpatialReference.hpp>
#include <pdal/StageFactory.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.optech",
    "Optech reader support.",
    "http://pdal.io/stages/readers.optech.html" );

CREATE_STATIC_PLUGIN(1, 0, OptechReader, Reader, s_info);

std::string OptechReader::getName() const
{
    return s_info.name;
}

#ifndef _WIN32
const size_t OptechReader::MaximumNumberOfReturns;
const size_t OptechReader::MaxNumRecordsInBuffer;
const size_t OptechReader::NumBytesInRecord;
#endif

OptechReader::OptechReader()
    : Reader()
    , m_header()
    , m_boresightMatrix(georeference::createIdentityMatrix())
    , m_istream()
    , m_buffer()
    , m_extractor(m_buffer.data(), 0)
    , m_recordIndex(0)
    , m_returnIndex(0)
    , m_pulse()
{
    // The Optech docs say that their lat/longs are referenced
    // to the WGS84 reference frame.
    SpatialReference spatialReference;
    spatialReference.setFromUserInput("EPSG:4326");
    setSpatialReference(spatialReference);
}


Dimension::IdList OptechReader::getDefaultDimensions()
{
    Dimension::IdList dims;
    dims.push_back(Dimension::Id::X);
    dims.push_back(Dimension::Id::Y);
    dims.push_back(Dimension::Id::Z);
    dims.push_back(Dimension::Id::GpsTime);
    dims.push_back(Dimension::Id::ReturnNumber);
    dims.push_back(Dimension::Id::NumberOfReturns);
    dims.push_back(Dimension::Id::EchoRange);
    dims.push_back(Dimension::Id::Intensity);
    dims.push_back(Dimension::Id::ScanAngleRank);
    return dims;
}


const CsdHeader& OptechReader::getHeader() const { return m_header; }


void OptechReader::initialize()
{
    ILeStream stream(FileUtils::openFile(m_filename));
    if (!stream)
    {
        std::stringstream ss;
        ss << "Unable to open " << m_filename << " for reading.";
        throw pdal_error(ss.str());
    }

    stream.get(m_header.signature, 4);
    if (strcmp(m_header.signature, "CSD") != 0)
    {
        std::stringstream ss;
        ss << "Invalid header signature when reading CSD file: '"
           << m_header.signature << "'";
        throw optech_error(ss.str());
    }
    stream.get(m_header.vendorId, 64);
    stream.get(m_header.softwareVersion, 32);
    stream >> m_header.formatVersion >> m_header.headerSize >>
        m_header.gpsWeek >> m_header.minTime >> m_header.maxTime >>
        m_header.numRecords >> m_header.numStrips;
    for (size_t i = 0; i < 256; ++i)
    {
        stream >> m_header.stripPointers[i];
    }
    stream >> m_header.misalignmentAngles[0] >>
        m_header.misalignmentAngles[1] >> m_header.misalignmentAngles[2] >>
        m_header.imuOffsets[0] >> m_header.imuOffsets[1] >>
        m_header.imuOffsets[2] >> m_header.temperature >> m_header.pressure;
    stream.get(m_header.freeSpace, 830);

    m_boresightMatrix = createOptechRotationMatrix(
        m_header.misalignmentAngles[0] + m_header.imuOffsets[0],
        m_header.misalignmentAngles[1] + m_header.imuOffsets[1],
        m_header.misalignmentAngles[2] + m_header.imuOffsets[2]);
}


void OptechReader::addDimensions(PointLayoutPtr layout)
{
    for (auto it : getDefaultDimensions())
    {
        layout->registerDim(it);
    }
}


void OptechReader::ready(PointTableRef)
{
    m_istream.reset(new IStream(m_filename));
    if (!*m_istream)
    {
        std::stringstream ss;
        ss << "Unable to open " << m_filename << " for reading.";
        throw pdal_error(ss.str());
    }

    m_istream->seek(m_header.headerSize);
    m_recordIndex = 0;
    m_returnIndex = 0;
    m_pulse = CsdPulse();
}


point_count_t OptechReader::read(PointViewPtr data,
                                 point_count_t countRequested)
{
    point_count_t numRead = 0;
    point_count_t dataIndex = data->size();

    while (numRead < countRequested)
    {
        if (m_returnIndex == 0)
        {
            if (!m_extractor.good())
            {
                if (m_recordIndex >= m_header.numRecords)
                {
                    break;
                }
                m_recordIndex += fillBuffer();
            }

            m_extractor >> m_pulse.gpsTime >> m_pulse.returnCount >>
                m_pulse.range[0] >> m_pulse.range[1] >> m_pulse.range[2] >>
                m_pulse.range[3] >> m_pulse.intensity[0] >>
                m_pulse.intensity[1] >> m_pulse.intensity[2] >>
                m_pulse.intensity[3] >> m_pulse.scanAngle >> m_pulse.roll >>
                m_pulse.pitch >> m_pulse.heading >> m_pulse.latitude >>
                m_pulse.longitude >> m_pulse.elevation;

            if (m_pulse.returnCount == 0)
            {
                m_returnIndex = 0;
                continue;
            }

            // In all the csd files that we've tested, the longitude
            // values have been less than -2pi.
            if (m_pulse.longitude < -M_PI * 2)
            {
                m_pulse.longitude = m_pulse.longitude + M_PI * 2;
            }
            else if (m_pulse.longitude > M_PI * 2)
            {
                m_pulse.longitude = m_pulse.longitude - M_PI * 2;
            }
        }

        georeference::Xyz gpsPoint = georeference::Xyz(
            m_pulse.longitude, m_pulse.latitude, m_pulse.elevation);
        georeference::RotationMatrix rotationMatrix =
            createOptechRotationMatrix(m_pulse.roll, m_pulse.pitch,
                                       m_pulse.heading);
        georeference::Xyz point = pdal::georeference::georeferenceWgs84(
            m_pulse.range[m_returnIndex], m_pulse.scanAngle,
            m_boresightMatrix, rotationMatrix, gpsPoint);

        data->setField(Dimension::Id::X, dataIndex, point.X * 180 / M_PI);
        data->setField(Dimension::Id::Y, dataIndex, point.Y * 180 / M_PI);
        data->setField(Dimension::Id::Z, dataIndex, point.Z);
        data->setField(Dimension::Id::GpsTime, dataIndex, m_pulse.gpsTime);
        if (m_returnIndex == MaximumNumberOfReturns - 1)
        {
            data->setField(Dimension::Id::ReturnNumber, dataIndex,
                          m_pulse.returnCount);
        }
        else
        {
            data->setField(Dimension::Id::ReturnNumber, dataIndex,
                          m_returnIndex + 1);
        }
        data->setField(Dimension::Id::NumberOfReturns, dataIndex,
                      m_pulse.returnCount);
        data->setField(Dimension::Id::EchoRange, dataIndex,
                      m_pulse.range[m_returnIndex]);
        data->setField(Dimension::Id::Intensity, dataIndex,
                      m_pulse.intensity[m_returnIndex]);
        data->setField(Dimension::Id::ScanAngleRank, dataIndex,
                      m_pulse.scanAngle * 180 / M_PI);

        if (m_cb)
            m_cb(*data, dataIndex);

        ++dataIndex;
        ++numRead;
        ++m_returnIndex;

        if (m_returnIndex >= m_pulse.returnCount ||
            m_returnIndex >= MaximumNumberOfReturns)
        {
            m_returnIndex = 0;
        }
    }
    return numRead;
}


size_t OptechReader::fillBuffer()
{
    size_t numRecords = std::min<size_t>(m_header.numRecords - m_recordIndex,
                                         MaxNumRecordsInBuffer);

    buffer_size_t bufferSize = NumBytesInRecord * numRecords;
    m_buffer.resize(bufferSize);
    m_istream->get(m_buffer);
    m_extractor = LeExtractor(m_buffer.data(), m_buffer.size());
    return numRecords;
}


void OptechReader::done(PointTableRef)
{
    m_istream.reset();
}

} // namespace pdal

