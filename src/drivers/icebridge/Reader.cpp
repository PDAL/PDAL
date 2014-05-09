/******************************************************************************
* Copyright (c) 2014, Connor Manning, connor@hobu.co
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

#include <pdal/drivers/icebridge/Reader.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>

#include <algorithm>
#include <map>

namespace
{
    const std::vector<pdal::hdf5::Hdf5ColumnData> hdf5Columns =
    {
        { "instrument_parameters/time_hhmmss",  H5::PredType::NATIVE_FLOAT },
        { "latitude",                           H5::PredType::NATIVE_FLOAT },
        { "longitude",                          H5::PredType::NATIVE_FLOAT },
        { "elevation",                          H5::PredType::NATIVE_FLOAT },
        { "instrument_parameters/xmt_sigstr",   H5::PredType::NATIVE_INT   },
        { "instrument_parameters/rcv_sigstr",   H5::PredType::NATIVE_INT   },
        { "instrument_parameters/azimuth",      H5::PredType::NATIVE_FLOAT },
        { "instrument_parameters/pitch",        H5::PredType::NATIVE_FLOAT },
        { "instrument_parameters/roll",         H5::PredType::NATIVE_FLOAT },
        { "instrument_parameters/gps_pdop",     H5::PredType::NATIVE_FLOAT },
        { "instrument_parameters/pulse_width",  H5::PredType::NATIVE_FLOAT },
        { "instrument_parameters/rel_time",     H5::PredType::NATIVE_FLOAT }
    };
}

namespace pdal
{
namespace drivers
{
namespace icebridge
{

Reader::Reader(const Options& options)
    : pdal::Reader(options)
{
    setSchema(Schema(getDefaultDimensions()));
}

void Reader::initialize()
{
    Hdf5Handler hdf5Handler;
    hdf5Handler.initialize(getFileName(), hdf5Columns);
    setNumPoints(hdf5Handler.getNumPoints());
    hdf5Handler.close();

    const std::vector<Dimension> dimensions(getDefaultDimensions());

    // Populate our mapping of HDF5 column names to their Dimensions.
    if (dimensions.size() == hdf5Columns.size())
    {
        for (std::size_t i = 0; i < dimensions.size(); ++i)
        {
            m_dimensionNamesMap.insert(
                    std::make_pair(hdf5Columns[i].name, dimensions[i]));
        }
    }
    else
    {
        throw icebridge_error("Inconsistent number of columns");
    }
}

Options Reader::getDefaultOptions()
{
    Options options;
    Option filename("filename", "", "file to read from");
    options.add(filename);
    return options;
}

std::vector<Dimension> Reader::getDefaultDimensions()
{
    std::vector<Dimension> output;

    Dimension time("Time", dimension::Float, 4,
            "Relative Time (seconds from start of data file)");
    time.setUUID("4a1e8fcb-321d-41d6-a0fb-b9cb8bad9216");
    time.setNamespace(s_getName());
    output.push_back(time);

    Dimension y("Y", dimension::Float, 4,
            "Laser Spot Latitude (degrees).");
    y.setUUID("e2a153ff-5717-425d-9965-2ed744611c44");
    y.setNamespace(s_getName());
    output.push_back(y);

    Dimension x("X", dimension::Float, 4,
            "Laser Spot Longitude (degrees).");
    x.setUUID("577bbb27-263b-431b-beef-3fae07042207");
    x.setNamespace(s_getName());
    output.push_back(x);

    Dimension z("Z", dimension::Float, 4,
            "Elevation (meters)");
    z.setUUID("68e255e6-5463-4f2e-9b35-8b7c159a6a79");
    z.setNamespace(s_getName());
    output.push_back(z);

    Dimension startPulse("StartPulse", dimension::SignedInteger, 4,
            "Start Pulse Signal Strength (relative)");
    startPulse.setUUID("37963e29-baf3-49f8-97e0-9c901fe9a611");
    startPulse.setNamespace(s_getName());
    output.push_back(startPulse);

    Dimension reflectedPulse("ReflectedPulse", dimension::SignedInteger, 4,
            "Reflected Pulse Signal Strength (relative)");
    reflectedPulse.setUUID("ae4bbfbd-a275-46e3-b1b7-acd1d8a6e7ae");
    reflectedPulse.setNamespace(s_getName());
    output.push_back(reflectedPulse);

    Dimension scanAngle("ScanAngleRank", dimension::Float, 4,
            "Scan Azimuth (degrees)");
    scanAngle.setUUID("5d75f366-075e-4e5a-b7c0-0627eddd529e");
    scanAngle.setNamespace(s_getName());
    output.push_back(scanAngle);

    Dimension pitch("Pitch", dimension::Float, 4,
            "Pitch (degrees)");
    pitch.setUUID("6720a2cd-7ccf-4c19-a71c-48bfb77abf5a");
    pitch.setNamespace(s_getName());
    output.push_back(pitch);

    Dimension roll("Roll", dimension::Float, 4,
            "Roll (degrees)");
    roll.setUUID("28565c97-aaae-4099-9b8c-4f38630993da");
    roll.setNamespace(s_getName());
    output.push_back(roll);
                    
    Dimension pdop("PDOP", dimension::Float, 4,
            "GPS PDOP (dilution of precision)");
    pdop.setUUID("34db8b9c-f488-4c8a-a93f-a96ff74b1d8d");
    pdop.setNamespace(s_getName());
    output.push_back(pdop);

    Dimension width("PulseWidth", dimension::Float, 4,
            "Laser Received Pulse Width (digitizer samples)");
    width.setUUID("4f66ef25-97e2-4f6e-8335-ad706b9164dc");
    width.setNamespace(s_getName());
    output.push_back(width);

    Dimension gpsTime("GpsTime", dimension::Float, 4,
            "GPS Time packed (seconds) "
            "(example: 153320.100 = 15h 33m 20s 100m)");
    gpsTime.setUUID("d8868d61-49b9-4c7b-a75b-6acd8b26818e");
    gpsTime.setNamespace(s_getName());
    output.push_back(gpsTime);

    return output;
}

std::string Reader::getFileName() const
{
    return getOptions().getOption("filename").getValue<std::string>();
}

pdal::StageSequentialIterator* 
Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::icebridge::iterators::sequential::Iterator(
                *this,
                buffer);
}

pdal::StageRandomIterator*
Reader::createRandomIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::icebridge::iterators::random::Iterator(
                *this,
                buffer);
}

std::map<std::string, Dimension> Reader::getDimensionNamesMap() const
{
    return m_dimensionNamesMap;
}

namespace iterators
{

IteratorBase::IteratorBase(const pdal::drivers::icebridge::Reader& reader)
    : m_numPoints(reader.getNumPoints())
    , m_dimensionNamesMap(reader.getDimensionNamesMap())
    , m_schema(reader.getSchema())
    , m_hdf5Handler()
{
    m_hdf5Handler.initialize(reader.getFileName(), hdf5Columns);
}

boost::uint32_t IteratorBase::readIcebridgeIntoBuffer(
        PointBuffer& pointBuffer,
        const boost::uint64_t index)
{
    const boost::uint64_t numPoints64 =
        std::min<boost::uint64_t>(
                pointBuffer.getCapacity(), m_numPoints - index);

    const boost::uint32_t numPoints =
        boost::numeric_cast<boost::uint32_t>(std::min<boost::uint64_t>(
                numPoints64, std::numeric_limits<boost::uint32_t>::max()));

    PointBuffer tmpPointBuffer(m_schema, numPoints);

    for (auto column : hdf5Columns)
    {
        const Dimension dimension =
            m_dimensionNamesMap.find(column.name)->second;

        unsigned char* rawData = static_cast<unsigned char*>(
                ::operator new(numPoints * dimension.getByteSize()));

        try
        {
            m_hdf5Handler.getColumnEntries(
                    rawData,
                    column.name,
                    numPoints,
                    index);

            for (std::size_t i = 0; i < numPoints; ++i)
            {
                tmpPointBuffer.setRawField(
                        m_schema.getDimension(dimension.getName()),
                        i,
                        rawData + dimension.getByteSize() * i);
            }

            delete[] rawData;
        }
        catch(...)
        {
            delete[] rawData;
            throw icebridge_error("Error fetching column data");
        }
    }

    boost::scoped_ptr<schema::DimensionMap> dimensionMap(
        tmpPointBuffer.getSchema().mapDimensions(pointBuffer.getSchema()));

    PointBuffer::copyLikeDimensions(
            tmpPointBuffer,
            pointBuffer,
            *dimensionMap.get(),
            0,
            pointBuffer.getNumPoints(),
            numPoints);

    pointBuffer.setNumPoints(pointBuffer.getNumPoints() + numPoints);

    return numPoints;
}

namespace sequential
{

Iterator::Iterator(
        const pdal::drivers::icebridge::Reader& reader,
        PointBuffer& pointBuffer)
    : pdal::ReaderSequentialIterator(pointBuffer)
    , IteratorBase(reader)
{ }

boost::uint64_t Iterator::skipImpl(boost::uint64_t count)
{
    const boost::uint64_t skipped =
        std::min<boost::uint64_t>(count, m_numPoints - m_index);

    m_index += skipped;

    return skipped;
}

boost::uint32_t Iterator::readBufferImpl(PointBuffer& data)
{
    return readIcebridgeIntoBuffer(data, m_index);
}

bool Iterator::atEndImpl() const
{
    return m_index >= m_numPoints;
}

} // namespace sequential

namespace random
{

Iterator::Iterator(
        const pdal::drivers::icebridge::Reader& reader,
        PointBuffer& pointBuffer)
    : pdal::ReaderRandomIterator(pointBuffer)
    , IteratorBase(reader)
    { }

boost::uint32_t Iterator::readBufferImpl(PointBuffer& data)
{
    return readIcebridgeIntoBuffer(data, m_index);
}

boost::uint64_t Iterator::seekImpl(const boost::uint64_t numSeek)
{
    const boost::uint64_t skipped =
        std::min<boost::uint64_t>(numSeek, m_numPoints - m_index);

    m_index += skipped;

    return skipped;
}

} // namespace random

} // iterators

} // icebridge
} // drivers
} // pdal

