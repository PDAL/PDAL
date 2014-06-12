/******************************************************************************
* Copyright (c) 2014, Peter J. Gadomski (pete.gadomski@gmail.com)
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

#include <pdal/drivers/sbet/Reader.hpp>

#include <pdal/drivers/sbet/Common.hpp>


namespace pdal
{
namespace drivers
{
namespace sbet
{


Reader::Reader(const Options& options)
    : pdal::Reader(options)
{
    setSchema(Schema(getDefaultDimensions()));
}


Reader::~Reader()
{}


void Reader::initialize()
{
    boost::uintmax_t fileSize = FileUtils::fileSize(getFileName());
    if (fileSize % pdal::drivers::sbet::pointByteSize != 0)
    {
        throw pdal_error("invalid sbet file size");
    }
    else
    {
        setNumPoints(fileSize / pdal::drivers::sbet::pointByteSize);
    }
}


Options Reader::getDefaultOptions()
{
    Options options;
    return options;
}


std::vector<Dimension> Reader::getDefaultDimensions()
{
    std::vector<Dimension> output;

    Dimension time("Time", dimension::Float, 8);
    time.setUUID("D228D324-B40E-4C85-A0DB-ED741A665AFA");
    time.setNamespace(s_getName());
    output.push_back(time);

    Dimension y("Y", dimension::Float, 8);
    y.setUUID("A1322536-C086-42C8-924A-A9B3BB290A1E");
    y.setNamespace(s_getName());
    output.push_back(y);

    Dimension x("X", dimension::Float, 8);
    x.setUUID("57E88FE8-DD3C-4232-A70A-C840AC7129F3");
    x.setNamespace(s_getName());
    output.push_back(x);

    Dimension z("Z", dimension::Float, 8);
    z.setUUID("7344D3CF-A943-4CA7-90BD-5B55838E610A");
    z.setNamespace(s_getName());
    output.push_back(z);

    Dimension xVelocity("XVelocity", dimension::Float, 8);
    xVelocity.setUUID("0F482EC5-7F9B-44E2-A9D6-4248D99C7E82");
    xVelocity.setNamespace(s_getName());
    output.push_back(xVelocity);

    Dimension yVelocity("YVelocity", dimension::Float, 8);
    yVelocity.setUUID("C221E33D-6AD9-4FE0-8090-D7BDD3826E51");
    yVelocity.setNamespace(s_getName());
    output.push_back(yVelocity);

    Dimension zVelocity("ZVelocity", dimension::Float, 8);
    zVelocity.setUUID("B073200F-80EE-4DE1-BEE9-0B982695EDFA");
    zVelocity.setNamespace(s_getName());
    output.push_back(zVelocity);

    Dimension roll("Roll", dimension::Float, 8);
    roll.setUUID("C2BD0053-34AB-4699-8AFF-0EA7DBC015E7");
    roll.setNamespace(s_getName());
    output.push_back(roll);

    Dimension pitch("Pitch", dimension::Float, 8);
    pitch.setUUID("CD305166-B474-4047-AA93-7F2C98B22532");
    pitch.setNamespace(s_getName());
    output.push_back(pitch);

    Dimension platformHeading("PlatformHeading", dimension::Float, 8);
    platformHeading.setUUID("F619BC08-3FF8-40ED-8DEB-4F18EF695C0C");
    platformHeading.setNamespace(s_getName());
    output.push_back(platformHeading);

    Dimension wanderAngle("WanderAngle", dimension::Float, 8);
    wanderAngle.setUUID("E9B6061B-B221-47C8-905C-0E6CA266727C");
    wanderAngle.setNamespace(s_getName());
    output.push_back(wanderAngle);

    Dimension xBodyAccel("XBodyAccel", dimension::Float, 8);
    xBodyAccel.setUUID("24781F89-083C-43BD-9F14-CAF801D70E03");
    xBodyAccel.setNamespace(s_getName());
    output.push_back(xBodyAccel);

    Dimension yBodyAccel("YBodyAccel", dimension::Float, 8);
    yBodyAccel.setUUID("6F3CC468-8871-4E54-8B8B-E66DEA16A90A");
    yBodyAccel.setNamespace(s_getName());
    output.push_back(yBodyAccel);

    Dimension zBodyAccel("ZBodyAccel", dimension::Float, 8);
    zBodyAccel.setUUID("23A5AF6E-97B2-4BDF-9565-B0CBEF5F4BD1");
    zBodyAccel.setNamespace(s_getName());
    output.push_back(zBodyAccel);

    Dimension xBodyAngRate("XBodyAngRate", dimension::Float, 8);
    xBodyAngRate.setUUID("34593AC6-2E46-4E45-BE7E-60B027012052");
    xBodyAngRate.setNamespace(s_getName());
    output.push_back(xBodyAngRate);

    Dimension yBodyAngRate("YBodyAngRate", dimension::Float, 8);
    yBodyAngRate.setUUID("C76A0C32-57FA-4007-BFC7-C7867D50E9AC");
    yBodyAngRate.setNamespace(s_getName());
    output.push_back(yBodyAngRate);

    Dimension zBodyAngRate("ZBodyAngRate", dimension::Float, 8);
    zBodyAngRate.setUUID("A30B9BB8-363D-467A-933A-B1EBD227AA5C");
    zBodyAngRate.setNamespace(s_getName());
    output.push_back(zBodyAngRate);

    return output;
}


std::string Reader::getFileName() const
{
    return getOptions().getOption("filename").getValue<std::string>();
}


pdal::StageSequentialIterator*
Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::sbet::iterators::sequential::Iterator(*this, buffer);
}


pdal::StageRandomIterator*
Reader::createRandomIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::sbet::iterators::random::Iterator(*this, buffer);
}


namespace iterators
{


IteratorBase::IteratorBase(const pdal::drivers::sbet::Reader& reader,
                           PointBuffer& buffer)
    : m_numPoints(reader.getNumPoints())
    , m_schema(reader.getSchema())
    , m_readBuffer(pdal::PointBuffer(m_schema, m_numPoints))
{
    m_istream = FileUtils::openFile(reader.getFileName());
}


IteratorBase::~IteratorBase()
{
    FileUtils::closeFile(m_istream);
}


boost::uint32_t IteratorBase::readSbetIntoBuffer(PointBuffer& data, const boost::uint64_t numPoints64)
{
    if (!m_istream->good())
    {
        throw pdal_error("sbet stream is no good");
    }

    const boost::uint32_t numPoints = (boost::uint32_t)std::min<boost::uint64_t>(numPoints64,
                                      std::numeric_limits<boost::uint32_t>::max());

    m_readBuffer.setNumPoints(0);

    schema::DimensionMap* dimensionMap = m_readBuffer.getSchema().mapDimensions(data.getSchema());

    boost::uint8_t* bufferData = m_readBuffer.getDataStart();
    if (!bufferData)
    {
        throw pdal_error("unable to access point buffer's data");
    }

    Utils::read_n(bufferData, *m_istream, numPoints * pdal::drivers::sbet::pointByteSize);
    m_readBuffer.setNumPoints(numPoints);

    PointBuffer::copyLikeDimensions(m_readBuffer, data, *dimensionMap, 0,
                                    data.getNumPoints(), numPoints);
    data.setNumPoints(data.getNumPoints() + numPoints);

    return numPoints;
}


namespace sequential
{


Iterator::Iterator(const pdal::drivers::sbet::Reader& reader,
                   PointBuffer& buffer)
    : pdal::ReaderSequentialIterator(buffer)
    , IteratorBase(reader, buffer)
{
    return;
}


Iterator::~Iterator()
{
    return;
}


boost::uint64_t Iterator::skipImpl(boost::uint64_t count)
{
    m_istream->seekg(pdal::drivers::sbet::pointByteSize * count, std::ios::cur);
    return count;
}


bool Iterator::atEndImpl() const
{
    return m_numPoints <=  getIndex();
}


boost::uint32_t Iterator::readBufferImpl(PointBuffer& data)
{
    const boost::uint64_t numPoints64 = std::min<boost::uint64_t>(data.getCapacity(),
                                        m_numPoints - getIndex());
    return readSbetIntoBuffer(data, numPoints64);
}


} // namespace sequential


namespace random
{


Iterator::Iterator(const pdal::drivers::sbet::Reader& reader,
                   PointBuffer& buffer)
    : pdal::ReaderRandomIterator(buffer)
    , IteratorBase(reader, buffer)
{
    return;
}


Iterator::~Iterator()
{
    return;
}


boost::uint32_t Iterator::readBufferImpl(PointBuffer& data)
{
    const boost::uint64_t numPoints64 = std::min<boost::uint64_t>(data.getCapacity(),
                                        m_numPoints - getIndex());
    return readSbetIntoBuffer(data, numPoints64);
}


boost::uint64_t Iterator::seekImpl(boost::uint64_t numSeek)
{
    m_istream->seekg(pdal::drivers::sbet::pointByteSize * numSeek);
    assert(m_istream->tellg() % pdal::drivers::sbet::pointByteSize == 0);
    return m_istream->tellg() / pdal::drivers::sbet::pointByteSize;
}


} // namespace random
} // namespace iterators


}
}
} // namespace pdal::drivers::sbet
