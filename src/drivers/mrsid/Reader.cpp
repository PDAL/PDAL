/******************************************************************************
* Copyright (c) 2011, Michael S. Rosen (michael.rosen@gmail.com)
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

#include <pdal/drivers/mrsid/Reader.hpp>

#include <pdal/PointBuffer.hpp>

#include <boost/algorithm/string.hpp>


namespace pdal
{
namespace drivers
{
namespace mrsid
{


Reader::Reader(const Options& options)
    : pdal::Reader(options)
{
    return;
}


Reader::Reader(LizardTech::PointSource *ps)
    : pdal::Reader(Options::none())
    , m_PS(ps), m_iter(NULL)
{
    m_PS->retain();
    return;
}

Reader::~Reader()
{
    m_iter->release();
    m_PS->release();
}

Dimension Reader::LTChannelToPDalDimension(const LizardTech::ChannelInfo & channel, pdal::Schema const& dimensions) const
{

    std::string name = channel.getName();

    // Map LT common names to PDAL common names (from drivers.las)
    if (boost::iequals(channel.getName(), CHANNEL_NAME_EdgeFlightLine)) name = "EdgeOfFlightLine";
    if (boost::iequals(channel.getName(), CHANNEL_NAME_ClassId)) name = "Classification";
    if (boost::iequals(channel.getName(), CHANNEL_NAME_ScanAngle)) name = "ScanAngleRank";
    if (boost::iequals(channel.getName(), CHANNEL_NAME_ScanDir)) name = "ScanDirectionFlag";
    if (boost::iequals(channel.getName(), CHANNEL_NAME_GPSTime)) name = "Time";
    if (boost::iequals(channel.getName(), CHANNEL_NAME_SourceId)) name = "PointSourceId";
    if (boost::iequals(channel.getName(), CHANNEL_NAME_ReturnNum)) name = "ReturnNumber";
    if (boost::iequals(channel.getName(), CHANNEL_NAME_NumReturns)) name = "NumberOfReturns";

    Dimension retval = dimensions.getDimension(name);

    return retval;
}


void Reader::initialize()
{
    const LizardTech::PointInfo& pointinfo = m_PS->getPointInfo();

    Schema const& dimensions(getDefaultDimensions());
    for (unsigned int i=0; i<pointinfo.getNumChannels(); i++)
    {
        const LizardTech::ChannelInfo &channel = pointinfo.getChannel(i);
        Dimension dim = LTChannelToPDalDimension(channel, dimensions);
        m_schema.appendDimension(dim);
    }

    setNumPoints(m_PS->getNumPoints());
    pdal::Bounds<double> b(m_PS->getBounds().x.min, m_PS->getBounds().x.max,
        m_PS->getBounds().y.min, m_PS->getBounds().y.max,
        m_PS->getBounds().z.min,m_PS->getBounds().z.max);
    setBounds(b);
    m_iter = m_PS->createIterator(m_PS->getBounds(), 1.0,
        m_PS->getPointInfo(), NULL);
}


Options Reader::getDefaultOptions()
{
    Options options;
    return options;
}

pdal::StageSequentialIterator* Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::mrsid::iterators::sequential::Reader(*this, buffer, getNumPoints());
}

int Reader::SchemaToPointInfo(const Schema &schema, LizardTech::PointInfo &pointInfo) const
{
    schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();

    pointInfo.init(dims.size());
    for (unsigned int idx=0; idx<dims.size(); idx++)
    {
        Dimension const& dim = dims[idx];

        std::string name = dim.getName();
        if (boost::iequals(dim.getName(),"EdgeOfFlightLine")) name = CHANNEL_NAME_EdgeFlightLine;
        if (boost::iequals(dim.getName(), "Classification")) name = CHANNEL_NAME_ClassId;
        if (boost::iequals(dim.getName(), "ScanAngleRank")) name = CHANNEL_NAME_ScanAngle;
        if (boost::iequals(dim.getName(), "ScanDirectionFlag")) name = CHANNEL_NAME_ScanDir;
        if (boost::iequals(dim.getName(), "Time")) name = CHANNEL_NAME_GPSTime;
        if (boost::iequals(dim.getName(), "PointSourceId")) name = CHANNEL_NAME_SourceId;
        if (boost::iequals(dim.getName(), "ReturnNumber")) name = CHANNEL_NAME_ReturnNum;
        if (boost::iequals(dim.getName(), "NumberOfReturns")) name = CHANNEL_NAME_NumReturns;

        if (dim.getInterpretation() == dimension::Float)
        {
            if (dim.getByteSize() == 8)
                pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_FLOAT64, 64);
            if (dim.getByteSize() == 4)
                pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_FLOAT32, 32);
        }
        if (dim.getInterpretation() == dimension::SignedInteger)
        {
            if (dim.getByteSize() == 8)
                pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_SINT64, 64);
            if (dim.getByteSize() == 4)
                pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_SINT32, 32);
            if (dim.getByteSize() == 2)
                pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_SINT16, 16);
            if (dim.getByteSize() == 1)
                pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_SINT8, 8);
        }
        if (dim.getInterpretation() == dimension::UnsignedInteger)
        {
            if (dim.getByteSize() == 8)
                pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_UINT64, 64);
            if (dim.getByteSize() == 4)
                pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_UINT32, 32);
            if (dim.getByteSize() == 2)
                pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_UINT16, 16);
            if (dim.getByteSize() == 1)
                pointInfo.getChannel(idx).init(name.c_str(), LizardTech::DATATYPE_UINT8, 8);
        }

    }
    return 0; //bug, do error checking
}


boost::uint32_t Reader::processBuffer(PointBuffer& data, boost::uint64_t index) const
{
    const Schema& schema = data.getSchema();

    // how many are they asking for?
    boost::uint64_t numPointsWanted = data.getCapacity();

    // we can only give them as many as we have left
    boost::uint64_t numPointsAvailable = getNumPoints() - index;
    if (numPointsAvailable < numPointsWanted)
        numPointsWanted = numPointsAvailable;


    LizardTech::PointData points;
    // to do:  specify a PointInfo structure that reads only the channels we will output.
    points.init(m_PS->getPointInfo(), (size_t)numPointsWanted);
    size_t count = m_iter->getNextPoints(points);

    boost::uint32_t cnt = 0;
    data.setNumPoints(0);

    schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();

    for (boost::uint32_t pointIndex=0; pointIndex<count; pointIndex++)
    {
        ++cnt;

        for (unsigned int i=0; i < dims.size(); i++)
        {
            Dimension const& d = dims[i];

            if (d.getName() == "X" && d.getInterpretation() == dimension::Float && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_X))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_X)->getData());
                double value = static_cast<double>(pData[pointIndex]);
                data.setField<double>(d, pointIndex, value);
            }
            else if (d.getName() == "X" && d.getInterpretation() == dimension::SignedInteger && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_X))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_X)->getData());
                boost::int32_t value = static_cast<boost::int32_t>(pData[pointIndex]);
                data.setField<boost::int32_t>(d, pointIndex, value);
            }
            else if (d.getName() == "Y" && d.getInterpretation() == dimension::Float && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_Y))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_Y)->getData());
                double value = static_cast<double>(pData[pointIndex]);
                data.setField<double>(d, pointIndex, value);
            }
            else if (d.getName() == "Y" && d.getInterpretation() == dimension::SignedInteger && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_Y))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_Y)->getData());
                boost::int32_t value = static_cast<boost::int32_t>(pData[pointIndex]);
                data.setField<boost::int32_t>(d, pointIndex, value);
            }
            else if (d.getName() == "Z" && d.getInterpretation() == dimension::Float && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_Z))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_Z)->getData());
                double value = static_cast<double>(pData[pointIndex]);
                data.setField<double>(d, pointIndex, value);
            }
            else if (d.getName() == "Z" && d.getInterpretation() == dimension::SignedInteger && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_Z))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_Z)->getData());
                boost::int32_t value = static_cast<boost::int32_t>(pData[pointIndex]);
                data.setField<boost::int32_t>(d, pointIndex, value);
            }
            else if (d.getName() == "Time" && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_GPSTime))
            {
                double *pData = static_cast<double*>(points.getChannel(CHANNEL_NAME_GPSTime)->getData());
                boost::uint64_t  value = static_cast<boost::uint64_t>(pData[pointIndex]);
                data.setField<boost::uint64_t>(d, pointIndex, value);
            }
            else if (d.getName() == "Intensity" && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_Intensity))
            {
                uint16_t *pData = static_cast<uint16_t*>(points.getChannel(CHANNEL_NAME_Intensity)->getData());
                boost::uint16_t value = static_cast<boost::uint16_t>(pData[pointIndex]);
                data.setField<boost::uint16_t>(d, pointIndex, value);
            }
            else if (d.getName() == "ReturnNumber" && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_ReturnNum))
            {
                uint8_t *pData = static_cast<uint8_t*>(points.getChannel(CHANNEL_NAME_ReturnNum)->getData());
                boost::uint8_t value = static_cast<boost::uint8_t>(pData[pointIndex]);
                data.setField<boost::uint8_t>(d, pointIndex, value);
            }
            else if (d.getName() == "NumberOfReturns" && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_NumReturns))
            {
                uint8_t *pData = static_cast<uint8_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::uint8_t value = static_cast<boost::uint8_t>(pData[pointIndex]);
                data.setField<boost::uint8_t>(d, pointIndex, value);
            }
            else if (d.getName() == "ScanDirectionFlag" && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_ScanDir))
            {
                uint8_t *pData = static_cast<uint8_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::uint8_t value = static_cast<boost::uint8_t>(pData[pointIndex]);
                data.setField<boost::uint8_t>(d, pointIndex, value);
            }
            else if (d.getName() == "ScanAngleRank" && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_ScanAngle))
            {
                boost::int8_t *pData = static_cast<int8_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::int8_t value = static_cast<boost::int8_t>(pData[pointIndex]);
                data.setField<boost::int8_t>(d, pointIndex, value);
            }
            else if (d.getName() == "EdgeOfFlightLine" && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_EdgeFlightLine))
            {
                uint8_t *pData = static_cast<uint8_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::uint8_t value = static_cast<boost::uint8_t>(pData[pointIndex]);
                data.setField<boost::uint8_t>(d, pointIndex, value);
            }
            else if (d.getName() == "Classification" && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_ClassId))
            {
                uint8_t *pData = static_cast<uint8_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::uint8_t value = static_cast<boost::uint8_t>(pData[pointIndex]);
                data.setField<boost::uint8_t>(d, pointIndex, value);
            }
            else if (d.getName() == "UserData" && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_UserData))
            {
                uint8_t *pData = static_cast<uint8_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::uint8_t value = static_cast<boost::uint8_t>(pData[pointIndex]);
                data.setField<boost::uint8_t>(d, pointIndex, value);
            }
            else if (d.getName() == "PointSourceId" && m_PS->getPointInfo().hasChannel(CHANNEL_NAME_SourceId))
            {
                uint16_t *pData = static_cast<uint16_t*>(points.getChannel(CHANNEL_NAME_NumReturns)->getData());
                boost::uint16_t value = static_cast<boost::uint16_t>(pData[pointIndex]);
                data.setField<boost::uint16_t>(d, pointIndex, value);
            }


        }
    }
    data.setNumPoints(cnt);
    assert(cnt <= data.getCapacity());

    return cnt;
}

std::vector<Dimension> Reader::getDefaultDimensions()
{
    std::vector<Dimension> output;

    Dimension x("X", dimension::Float, 8);
    x.setUUID("8c54ff0c-234f-43a2-8959-d9681ad1dea3");
    x.setNamespace(s_getName());
    output.push_back(x);

    Dimension y("Y", dimension::Float, 8);
    y.setUUID("9cee971b-2505-40cd-b7e9-f32c399afac7");
    y.setNamespace(s_getName());
    output.push_back(y);

    Dimension z("Z", dimension::Float, 8);
    z.setUUID("89dc4e36-6166-4bc8-bf95-5660657b0ea6");
    z.setNamespace(s_getName());
    output.push_back(z);

    Dimension t("Time", dimension::Float, 8);
    t.setUUID("3aea2826-4a6e-4c1b-ae27-7b2f24763ce1");
    t.setNamespace(s_getName());
    output.push_back(t);

    Dimension blue("Blue", dimension::UnsignedInteger, 2);
    blue.setUUID("f69977e3-23e8-4483-91b6-14cad981e9fd");
    blue.setNamespace(s_getName());
    output.push_back(blue);

    Dimension red("Red", dimension::UnsignedInteger, 2);
    red.setUUID("40a02a80-c399-42f0-a587-a286635b446e");
    red.setNamespace(s_getName());
    output.push_back(red);

    Dimension green("Green", dimension::UnsignedInteger, 2);
    green.setUUID("b329df2d-44f1-4f35-8993-d183dacaa1ff");
    green.setNamespace(s_getName());
    output.push_back(green);

    Dimension cls("Classification", dimension::UnsignedInteger, 1);
    cls.setUUID("ba1e2af8-6dfd-46a7-972a-ecc00f68914d");
    cls.setNamespace(s_getName());
    output.push_back(cls);

    Dimension edge("EdgeOfFlightLine", dimension::UnsignedInteger, 1);
    edge.setUUID("daed3bfc-650d-4265-93a7-d8093cbd75a0");
    edge.setNamespace(s_getName());
    output.push_back(edge);

    Dimension intensity("Intensity", dimension::UnsignedInteger, 2);
    intensity.setUUID("a43332a8-26b3-4609-a2b1-ddfda30e0565");
    intensity.setNamespace(s_getName());
    output.push_back(intensity);

    Dimension num_returns("NumberOfReturns", dimension::UnsignedInteger, 1);
    num_returns.setUUID("fa7c5d56-fb2b-4f81-9126-f183000c3cd8");
    num_returns.setNamespace(s_getName());
    output.push_back(num_returns);

    Dimension return_no("ReturnNumber", dimension::UnsignedInteger, 1);
    return_no.setUUID("e38cc121-8d26-482a-8920-c5599b2cdd19");
    return_no.setNamespace(s_getName());
    output.push_back(return_no);

    Dimension scan_angle("ScanAngleRank", dimension::UnsignedInteger, 1);
    scan_angle.setUUID("5d816875-10a5-4048-ad9d-fd3b8d065a6a");
    scan_angle.setNamespace(s_getName());
    output.push_back(scan_angle);

    Dimension scan_dir("ScanDirectionFlag", dimension::UnsignedInteger, 1);
    scan_dir.setUUID("d1054379-8bf6-4685-abfc-1f0ec37aa819");
    scan_dir.setNamespace(s_getName());
    output.push_back(scan_dir);

    Dimension ptsource("PointSourceId", dimension::UnsignedInteger, 2);
    ptsource.setUUID("be6e71af-b2f7-4107-a902-96e2fb71343f");
    ptsource.setNamespace(s_getName());
    output.push_back(ptsource);

    Dimension userdata("UserData", dimension::UnsignedInteger, 1);
    userdata.setUUID("551ca4be-cb6e-47a4-93a9-e403e9a06a8a");
    userdata.setNamespace(s_getName());
    output.push_back(userdata);

    return output;
}


namespace iterators
{

namespace sequential
{


Reader::Reader(const pdal::drivers::mrsid::Reader& reader, PointBuffer& buffer, boost::uint32_t numPoints)
    : pdal::ReaderSequentialIterator(buffer)
    , m_numPoints(numPoints)
    , m_reader(reader)
{
    return;
}


boost::uint64_t Reader::skipImpl(boost::uint64_t count)
{
    return count;
}


bool Reader::atEndImpl() const
{
    // const boost::uint64_t numPoints = getStage().getNumPoints();
    const boost::uint64_t currPoint = getIndex();
    return currPoint >= m_numPoints;
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    return m_reader.processBuffer(data, getIndex());
}

} // sequential


} // iterators

}
}
} // namespaces
