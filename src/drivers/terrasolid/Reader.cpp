/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#include <pdal/drivers/terrasolid/Reader.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/FileUtils.hpp>

#include <map>

namespace pdal
{
namespace drivers
{
namespace terrasolid
{

PointDimensions::PointDimensions(const Schema& schema, std::string const& ns)
{
    X = schema.getDimension("X", ns);
    Y = schema.getDimension("Y", ns);
    Z = schema.getDimension("Z", ns);
    Classification = schema.getDimension("Classification", ns);
    PointSourceId = schema.getDimension("PointSourceId", ns);
    ReturnNumber = schema.getDimension("ReturnNumber", ns);
    Intensity = schema.getDimension("Intensity", ns);
    Mark = schema.getDimension("Mark", ns);
    Flag = schema.getDimension("Flag", ns);
    Time = schema.getDimension("Time", ns);
    Red = schema.getDimension("Red", ns);
    Green = schema.getDimension("Green", ns);
    Blue = schema.getDimension("Blue", ns);
    Alpha = schema.getDimension("Alpha", ns);
}


Reader::Reader(const Options& options)
    : pdal::Reader(options)
    , m_format(TERRASOLID_Format_Unknown)
    , m_haveColor(false)
    , m_haveTime(false)
{
    std::string filename= getFileName();

    std::istream* stream = FileUtils::openFile(filename);

    stream->seekg(0);

    TerraSolidHeaderPtr h(new TerraSolidHeader);
    m_header.swap(h);
    Utils::read_n(*m_header, *stream, sizeof(TerraSolidHeader));

    if (m_header->RecogVal != 970401)
        throw terrasolid_error("Header identifier was not '970401', is this "
            "a TerraSolid .bin file?");

    m_haveColor = (m_header->Color != 0);
    m_haveTime = (m_header->Time != 0);
    m_format = static_cast<TERRASOLID_Format_Type>(m_header->HdrVersion);

    if (!((m_format==TERRASOLID_Format_1) || (m_format == TERRASOLID_Format_2)))
    {
        std::ostringstream oss;
        oss << "Version was '" << m_format << "', not '" <<
            TERRASOLID_Format_1 << "' or '" << TERRASOLID_Format_2 << "'";
        throw terrasolid_error(oss.str());

    }

    m_offset = 56;
//ABELL - This should be done in ready().
    m_size = 0;
//    m_size = schema.getByteSize();

    delete stream;
}


/**
void Reader::initialize()
{
    log()->get(logDEBUG) << "TerraSolid Reader::initialize format: " << m_format << std::endl;
    log()->get(logDEBUG) << "OrgX: " << m_header->OrgX << std::endl;
    log()->get(logDEBUG) << "OrgY: " << m_header->OrgY << std::endl;
    log()->get(logDEBUG) << "OrgZ: " << m_header->OrgZ << std::endl;
    log()->get(logDEBUG) << "Units: " << m_header->Units << std::endl;
    log()->get(logDEBUG) << "Time: " << m_header->Time << std::endl;
    log()->get(logDEBUG) << "Color: " << m_header->Color << std::endl;
    log()->get(logDEBUG) << "Count: " << m_header->PntCnt << std::endl;
    log()->get(logDEBUG) << "RecogVal: " << m_header->RecogVal << std::endl;
}
**/


Options Reader::getDefaultOptions()
{
    Options options;
    Option filename("filename", "", "file to read from");
    return options;
}


std::string Reader::getFileName() const
{
    return getOptions().getValueOrThrow<std::string>("filename");
}

void Reader::buildSchema(Schema *s)
{
    std::vector<Dimension> dims = getDefaultDimensions();
    std::map<std::string, Dimension> dimMap;
    for (auto di = dims.begin(); di != dims.end(); ++di)
        dimMap.insert(std::make_pair(di->getName(), *di));

    double xyz_scale = 1/static_cast<double>(m_header->Units);
    Dimension& x = dimMap["X"];
    x.setNumericScale(xyz_scale);
    x.setNumericOffset(-m_header->OrgX);

    Dimension& y = dimMap["Y"];
    y.setNumericScale(xyz_scale);
    y.setNumericOffset(-m_header->OrgY);

    Dimension& z = dimMap["Z"];
    z.setNumericScale(xyz_scale);
    z.setNumericOffset(-m_header->OrgZ);

    if (m_format == TERRASOLID_Format_1)
    {
        s->appendDimension(dimMap["Classification"]);

        // Fetch PointSource ID Uint8 dimension by UUID because dimensions
        // has two "PointSourceId" dimensions added.
        //ABELL - Huh?
        /**
        s->appendDimension(dimensions.getDimension(
            boost::uuids::string_generator()(
                "68c03b56-4248-4cca-ade5-33e90d5c5563")));
        **/
        s->appendDimension(dimMap["PointSourceId"]);

        s->appendDimension(dimMap["Intensity"]);
        s->appendDimension(x);
        s->appendDimension(y);
        s->appendDimension(z);
    }

    if (m_format == TERRASOLID_Format_2)
    {
        s->appendDimension(x);
        s->appendDimension(y);
        s->appendDimension(z);

        s->appendDimension(dimMap["Classification"]);

        //ABELL
        /**
        s->appendDimension(dimensions.getDimension(
            boost::uuids::string_generator()(
                "465a9a7e-1e04-47b0-97b6-4f826411bc71")));
        **/
        s->appendDimension(dimMap["ReturnNumber"]);

        s->appendDimension(dimMap["Flag"]);
        s->appendDimension(dimMap["Mark"]);

        //ABELL
        /**
        s->appendDimension(dimensions.getDimension(
            boost::uuids::string_generator()(
                "7193bb9f-3ca2-491f-ba18-594321493789")));
        **/
        s->appendDimension(dimMap["PointSourceId"]);

        s->appendDimension(dimMap["Intensity"]);
    }

    if (m_haveTime)
        s->appendDimension(dimMap["Time"]);

    if (m_haveColor)
    {
        s->appendDimension(dimMap["Red"]);
        s->appendDimension(dimMap["Green"]);
        s->appendDimension(dimMap["Blue"]);
        s->appendDimension(dimMap["Alpha"]);
    }
}


uint32_t Reader::processBuffer(PointBuffer& data, std::istream& stream,
    uint64_t numPointsLeft) const
{
    // we must not read more points than are left in the file
    uint32_t numPoints =
        (uint32_t)std::min((uint64_t)data.size(), numPointsLeft);

    const Schema& schema = data.getSchema();

    const int pointByteCount = getPointDataSize();

    const PointDimensions dimensions(schema, getName());

    uint8_t *buf = new uint8_t[pointByteCount * numPoints];
    Utils::read_n(buf, stream, pointByteCount * numPoints);

    for (uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        uint8_t* p = buf + pointByteCount * pointIndex;

        if (m_format == TERRASOLID_Format_1)
        {
            uint8_t classification = Utils::read_field<uint8_t>(p);
            if (dimensions.Classification)
                data.setField(dimensions.Classification, pointIndex,
                    classification);

            uint8_t flight_line = Utils::read_field<uint8_t>(p);
            if (dimensions.PointSourceId)
                data.setField(dimensions.PointSourceId, pointIndex,
                    flight_line);

            uint16_t echo_int = Utils::read_field<uint16_t>(p);
            if (dimensions.ReturnNumber)
                data.setField(dimensions.ReturnNumber, pointIndex, echo_int);

            int32_t x = Utils::read_field<int32_t>(p);

            if (dimensions.X)
                data.setField(dimensions.X, pointIndex, x);

            int32_t y = Utils::read_field<int32_t>(p);
            if (dimensions.Y)
                data.setField(dimensions.Y, pointIndex, y);

            int32_t z = Utils::read_field<int32_t>(p);
            if (dimensions.Z)
                data.setField(dimensions.Z, pointIndex, z);

            uint32_t time = Utils::read_field<uint32_t>(p);
            if (dimensions.Time)
                data.setField(dimensions.Time, pointIndex, time);

            uint8_t red = Utils::read_field<uint8_t>(p);
            if (dimensions.Red)
                data.setField(dimensions.Red, pointIndex, red);

            uint8_t green = Utils::read_field<uint8_t>(p);
            if (dimensions.Green)
                data.setField(dimensions.Green, pointIndex, green);

            uint8_t blue = Utils::read_field<uint8_t>(p);
            if (dimensions.Blue)
                data.setField<uint8_t>(dimensions.Blue, pointIndex,  blue);

            uint8_t alpha = Utils::read_field<uint8_t>(p);
            if (dimensions.Alpha)
                data.setField(dimensions.Alpha, pointIndex,  alpha);
        }

        if (m_format == TERRASOLID_Format_2)
        {
            int32_t x = Utils::read_field<int32_t>(p);
            if (dimensions.X)
                data.setField(dimensions.X, pointIndex, x);

            int32_t y = Utils::read_field<int32_t>(p);
            if (dimensions.Y)
                data.setField(dimensions.Y, pointIndex, y);

            int32_t z = Utils::read_field<int32_t>(p);
            if (dimensions.Z)
                data.setField(dimensions.Z, pointIndex, z);

            uint8_t classification = Utils::read_field<uint8_t>(p);
            if (dimensions.Classification)
                data.setField(dimensions.Classification, pointIndex,
                    classification);

            uint8_t return_number = Utils::read_field<uint8_t>(p);
            if (dimensions.ReturnNumber)
                data.setField(dimensions.ReturnNumber, pointIndex,
                    return_number);

            uint8_t flag = Utils::read_field<uint8_t>(p);
            if (dimensions.Flag)
                data.setField(dimensions.Flag, pointIndex, flag);

            boost::uint8_t mark = Utils::read_field<uint8_t>(p);
            if (dimensions.Mark)
                data.setField(dimensions.Mark, pointIndex, mark);

            uint16_t flight_line = Utils::read_field<uint16_t>(p);
            if (dimensions.PointSourceId)
                data.setField(dimensions.PointSourceId, pointIndex,
                    flight_line);

            uint16_t intensity = Utils::read_field<uint16_t>(p);
            if (dimensions.Intensity)
                data.setField(dimensions.Intensity, pointIndex, intensity);

            uint32_t time = Utils::read_field<uint32_t>(p);
            if (dimensions.Time)
                data.setField(dimensions.Time, pointIndex, time);

            uint8_t red = Utils::read_field<uint8_t>(p);
            if (dimensions.Red)
                data.setField(dimensions.Red, pointIndex, red);

            uint8_t green = Utils::read_field<uint8_t>(p);
            if (dimensions.Green)
                data.setField(dimensions.Green, pointIndex,  green);

            uint8_t blue = Utils::read_field<uint8_t>(p);
            if (dimensions.Blue)
                data.setField(dimensions.Blue, pointIndex, blue);

            uint8_t alpha = Utils::read_field<uint8_t>(p);
            if (dimensions.Alpha)
                data.setField(dimensions.Alpha, pointIndex, alpha);
        }
    }

    delete[] buf;

    return numPoints;
}

pdal::StageSequentialIterator*
Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::terrasolid::iterators::sequential::Reader(
        *this, buffer);
}


pdal::StageRandomIterator* Reader::createRandomIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::terrasolid::iterators::random::Reader(
        *this, buffer);
}

std::vector<Dimension> Reader::getDefaultDimensions()
{
    std::vector<Dimension> output;

    Dimension alpha("Alpha", dimension::UnsignedInteger, 1,
                    "The alpha image channel value associated with this point");
    alpha.setUUID("f3806ee6-e82e-45af-89bd-59b20cda8ffa");
    alpha.setNamespace(s_getName());
    output.push_back(alpha);

    Dimension classification("Classification", dimension::UnsignedInteger, 1,
                             "Classification code 0-255");
    classification.setUUID("845e23ca-fc4b-4dfc-aa71-a40cc2927421");
    classification.setNamespace(s_getName());
    output.push_back(classification);

    Dimension point_source("PointSourceId", dimension::UnsignedInteger, 1,
                           "Flightline number 0-255");
    point_source.setUUID("68c03b56-4248-4cca-ade5-33e90d5c5563");
    point_source.setNamespace(s_getName());
    output.push_back(point_source);

    Dimension point_source2("PointSourceId", dimension::UnsignedInteger, 2,
                            "Flightline number 0-65536");
    point_source2.setUUID("7193bb9f-3ca2-491f-ba18-594321493789");
    point_source2.setNamespace(s_getName());
    output.push_back(point_source2);

    Dimension return_number("ReturnNumber", dimension::UnsignedInteger, 1,
                            "Echo/Return Number.  0 - Only echo. 1 - First of many echo. 2 - Intermediate echo. 3 - Last of many echo.");
    return_number.setUUID("465a9a7e-1e04-47b0-97b6-4f826411bc71");
    return_number.setNamespace(s_getName());
    output.push_back(return_number);

    Dimension return_number2("ReturnNumber", dimension::UnsignedInteger, 2,
                             "Echo/Return Number.  0 - Only echo. 1 - First of many echo. 2 - Intermediate echo. 3 - Last of many echo.");
    return_number2.setUUID("43a1c59d-02ae-4a05-85af-526fae890eb9");
    return_number2.setNamespace(s_getName());
    output.push_back(return_number2);

    Dimension flag("Flag", dimension::UnsignedInteger, 1,
                   "Runtime flag (view visibility)");
    flag.setUUID("583a5904-ee67-47a7-9fba-2f46daf11441");
    flag.setNamespace(s_getName());
    output.push_back(flag);

    Dimension mark("Mark", dimension::UnsignedInteger, 1,
                   "Runtime flag");
    mark.setUUID("e889747c-2f19-4244-b282-b0b223868401");
    mark.setNamespace(s_getName());
    output.push_back(mark);

    Dimension intensity("Intensity", dimension::UnsignedInteger, 2,
                        "Runtime flag");
    intensity.setUUID("beaa015b-20dd-4922-bf1d-da6972596fe6");
    intensity.setNamespace(s_getName());
    output.push_back(intensity);

    Dimension x("X", dimension::SignedInteger, 4,
                "X dimension as a scaled integer");
    x.setUUID("64e530ee-7304-4d6a-9fe4-231b6c960e69");
    x.setNamespace(s_getName());
    output.push_back(x);

    Dimension y("Y", dimension::SignedInteger, 4,
                "Y dimension as a scaled integer");
    y.setUUID("9b4fce29-2846-45fa-be0c-f50228407f05");
    y.setNamespace(s_getName());
    output.push_back(y);

    Dimension z("Z", dimension::SignedInteger, 4,
                "Z dimension as a scaled integer");
    z.setUUID("464cd1f6-5bec-4610-9f25-79e839ee39a6");
    z.setNamespace(s_getName());
    output.push_back(z);

    Dimension red("Red", dimension::UnsignedInteger, 1,
                  "Red color value 0 - 256 ");
    red.setUUID("2157fd43-a492-40e4-a27c-7c37b48bd55c");
    red.setNamespace(s_getName());
    output.push_back(red);

    Dimension green("Green", dimension::UnsignedInteger, 1,
                    "Green color value 0 - 256 ");
    green.setUUID("c9cd71ef-1ce0-48c2-99f8-5b283e598eac");
    green.setNamespace(s_getName());
    output.push_back(green);

    Dimension blue("Blue", dimension::UnsignedInteger, 1,
                   "Blue color value 0 - 256 ");
    blue.setUUID("649f383f-8a7a-4658-ac2a-e1e36cfed05e");
    blue.setNamespace(s_getName());
    output.push_back(blue);

    Dimension time("Time", dimension::UnsignedInteger, 4,
                   "32 bit integer time stamps. Time stamps are assumed to be "
                   "GPS week seconds. The storage format is a 32 bit unsigned "
                   "integer where each integer step is 0.0002 seconds.");
    time.setNumericScale(0.0002);
    time.setNumericOffset(0.0);
    time.setUUID("0dcda772-56da-47f6-b04a-edad72361da9");
    time.setNamespace(s_getName());
    output.push_back(time);
    return output;
}


namespace iterators
{

namespace sequential
{


Reader::Reader(const terrasolid::Reader& reader, PointBuffer& buffer)
    : pdal::ReaderSequentialIterator(buffer), m_reader(reader)
{
    m_istream = FileUtils::openFile(m_reader.getFileName());
    m_istream->seekg(m_reader.getPointDataOffset());
}


Reader::~Reader()
{
    FileUtils::closeFile(m_istream);
    return;
}


boost::uint64_t Reader::skipImpl(boost::uint64_t count)
{
    m_istream->seekg(m_reader.getPointDataSize() * count, std::ios::cur);
    return count;
}


bool Reader::atEndImpl() const
{
    return getIndex() >= m_reader.getNumPoints();
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    uint32_t numToRead = m_reader.getNumPoints() - getIndex();
    return m_reader.processBuffer(data, *m_istream, numToRead);
}


} // sequential

namespace random
{



Reader::Reader(const terrasolid::Reader& reader, PointBuffer& buffer)
    : pdal::ReaderRandomIterator(buffer), m_reader(reader)
{
    m_istream = FileUtils::openFile(m_reader.getFileName());
    m_istream->seekg(m_reader.getPointDataOffset());
}


Reader::~Reader()
{
    FileUtils::closeFile(m_istream);
    return;
}


boost::uint64_t Reader::seekImpl(boost::uint64_t count)
{

    m_istream->seekg(m_reader.getPointDataSize() * count + m_reader.getPointDataOffset(), std::ios::cur);

    return count;
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    boost::uint32_t numToRead = m_reader.getNumPoints() - getIndex();
    return m_reader.processBuffer(data, *m_istream, numToRead);
}

} // random
} // iterators

} // namespace terrasolid
} // namespace drivers
} // namespace pdal

