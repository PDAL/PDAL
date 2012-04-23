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

    X = &schema.getDimension("X", ns);
    Y = &schema.getDimension("Y", ns);
    Z = &schema.getDimension("Z", ns);

    Classification = &schema.getDimension("Classification", ns);
    PointSourceId = &schema.getDimension("PointSourceId", ns);
    ReturnNumber = &schema.getDimension("ReturnNumber", ns);

    try
    {
        Intensity = &schema.getDimension("Intensity", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        Intensity = 0;
    }

    try
    {
        Mark = &schema.getDimension("Mark", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        Mark = 0;
    }

    try
    {
        Flag = &schema.getDimension("Flag", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        Flag = 0;
    }

    try
    {
        Time = &schema.getDimension("Time", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        Time = 0;
    }

    try
    {
        Red = &schema.getDimension("Red", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        Red = 0;
    }

    try
    {
        Green = &schema.getDimension("Green", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        Green = 0;
    }

    try
    {
        Blue = &schema.getDimension("Blue", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        Blue = 0;
    }

    try
    {
        Alpha = &schema.getDimension("Alpha", ns);
    }
    catch (pdal::dimension_not_found&)
    {
        Alpha = 0;
    }


    return;
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
        throw terrasolid_error("Header identifier was not '970401', is this a TerraSolid .bin file?");


    setNumPoints(m_header->PntCnt);

    m_haveColor = (m_header->Color != 0);
    m_haveTime = (m_header->Time != 0);
    m_format = static_cast<TERRASOLID_Format_Type>(m_header->HdrVersion);


    if (!((m_format==TERRASOLID_Format_1) || (m_format == TERRASOLID_Format_2)))
    {
        std::ostringstream oss;
        oss << "Version was '" << m_format << "', not '" << TERRASOLID_Format_1 << "' or '" << TERRASOLID_Format_2 << "'";
        throw terrasolid_error(oss.str());

    }

    addDefaultDimensions();

    registerFields();

    m_offset = 56;
    const Schema& schema = getSchema();
    m_size = schema.getByteSize();

    delete stream;
}


void Reader::initialize()
{
    pdal::Reader::initialize();

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


const Options Reader::getDefaultOptions() const
{
    Options options;
    Option filename("filename", "", "file to read from");
    return options;
}


std::string Reader::getFileName() const
{
    return getOptions().getValueOrThrow<std::string>("filename");
}

void Reader::registerFields()
{
    Schema& schema = getSchemaRef();

    Schema dimensions(getDefaultDimensions());

    double xyz_scale = 1/static_cast<double>(m_header->Units);
    Dimension x = dimensions.getDimension("X");
    x.setNumericScale(xyz_scale);
    x.setNumericOffset(m_header->OrgX);

    Dimension y = dimensions.getDimension("Y");
    y.setNumericScale(xyz_scale);
    y.setNumericOffset(m_header->OrgY);

    Dimension z = dimensions.getDimension("Z");
    z.setNumericScale(xyz_scale);
    z.setNumericOffset(m_header->OrgZ);

    if (m_format == TERRASOLID_Format_1)
    {
        schema.appendDimension(dimensions.getDimension("Classification"));

        // Fetch PointSource ID Uint8 dimension by UUID because dimensions
        // has two "PointSourceId" dimensions added.

        schema.appendDimension(dimensions.getDimension("68c03b56-4248-4cca-ade5-33e90d5c5563"));

        schema.appendDimension(dimensions.getDimension("Intensity"));

        schema.appendDimension(x);
        schema.appendDimension(y);
        schema.appendDimension(z);
    }

    if (m_format == TERRASOLID_Format_2)
    {
        schema.appendDimension(x);
        schema.appendDimension(y);
        schema.appendDimension(z);

        schema.appendDimension(dimensions.getDimension("Classification"));

        schema.appendDimension(dimensions.getDimension("465a9a7e-1e04-47b0-97b6-4f826411bc71"));

        schema.appendDimension(dimensions.getDimension("Flag"));
        schema.appendDimension(dimensions.getDimension("Mark"));

        schema.appendDimension(dimensions.getDimension("7193bb9f-3ca2-491f-ba18-594321493789"));

        schema.appendDimension(dimensions.getDimension("Intensity"));
    }

    if (m_haveTime)
    {
        schema.appendDimension(dimensions.getDimension("Time"));
    }

    if (m_haveColor)
    {
        schema.appendDimension(dimensions.getDimension("Red"));
        schema.appendDimension(dimensions.getDimension("Green"));
        schema.appendDimension(dimensions.getDimension("Blue"));
        schema.appendDimension(dimensions.getDimension("Alpha"));
    }

    return;
}


Reader::~Reader()
{

    return;
}

boost::uint32_t Reader::processBuffer(PointBuffer& data, std::istream& stream, boost::uint64_t numPointsLeft) const
{
    // we must not read more points than are left in the file
    const boost::uint64_t numPoints64 = std::min<boost::uint64_t>(data.getCapacity(), numPointsLeft);
    const boost::uint32_t numPoints = (boost::uint32_t)std::min<boost::uint64_t>(numPoints64, std::numeric_limits<boost::uint32_t>::max());

    const Schema& schema = data.getSchema();

    const int pointByteCount = getPointDataSize();

    const PointDimensions dimensions(schema, getName());

    boost::uint8_t* buf = new boost::uint8_t[pointByteCount * numPoints];
    Utils::read_n(buf, stream, pointByteCount * numPoints);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        boost::uint8_t* p = buf + pointByteCount * pointIndex;

        if (m_format == TERRASOLID_Format_1)
        {
            boost::uint8_t classification = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.Classification)
                data.setField<boost::uint8_t>(*dimensions.Classification, pointIndex, classification);

            boost::uint8_t flight_line = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.PointSourceId)
                data.setField<boost::uint8_t>(*dimensions.PointSourceId, pointIndex, flight_line);

            boost::uint16_t echo_int = Utils::read_field<boost::uint16_t>(p);
            if (dimensions.ReturnNumber)
                data.setField<boost::uint16_t>(*dimensions.ReturnNumber, pointIndex, echo_int);

            boost::int32_t x = Utils::read_field<boost::int32_t>(p);
            
            if (dimensions.X)
                data.setField<boost::int32_t>(*dimensions.X, pointIndex, x);

            boost::int32_t y = Utils::read_field<boost::int32_t>(p);
            if (dimensions.Y)
                data.setField<boost::int32_t>(*dimensions.Y, pointIndex, y);

            boost::int32_t z = Utils::read_field<boost::int32_t>(p);
            if (dimensions.Z)
                data.setField<boost::int32_t>(*dimensions.Z, pointIndex, z);
            
            boost::uint32_t time = Utils::read_field<boost::uint32_t>(p);
            if (dimensions.Time)
                data.setField<boost::uint32_t>(*dimensions.Time, pointIndex, time);

            boost::uint8_t red = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.Red)
                data.setField<boost::uint8_t>(*dimensions.Red, pointIndex, red);
            
            boost::uint8_t green = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.Green)
                data.setField<boost::uint8_t>(*dimensions.Green, pointIndex, green);
            
            boost::uint8_t blue = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.Blue)
                data.setField<boost::uint8_t>(*dimensions.Blue, pointIndex,  blue);

            boost::uint8_t alpha = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.Alpha)
                data.setField<boost::uint8_t>(*dimensions.Alpha, pointIndex,  alpha);
        }

        if (m_format == TERRASOLID_Format_2)
        {
            boost::int32_t x = Utils::read_field<boost::int32_t>(p);
            if (dimensions.X)
                data.setField<boost::int32_t>(*dimensions.X, pointIndex, x);

            boost::int32_t y = Utils::read_field<boost::int32_t>(p);
            if (dimensions.Y)
                data.setField<boost::int32_t>(*dimensions.Y, pointIndex, y);

            boost::int32_t z = Utils::read_field<boost::int32_t>(p);
            if (dimensions.Z)
                data.setField<boost::int32_t>(*dimensions.Z, pointIndex, z);

            boost::uint8_t classification = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.Classification)
                data.setField<boost::uint8_t>(*dimensions.Classification, pointIndex, classification);

            boost::uint8_t return_number = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.ReturnNumber)
                data.setField<boost::uint8_t>(*dimensions.ReturnNumber, pointIndex, return_number);

            boost::uint8_t flag = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.Flag)
                data.setField<boost::uint8_t>(*dimensions.Flag, pointIndex, flag);

            boost::uint8_t mark = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.Mark)
                data.setField<boost::uint8_t>(*dimensions.Mark, pointIndex, mark);

            boost::uint16_t flight_line = Utils::read_field<boost::uint16_t>(p);
            if (dimensions.PointSourceId)
                data.setField<boost::uint16_t>(*dimensions.PointSourceId, pointIndex, flight_line);

            boost::uint16_t intensity = Utils::read_field<boost::uint16_t>(p);
            if (dimensions.Intensity)
                data.setField<boost::uint16_t>(*dimensions.Intensity, pointIndex, intensity);

            boost::uint32_t time = Utils::read_field<boost::uint32_t>(p);
            if (dimensions.Time)
                data.setField<boost::uint32_t>(*dimensions.Time, pointIndex, time);

            boost::uint8_t red = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.Red)
                data.setField<boost::uint8_t>(*dimensions.Red, pointIndex, red);

            boost::uint8_t green = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.Green)
                data.setField<boost::uint8_t>(*dimensions.Green, pointIndex,  green);

            boost::uint8_t blue = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.Blue)
                data.setField<boost::uint8_t>(*dimensions.Blue, pointIndex, blue);

            boost::uint8_t alpha = Utils::read_field<boost::uint8_t>(p);
            if (dimensions.Alpha)
                data.setField<boost::uint8_t>(*dimensions.Alpha, pointIndex, alpha);

        }

        data.setNumPoints(pointIndex+1);
    }

    delete[] buf;

    return numPoints;
}

pdal::StageSequentialIterator* Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::terrasolid::iterators::sequential::Reader(*this, buffer);
}


pdal::StageRandomIterator* Reader::createRandomIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::terrasolid::iterators::random::Reader(*this, buffer);
}


boost::property_tree::ptree Reader::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Reader::toPTree();

    // add stuff here specific to this stage type

    return tree;
}

void Reader::addDefaultDimensions()
{
    Dimension alpha("Alpha", dimension::UnsignedInteger, 1,
                    "The alpha image channel value associated with this point");
    alpha.setUUID("f3806ee6-e82e-45af-89bd-59b20cda8ffa");
    addDefaultDimension(alpha, getName());

    Dimension classification("Classification", dimension::UnsignedInteger, 1,
                             "Classification code 0-255");
    classification.setUUID("845e23ca-fc4b-4dfc-aa71-a40cc2927421");
    addDefaultDimension(classification, getName());

    Dimension point_source("PointSourceId", dimension::UnsignedInteger, 1,
                           "Flightline number 0-255");
    point_source.setUUID("68c03b56-4248-4cca-ade5-33e90d5c5563");
    addDefaultDimension(point_source, getName());

    Dimension point_source2("PointSourceId", dimension::UnsignedInteger, 2,
                            "Flightline number 0-65536");
    point_source2.setUUID("7193bb9f-3ca2-491f-ba18-594321493789");
    addDefaultDimension(point_source2, getName());

    Dimension return_number("ReturnNumber", dimension::UnsignedInteger, 1,
                            "Echo/Return Number.  0 - Only echo. 1 - First of many echo. 2 - Intermediate echo. 3 - Last of many echo.");
    return_number.setUUID("465a9a7e-1e04-47b0-97b6-4f826411bc71");
    addDefaultDimension(return_number, getName());

    Dimension return_number2("ReturnNumber", dimension::UnsignedInteger, 2,
                             "Echo/Return Number.  0 - Only echo. 1 - First of many echo. 2 - Intermediate echo. 3 - Last of many echo.");
    return_number2.setUUID("43a1c59d-02ae-4a05-85af-526fae890eb9");
    addDefaultDimension(return_number2, getName());

    Dimension flag("Flag", dimension::UnsignedInteger, 1,
                   "Runtime flag (view visibility)");
    flag.setUUID("583a5904-ee67-47a7-9fba-2f46daf11441");
    addDefaultDimension(flag, getName());

    Dimension mark("Mark", dimension::UnsignedInteger, 1,
                   "Runtime flag");
    mark.setUUID("e889747c-2f19-4244-b282-b0b223868401");
    addDefaultDimension(mark, getName());

    Dimension intensity("Intensity", dimension::UnsignedInteger, 2,
                        "Runtime flag");
    intensity.setUUID("beaa015b-20dd-4922-bf1d-da6972596fe6");
    addDefaultDimension(intensity, getName());

    Dimension x("X", dimension::SignedInteger, 4,
                "X dimension as a scaled integer");
    x.setUUID("64e530ee-7304-4d6a-9fe4-231b6c960e69");
    addDefaultDimension(x, getName());

    Dimension y("Y", dimension::SignedInteger, 4,
                "Y dimension as a scaled integer");
    y.setUUID("9b4fce29-2846-45fa-be0c-f50228407f05");
    addDefaultDimension(y, getName());

    Dimension z("Z", dimension::SignedInteger, 4,
                "Z dimension as a scaled integer");
    z.setUUID("464cd1f6-5bec-4610-9f25-79e839ee39a6");
    addDefaultDimension(z, getName());

    Dimension red("Red", dimension::UnsignedInteger, 1,
                  "Red color value 0 - 256 ");
    red.setUUID("2157fd43-a492-40e4-a27c-7c37b48bd55c");
    addDefaultDimension(red, getName());

    Dimension green("Green", dimension::UnsignedInteger, 1,
                    "Green color value 0 - 256 ");
    green.setUUID("c9cd71ef-1ce0-48c2-99f8-5b283e598eac");
    addDefaultDimension(green, getName());

    Dimension blue("Blue", dimension::UnsignedInteger, 1,
                   "Blue color value 0 - 256 ");
    blue.setUUID("c9cd71ef-1ce0-48c2-99f8-5b283e598eac");
    addDefaultDimension(blue, getName());

    Dimension time("Time", dimension::UnsignedInteger, 4,
                   "32 bit integer time stamps. Time stamps are assumed to be "
                   "GPS week seconds. The storage format is a 32 bit unsigned "
                   "integer where each integer step is 0.0002 seconds.");
    time.setNumericScale(0.0002);
    time.setNumericOffset(0.0);
    time.setUUID("0dcda772-56da-47f6-b04a-edad72361da9");
    addDefaultDimension(time, getName());
}


namespace iterators
{

namespace sequential
{


Reader::Reader(const terrasolid::Reader& reader, PointBuffer& buffer)
    : pdal::ReaderSequentialIterator(reader, buffer)
    , m_reader(reader)
    , m_istream(NULL)
{
    m_istream = FileUtils::openFile(m_reader.getFileName());
    m_istream->seekg(m_reader.getPointDataOffset());
    return;
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
    return getIndex() >= getStage().getNumPoints();
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    return m_reader.processBuffer(data, *m_istream, getStage().getNumPoints()-this->getIndex());
}


} // sequential

namespace random
{



Reader::Reader(const terrasolid::Reader& reader, PointBuffer& buffer)
    : pdal::ReaderRandomIterator(reader, buffer)
    , m_reader(reader)
    , m_istream(NULL)
{
    m_istream = FileUtils::openFile(m_reader.getFileName());
    m_istream->seekg(m_reader.getPointDataOffset());
    return;
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
    return m_reader.processBuffer(data, *m_istream, getStage().getNumPoints()-this->getIndex());
}

} // random
} // iterators


}
}
} // namespace pdal::driver::oci
