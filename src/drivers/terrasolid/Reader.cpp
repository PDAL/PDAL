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
#include <pdal/drivers/terrasolid/Iterator.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Utils.hpp>

#include <pdal/exceptions.hpp>

#include <iostream>
#include <map>

namespace pdal { namespace drivers { namespace terrasolid {

PointIndexes::PointIndexes(const Schema& schema, TERRASOLID_Format_Type format)
{

    if (format == TERRASOLID_Format_1) 
    {
        Classification = schema.getDimensionIndex(Dimension::Field_Classification, Dimension::Uint8);
        PointSourceId = schema.getDimensionIndex(Dimension::Field_PointSourceId, Dimension::Uint8);
        EchoInt = schema.getDimensionIndex(Dimension::Field_ReturnNumber, Dimension::Uint16);
        X = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
        Y = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
        Z = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
        
    } else if (format == TERRASOLID_Format_2)
    {
        X = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
        Y = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
        Z = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
        Classification = schema.getDimensionIndex(Dimension::Field_Classification, Dimension::Uint8);
        ReturnNumber = schema.getDimensionIndex(Dimension::Field_ReturnNumber, Dimension::Uint8);
        Flag = schema.getDimensionIndex(Dimension::Field_User1, Dimension::Uint8);
        Mark = schema.getDimensionIndex(Dimension::Field_User2, Dimension::Uint8);
        PointSourceId = schema.getDimensionIndex(Dimension::Field_PointSourceId, Dimension::Uint16);
        Intensity = schema.getDimensionIndex(Dimension::Field_Intensity, Dimension::Uint16);

    } 

    Time = schema.getDimensionIndex(Dimension::Field_Time, Dimension::Uint32);

    Red = schema.getDimensionIndex(Dimension::Field_Red, Dimension::Uint8);
    Green = schema.getDimensionIndex(Dimension::Field_Green, Dimension::Uint8);
    Blue = schema.getDimensionIndex(Dimension::Field_Blue, Dimension::Uint8);
    Alpha = schema.getDimensionIndex(Dimension::Field_Alpha, Dimension::Uint8);
    


    return;
}

Reader::Reader(const Options& options)
    : pdal::Reader(options)
    , m_optionsOld(*(new OptionsOld()))
    , m_format(TERRASOLID_Format_Unknown)
    , m_haveColor(false)
    , m_haveTime(false)
{
     throw not_yet_implemented("terrasolid reader options support"); 
}



Reader::Reader(OptionsOld& optionsOld)
    : pdal::Reader(Options::empty())
    , m_optionsOld(optionsOld)
    , m_format(TERRASOLID_Format_Unknown)
    , m_haveColor(false)
    , m_haveTime(false)
{
    std::string filename= getFileName();
    
    std::istream* stream = Utils::openFile(filename);
    
    stream->seekg(0);
    
    TerraSolidHeaderPtr h(new TerraSolidHeader);
    m_header.swap(h);
    Utils::read_n(*m_header, *stream, sizeof(TerraSolidHeader));

    // std::cout << "RecogVal: " << m_header->RecogVal << std::endl;
    if ( m_header->RecogVal != 970401)
        throw terrasolid_error("Header identifier was not '970401', is this a TerraSolid .bin file?");


    setNumPoints(m_header->PntCnt);

    m_haveColor = (m_header->Color==1);
    m_haveTime = (m_header->Time==1);
    m_format = static_cast<TERRASOLID_Format_Type>(m_header->HdrVersion);
    
    
    if ( !( (m_format==TERRASOLID_Format_1) || (m_format == TERRASOLID_Format_2) ) )
    {
        std::ostringstream oss;
        oss << "Version was '" << m_format << "', not '" << TERRASOLID_Format_1 << "' or '" << TERRASOLID_Format_2 << "'";
        throw terrasolid_error(oss.str());
        
    }

    registerFields();
    
    m_offset = 56;
    SchemaLayout layout(getSchemaRef());
    m_size = layout.getByteSize();
    
    // std::cout << "format: " << m_format << std::endl;
    // std::cout << "OrgX: " << m_header->OrgX << std::endl;
    // std::cout << "OrgY: " << m_header->OrgY << std::endl;
    // std::cout << "OrgZ: " << m_header->OrgZ << std::endl;
    // std::cout << "Units: " << m_header->Units << std::endl;
    // std::cout << "Time: " << m_header->Time << std::endl;
    // std::cout << "Color: " << m_header->Color << std::endl;
    // std::cout << "Count: " << m_header->PntCnt << std::endl;
   
    // getSchemaRef().dump();
    delete stream;
}    


std::string Reader::getFileName() const
{
    try
    {
        return m_optionsOld.GetPTree().get<std::string>("input");
        
    } catch (boost::property_tree::ptree_bad_path const&)
    {
        return std::string("");
    }
}
void Reader::registerFields()
{
    Schema& schema = getSchemaRef();
    
    double xyz_scale = 1/static_cast<double>(m_header->Units);
    
    std::ostringstream text;
    
    if (m_format == TERRASOLID_Format_1)
    {
        Dimension classification(Dimension::Field_Classification, Dimension::Uint8);
        text << "Classification code 0-255";
        classification.setDescription(text.str());
        schema.addDimension(classification);
        text.str("");

        Dimension flight_line(Dimension::Field_PointSourceId, Dimension::Uint8);
        text << "Flightline number 0-255";
        flight_line.setDescription(text.str());
        schema.addDimension(flight_line);
        text.str("");

        Dimension echoint(Dimension::Field_User1, Dimension::Uint16);
        text << "Intensity bits 0-13, echo bits 14-15";
        echoint.setDescription(text.str());
        schema.addDimension(echoint);
        text.str("");

        Dimension x(Dimension::Field_X, Dimension::Int32);
        text << "Easting";
        x.setDescription(text.str());
        x.setNumericScale(xyz_scale);
        x.setNumericOffset(m_header->OrgX);
        schema.addDimension(x);
        text.str("");

        Dimension y(Dimension::Field_Y, Dimension::Int32);
        text << "Northing";
        y.setDescription(text.str());
        y.setNumericScale(xyz_scale);
        y.setNumericOffset(m_header->OrgY);
        schema.addDimension(y);
        text.str("");

        Dimension z(Dimension::Field_Z, Dimension::Int32);
        text << "Elevation";
        z.setDescription(text.str());
        z.setNumericScale(xyz_scale);
        z.setNumericOffset(m_header->OrgZ);
        schema.addDimension(z);
        text.str("");

    }

    if (m_format == TERRASOLID_Format_2)
    {
        Dimension x(Dimension::Field_X, Dimension::Int32);
        text << "Easting";
        x.setDescription(text.str());
        x.setNumericScale(xyz_scale);
        x.setNumericOffset(m_header->OrgX);
        schema.addDimension(x);
        text.str("");

        Dimension y(Dimension::Field_Y, Dimension::Int32);
        text << "Northing";
        y.setDescription(text.str());
        y.setNumericScale(xyz_scale);
        y.setNumericOffset(m_header->OrgY);
        schema.addDimension(y);
        text.str("");

        Dimension z(Dimension::Field_Z, Dimension::Int32);
        text << "Elevation";
        z.setDescription(text.str());
        z.setNumericScale(xyz_scale);
        z.setNumericOffset(m_header->OrgZ);
        schema.addDimension(z);
        text.str("");

        Dimension classification(Dimension::Field_Classification, Dimension::Uint8);
        text << "Classification code 0-255";
        classification.setDescription(text.str());
        schema.addDimension(classification);
        text.str("");

        Dimension return_no(Dimension::Field_ReturnNumber, Dimension::Uint8); 
        text << "Echo/Return Number.  0 - Only echo. 1 - First of many echo. 2 - Intermediate echo. 3 - Last of many echo.";
        return_no.setDescription(text.str());
        schema.addDimension(return_no);
        text.str("");        

        Dimension flag(Dimension::Field_User1, Dimension::Uint8);
        text << "Runtime flag (view visibility)";
        flag.setDescription(text.str());
        schema.addDimension(flag);
        text.str("");

        Dimension mark(Dimension::Field_User2, Dimension::Uint8);
        text << "Runtime flag";
        mark.setDescription(text.str());
        schema.addDimension(mark);
        text.str("");

        Dimension flight_line(Dimension::Field_PointSourceId, Dimension::Uint16);
        text << "Flightline number 0-255";
        flight_line.setDescription(text.str());
        schema.addDimension(flight_line);
        text.str("");

        Dimension intensity(Dimension::Field_Intensity, Dimension::Uint16);
        text << "The intensity value is the integer representation of the pulse "
             "return magnitude. This value is optional and system specific. "
             "However, it should always be included if available.";
        intensity.setDescription(text.str());
        schema.addDimension(intensity);
        text.str("");
        
    }
    
    
    if (m_haveTime)
    {
        Dimension time(Dimension::Field_Time, Dimension::Uint32);
        text << "32 bit integer time stamps. Time stamps are assumed to be GPS week seconds. The storage format is a 32 bit unsigned integer where each integer step is 0.0002 seconds.";
        time.setDescription(text.str());
        time.setNumericScale(0.0002);
        time.setNumericOffset(0.0);
        schema.addDimension(time);
        text.str("");
    }

    if (m_haveColor)
    {
        Dimension red(Dimension::Field_Red, Dimension::Uint8);
        text << "The red image channel value associated with this point";
        red.setDescription(text.str());
        schema.addDimension(red);
        text.str("");

        Dimension green(Dimension::Field_Green, Dimension::Uint8);
        text << "The green image channel value associated with this point";
        green.setDescription(text.str());
        schema.addDimension(green);
        text.str("");

        Dimension blue(Dimension::Field_Blue, Dimension::Uint8);
        text << "The blue image channel value associated with this point";
        blue.setDescription(text.str());
        schema.addDimension(blue);
        text.str("");    

        Dimension alpha(Dimension::Field_Alpha, Dimension::Uint8);
        text << "The alpha image channel value associated with this point";
        alpha.setDescription(text.str());
        schema.addDimension(alpha);
        text.str("");

    }

    return;
}

const std::string& Reader::getDescription() const
{
    static std::string name("TerraSolid Reader");
    return name;
}

const std::string& Reader::getName() const
{
    static std::string name("drivers.terrasolid.reader");
    return name;
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

    const SchemaLayout& schemaLayout = data.getSchemaLayout();
    const Schema& schema = schemaLayout.getSchema();
    
    const int pointByteCount = getPointDataSize();
    const PointIndexes indexes(schema, m_format);
    
    boost::uint8_t* buf = new boost::uint8_t[pointByteCount * numPoints];
    Utils::read_n(buf, stream, pointByteCount * numPoints);

    for (boost::uint32_t pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        boost::uint8_t* p = buf + pointByteCount * pointIndex;

        if (m_format == TERRASOLID_Format_1) 
        {
            boost::uint8_t classification = Utils::read_field<boost::uint8_t>(p);
            data.setField<boost::uint8_t>(pointIndex, indexes.Classification, classification);

            boost::uint8_t flight_line = Utils::read_field<boost::uint8_t>(p);
            data.setField<boost::uint8_t>(pointIndex, indexes.PointSourceId, flight_line);            

            boost::uint16_t echo_int = Utils::read_field<boost::uint16_t>(p);
            data.setField<boost::uint16_t>(pointIndex, indexes.EchoInt, echo_int);

            boost::int32_t x = Utils::read_field<boost::int32_t>(p);
            data.setField<boost::int32_t>(pointIndex, indexes.X, x);
            
            boost::int32_t y = Utils::read_field<boost::int32_t>(p);
            data.setField<boost::int32_t>(pointIndex, indexes.Y, y);
            
            boost::int32_t z = Utils::read_field<boost::int32_t>(p);
            data.setField<boost::int32_t>(pointIndex, indexes.Z, z);
            
            if (indexes.Time != -1)
            {
                boost::uint32_t time = Utils::read_field<boost::uint32_t>(p);
                data.setField<boost::uint32_t>(pointIndex, indexes.Time, time);
            }
            
            if (indexes.Red != -1)
            {
                boost::uint8_t red = Utils::read_field<boost::uint8_t>(p);
                data.setField<boost::uint8_t>(pointIndex, indexes.Red, red);

                boost::uint8_t green = Utils::read_field<boost::uint8_t>(p);
                data.setField<boost::uint8_t>(pointIndex, indexes.Green, green);

                boost::uint8_t blue = Utils::read_field<boost::uint8_t>(p);
                data.setField<boost::uint8_t>(pointIndex, indexes.Blue, blue);

                boost::uint8_t alpha = Utils::read_field<boost::uint8_t>(p);
                data.setField<boost::uint8_t>(pointIndex, indexes.Alpha, alpha);
            }
        }

        if (m_format == TERRASOLID_Format_2) 
        {
            // std::cout << "Reading TERRASOLID_Format_2" << std::endl;
            boost::int32_t x = Utils::read_field<boost::int32_t>(p);
            data.setField<boost::int32_t>(pointIndex, indexes.X, x);
            
            boost::int32_t y = Utils::read_field<boost::int32_t>(p);
            data.setField<boost::int32_t>(pointIndex, indexes.Y, y);
            
            boost::int32_t z = Utils::read_field<boost::int32_t>(p);
            data.setField<boost::int32_t>(pointIndex, indexes.Z, z);
            
            // std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
            
            boost::uint8_t classification = Utils::read_field<boost::uint8_t>(p);
            data.setField<boost::uint8_t>(pointIndex, indexes.Classification, classification);
            
            boost::uint8_t return_number = Utils::read_field<boost::uint8_t>(p);
            data.setField<boost::uint8_t>(pointIndex, indexes.ReturnNumber, return_number);

            boost::uint8_t flag = Utils::read_field<boost::uint8_t>(p);
            data.setField<boost::uint8_t>(pointIndex, indexes.Flag, flag);

            boost::uint8_t mark = Utils::read_field<boost::uint8_t>(p);
            data.setField<boost::uint8_t>(pointIndex, indexes.Mark, mark);

            boost::uint16_t flight_line = Utils::read_field<boost::uint16_t>(p);
            data.setField<boost::uint16_t>(pointIndex, indexes.PointSourceId, flight_line);            

            boost::uint16_t intensity = Utils::read_field<boost::uint16_t>(p);
            data.setField<boost::uint16_t>(pointIndex, indexes.Intensity, intensity);


            
            if (indexes.Time != -1)
            {
                boost::uint32_t time = Utils::read_field<boost::uint32_t>(p);
                data.setField<boost::uint32_t>(pointIndex, indexes.Time, time);
                // std::cout << "We have time " << time << std::endl;
            }
            
            if (indexes.Red != -1)
            {
                boost::uint8_t red = Utils::read_field<boost::uint8_t>(p);
                data.setField<boost::uint8_t>(pointIndex, indexes.Red, red);

                boost::uint8_t green = Utils::read_field<boost::uint8_t>(p);
                data.setField<boost::uint8_t>(pointIndex, indexes.Green, green);

                boost::uint8_t blue = Utils::read_field<boost::uint8_t>(p);
                data.setField<boost::uint8_t>(pointIndex, indexes.Blue, blue);

                boost::uint8_t alpha = Utils::read_field<boost::uint8_t>(p);
                data.setField<boost::uint8_t>(pointIndex, indexes.Alpha, alpha);
                // std::cout << "Red: " << (int)red << " Green: " << (int)green << " Blue: " << (int)blue << std::endl;

            }
        }

        data.setNumPoints(pointIndex+1);
    }

    delete[] buf;

    // data.setSpatialBounds( lasHeader.getBounds() );

    return numPoints;
}

pdal::StageSequentialIterator* Reader::createSequentialIterator() const
{
    return new pdal::drivers::terrasolid::SequentialIterator(*this);
}


pdal::StageRandomIterator* Reader::createRandomIterator() const
{
    return new pdal::drivers::terrasolid::RandomIterator(*this);
}




}}} // namespace pdal::driver::oci
