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
#include <pdal/FileUtils.hpp>

#include <map>

namespace pdal { namespace drivers { namespace terrasolid {

PointIndexes::PointIndexes(const Schema& schema, TERRASOLID_Format_Type format)
{
    if (format == TERRASOLID_Format_1) 
    {
        Classification = schema.getDimensionIndex(DimensionId::TerraSolid_Classification);
        PointSourceId = schema.getDimensionIndex(DimensionId::TerraSolid_PointSourceId_u8);
        EchoInt = schema.getDimensionIndex(DimensionId::TerraSolid_ReturnNumber_u16);
        X = schema.getDimensionIndex(DimensionId::X_i32);
        Y = schema.getDimensionIndex(DimensionId::Y_i32);
        Z = schema.getDimensionIndex(DimensionId::Z_i32);
    } 
    else if (format == TERRASOLID_Format_2)
    {
        X = schema.getDimensionIndex(DimensionId::X_i32);
        Y = schema.getDimensionIndex(DimensionId::Y_i32);
        Z = schema.getDimensionIndex(DimensionId::Z_i32);
        Classification = schema.getDimensionIndex(DimensionId::TerraSolid_Classification);
        ReturnNumber = schema.getDimensionIndex(DimensionId::TerraSolid_ReturnNumber_u8);
        Flag = schema.getDimensionIndex(DimensionId::TerraSolid_Flag);
        Mark = schema.getDimensionIndex(DimensionId::TerraSolid_Mark);
        PointSourceId = schema.getDimensionIndex(DimensionId::TerraSolid_PointSourceId_u16);
        Intensity = schema.getDimensionIndex(DimensionId::TerraSolid_Intensity);
    } 

    Time = schema.getDimensionIndex(DimensionId::TerraSolid_Time);

    Red = schema.getDimensionIndex(DimensionId::Red_u8);
    Green = schema.getDimensionIndex(DimensionId::Green_u8);
    Blue = schema.getDimensionIndex(DimensionId::Blue_u8);
    Alpha = schema.getDimensionIndex(DimensionId::TerraSolid_Alpha);
    
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

    if ( m_header->RecogVal != 970401)
        throw terrasolid_error("Header identifier was not '970401', is this a TerraSolid .bin file?");


    setNumPoints(m_header->PntCnt);

    m_haveColor = (m_header->Color != 0);
    m_haveTime = (m_header->Time != 0);
    m_format = static_cast<TERRASOLID_Format_Type>(m_header->HdrVersion);
    
    
    if ( !( (m_format==TERRASOLID_Format_1) || (m_format == TERRASOLID_Format_2) ) )
    {
        std::ostringstream oss;
        oss << "Version was '" << m_format << "', not '" << TERRASOLID_Format_1 << "' or '" << TERRASOLID_Format_2 << "'";
        throw terrasolid_error(oss.str());
        
    }

    registerFields();
    
    m_offset = 56;
    const Schema& schema = getSchema();
    m_size = schema.getByteSize();

    delete stream;
}    


void Reader::initialize()
{
    pdal::Reader::initialize();

    log()->get(logDEBUG3) << "TerraSolid Reader::initialize format: " << m_format << std::endl;
    log()->get(logDEBUG3) << "OrgX: " << m_header->OrgX << std::endl;
    log()->get(logDEBUG3) << "OrgY: " << m_header->OrgY << std::endl;
    log()->get(logDEBUG3) << "OrgZ: " << m_header->OrgZ << std::endl;
    log()->get(logDEBUG3) << "Units: " << m_header->Units << std::endl;
    log()->get(logDEBUG3) << "Time: " << m_header->Time << std::endl;
    log()->get(logDEBUG3) << "Color: " << m_header->Color << std::endl;
    log()->get(logDEBUG3) << "Count: " << m_header->PntCnt << std::endl;
    log()->get(logDEBUG3) << "RecogVal: " << m_header->RecogVal << std::endl;

    
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
    
    double xyz_scale = 1/static_cast<double>(m_header->Units);
    
    if (m_format == TERRASOLID_Format_1)
    {
        Dimension classification(DimensionId::TerraSolid_Classification);
        schema.appendDimension(classification);

        Dimension flight_line(DimensionId::TerraSolid_PointSourceId_u8);
        schema.appendDimension(flight_line);

        Dimension echoint(DimensionId::TerraSolid_Intensity);
        schema.appendDimension(echoint);

        Dimension x(DimensionId::X_i32);
        x.setNumericScale(xyz_scale);
        x.setNumericOffset(m_header->OrgX);
        schema.appendDimension(x);

        Dimension y(DimensionId::Y_i32);
        y.setNumericScale(xyz_scale);
        y.setNumericOffset(m_header->OrgY);
        schema.appendDimension(y);

        Dimension z(DimensionId::Z_i32);
        z.setNumericScale(xyz_scale);
        z.setNumericOffset(m_header->OrgZ);
        schema.appendDimension(z);
    }

    if (m_format == TERRASOLID_Format_2)
    {
        Dimension x(DimensionId::X_i32);
        x.setNumericScale(xyz_scale);
        x.setNumericOffset(m_header->OrgX);
        schema.appendDimension(x);

        Dimension y(DimensionId::Y_i32);
        y.setNumericScale(xyz_scale);
        y.setNumericOffset(m_header->OrgY);
        schema.appendDimension(y);

        Dimension z(DimensionId::Z_i32);
        z.setNumericScale(xyz_scale);
        z.setNumericOffset(m_header->OrgZ);
        schema.appendDimension(z);

        Dimension classification(DimensionId::TerraSolid_Classification);
        schema.appendDimension(classification);

        Dimension return_no(DimensionId::TerraSolid_ReturnNumber_u8); 
        schema.appendDimension(return_no);

        Dimension flag(DimensionId::TerraSolid_Flag);
        schema.appendDimension(flag);

        Dimension mark(DimensionId::TerraSolid_Mark);
        schema.appendDimension(mark);

        Dimension flight_line(DimensionId::TerraSolid_PointSourceId_u16);
        schema.appendDimension(flight_line);

        Dimension intensity(DimensionId::TerraSolid_Intensity);
        schema.appendDimension(intensity);
    }
    
    if (m_haveTime)
    {
        Dimension time(DimensionId::TerraSolid_Time);
        time.setNumericScale(0.0002);
        time.setNumericOffset(0.0);
        schema.appendDimension(time);
    }

    if (m_haveColor)
    {
        Dimension red(DimensionId::Red_u8);
        schema.appendDimension(red);

        Dimension green(DimensionId::Green_u8);
        schema.appendDimension(green);

        Dimension blue(DimensionId::Blue_u8);
        schema.appendDimension(blue);

        Dimension alpha(DimensionId::TerraSolid_Alpha);
        schema.appendDimension(alpha);
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


boost::property_tree::ptree Reader::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Reader::toPTree();

    // add stuff here specific to this stage type

    return tree;
}

void Reader::addDefaultDimensions()
{

}


}}} // namespace pdal::driver::oci
