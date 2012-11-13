/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
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

#include <pdal/drivers/pcd/Writer.hpp>
#include <pdal/PointBuffer.hpp>

#include <iostream>
#include <algorithm>

#include <boost/algorithm/string.hpp>



#ifdef USE_PDAL_PLUGIN_PCD
MAKE_WRITER_CREATOR(pcdWriter, pdal::drivers::pcd::Writer)
CREATE_WRITER_PLUGIN(pcd, pdal::drivers::pcd::Writer)
#endif


namespace pdal
{
namespace drivers
{
namespace pcd
{


struct FileStreamDeleter
{

    template <typename T>
    void operator()(T* ptr)
    {
        ptr->flush();
        FileUtils::closeFile(ptr);
    }
};

Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
    , m_wrote_header(false)

{

    return;
}


Writer::~Writer()
{
    return;
}


void Writer::initialize()
{
    pdal::Writer::initialize();

    std::string filename = getOptions().getValueOrThrow<std::string>("filename");

    // This is so the stream gets closed down if we throw any sort of
    // exception
    m_stream = FileStreamPtr(FileUtils::createFile(filename, true), FileStreamDeleter());

    return;
}



Options Writer::getDefaultOptions()
{
    Options options;

    Option filename("filename", "", "Filename to write PCD file to");


    options.add(filename);


    return options;
}


void Writer::writeBegin(boost::uint64_t /*targetNumPointsToWrite*/)
{
    return;
}


void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
    m_stream.reset();
    m_stream = FileStreamPtr();
    m_wrote_header = false;
    return;
}

void Writer::WriteHeader(pdal::Schema const& schema)
{

    boost::optional<Dimension const&> dimRed = schema.getDimensionOptional("Red");
    boost::optional<Dimension const&> dimGreen = schema.getDimensionOptional("Green");
    boost::optional<Dimension const&> dimBlue = schema.getDimensionOptional("Blue");
    
    bool bHaveColor(false);
    bool bRGBPacked  = getOptions().getValueOrDefault<bool>("pack_rgb", true);

    
    
    if (dimRed && dimGreen && dimBlue)
        bHaveColor = true;

    *m_stream << "# .PCD v.7 - Point Cloud Data file format" << std::endl;

    *m_stream << "VERSION 0.7" << std::endl;    

    *m_stream << "FIELDS x y z";
    if (bHaveColor)
    {
        if(bRGBPacked)
            *m_stream << " rgb";
        else
            *m_stream << " r g b";
    }
    *m_stream << std::endl;
    
    *m_stream << "SIZE 4 4 4";
    if (bHaveColor)
    {
        if(bRGBPacked)
            *m_stream << " 1";
        else
            *m_stream << " 1 1 1";
    }
    *m_stream << std::endl;
    
    *m_stream << "TYPE f f f";
    if (bHaveColor)
    {
        if(bRGBPacked)
            *m_stream << " f";
        else
            *m_stream << " u u u";
    }
    *m_stream << std::endl;
    
    *m_stream << "COUNT 1 1 1";
    if (bHaveColor)
    {
        if(bRGBPacked)
            *m_stream << " 1";
        else
            *m_stream << " 1 1 1";
    }
    *m_stream << std::endl;    

    boost::uint64_t width = getPrevStage().getNumPoints();
    *m_stream << "WIDTH " << width << std::endl;
    
    *m_stream << "HEIGHT 1" << std::endl;
    *m_stream << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    *m_stream << "POINTS " << width << std::endl;
    
    *m_stream << "DATA ascii" << std::endl;
    return;
}

std::string Writer::getStringRepresentation(PointBuffer const& data,
        Dimension const& d,
        std::size_t pointIndex) const
{
    std::ostringstream output;

    float flt(0.0);
    boost::int8_t i8(0);
    boost::uint8_t u8(0);
    boost::int16_t i16(0);
    boost::uint16_t u16(0);
    boost::int32_t i32(0);
    boost::uint32_t u32(0);
    boost::int64_t i64(0);
    boost::uint64_t u64(0);

    boost::uint32_t size = d.getByteSize();

    bool bHaveScaling = !Utils::compare_distance(d.getNumericScale(), 0.0);
    
    // FIXME: Allow selective scaling of requested dimensions
    if (bHaveScaling)
    {
        output.setf(std::ios::fixed, std::ios::floatfield);
        output.precision(Utils::getStreamPrecision(d.getNumericScale()));
    }
    switch (d.getInterpretation())
    {
        case dimension::Float:
            if (size == 4)
            {
                flt = data.getField<float>(d, pointIndex);
                output << static_cast<double>(flt);
            }
            if (size == 8)
            {
                output << data.getField<double>(d, pointIndex);
            }
            break;

        case dimension::SignedInteger:
        case dimension::SignedByte:
            if (size == 1)
            {
                i8 = data.getField<boost::int8_t>(d, pointIndex);
                output << d.applyScaling<boost::int8_t>(i8);
            }
            if (size == 2)
            {
                i16 = data.getField<boost::int16_t>(d, pointIndex);
                output << d.applyScaling<boost::int16_t>(i16);
            }
            if (size == 4)
            {
                i32 = data.getField<boost::int32_t>(d, pointIndex);
                output << d.applyScaling<boost::int32_t>(i32);
            }
            if (size == 8)
            {
                i64 = data.getField<boost::int64_t>(d, pointIndex);
                output << d.applyScaling<boost::int64_t>(i64);
            }
            break;

        case dimension::UnsignedInteger:
        case dimension::UnsignedByte:
            if (size == 1)
            {
                u8 = data.getField<boost::uint8_t>(d, pointIndex);
                output << d.applyScaling<boost::uint8_t>(u8);
            }
            if (size == 2)
            {
                u16 = data.getField<boost::uint16_t>(d, pointIndex);
                output << d.applyScaling<boost::uint16_t>(u16);
            }
            if (size == 4)
            {
                u32 = data.getField<boost::uint32_t>(d, pointIndex);
                output << d.applyScaling<boost::uint32_t>(u32);
            }
            if (size == 8)
            {
                u64 = data.getField<boost::uint64_t>(d, pointIndex);
                output << d.applyScaling<boost::uint64_t>(u64);
            }
            break;

        case dimension::Pointer:    // stored as 64 bits, even on a 32-bit box
        case dimension::Undefined:
            break;
    }

    return output.str();
}



boost::uint32_t Writer::writeBuffer(const PointBuffer& data)
{

    if (!m_wrote_header)
    {
        WriteHeader(data.getSchema());
        m_wrote_header = true;
    }

    pdal::Schema const& schema = data.getSchema();
    
    Dimension const& x = schema.getDimension("X");
    Dimension const& y = schema.getDimension("Y");
    Dimension const& z = schema.getDimension("Z");
    
    boost::optional<Dimension const&> dimRed = schema.getDimensionOptional("Red");
    boost::optional<Dimension const&> dimGreen = schema.getDimensionOptional("Green");
    boost::optional<Dimension const&> dimBlue = schema.getDimensionOptional("Blue");

    boost::uint32_t pointIndex(0);
    
    bool bHaveColor(false);
    if (dimRed && dimGreen && dimBlue)
        bHaveColor = true;
        
    bool bRGBPacked = getOptions().getValueOrDefault<bool>("pack_rgb", true);
        
    while (pointIndex != data.getNumPoints())
    {
        *m_stream << getStringRepresentation(data, x, pointIndex);
        *m_stream << " ";
        *m_stream << getStringRepresentation(data, y, pointIndex);
        *m_stream << " ";
        *m_stream << getStringRepresentation(data, z, pointIndex);
        *m_stream << " ";
        
        std::string color;
        if (bHaveColor) 
        {
            if (bRGBPacked)
            {
                boost::uint16_t r = data.getField<boost::uint16_t>(*dimRed, pointIndex);
                boost::uint16_t g = data.getField<boost::uint16_t>(*dimGreen, pointIndex);
                boost::uint16_t b = data.getField<boost::uint16_t>(*dimBlue, pointIndex);
                int rgb =  ((int)r) << 16 | ((int)g) << 8 | ((int)b);
                m_stream->precision(8);
                *m_stream << static_cast<float>(rgb);
            }
            else
            {
                std::string color;
                color = getStringRepresentation(data, *dimRed, pointIndex);
                *m_stream << color;
                *m_stream << " ";

                color = getStringRepresentation(data, *dimGreen, pointIndex);
                *m_stream << color;
                *m_stream << " ";

                color = getStringRepresentation(data, *dimBlue, pointIndex);
                *m_stream << color;
                *m_stream << " ";
            }
        }
        
        *m_stream << "\n";

        pointIndex++;

    }



    return data.getNumPoints();
}


boost::property_tree::ptree Writer::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Writer::toPTree();

    // add stuff here specific to this stage type

    return tree;
}


}
}
} // namespaces
