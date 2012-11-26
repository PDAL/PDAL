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

#include <pdal/drivers/text/Writer.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/pdal_macros.hpp>

#include <iostream>
#include <algorithm>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/erase.hpp>


#ifdef USE_PDAL_PLUGIN_TEXT
MAKE_WRITER_CREATOR(textWriter, pdal::drivers::text::Writer)
CREATE_WRITER_PLUGIN(text, pdal::drivers::text::Writer)
#endif


namespace pdal
{
namespace drivers
{
namespace text
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
    , bWroteHeader(false)
    , bWroteFirstPoint(false)

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

    Option delimiter("delimiter", ",", "Delimiter to use for writing text");
    Option newline("newline", "\n", "Newline character to use for additional lines");
    Option quote_header("quote_header", true, "Write dimension names in quotes");
    Option filename("filename", "", "Filename to write CSV file to");


    options.add(filename);
    options.add(delimiter);
    options.add(newline);
    options.add(quote_header);

    return options;
}


void Writer::writeBegin(boost::uint64_t /*targetNumPointsToWrite*/)
{
    return;
}


void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
    std::string outputType = getOptions().getValueOrDefault<std::string>("format", "csv");
    bool bGeoJSON = boost::iequals(outputType, "GEOJSON");
    if (bGeoJSON)
    {
        *m_stream << "]}";
        std::string callback = getOptions().getValueOrDefault<std::string>("jscallback", "");
        
        if (callback.size())
        {
            *m_stream  <<")";
        }        
    }

    m_stream.reset();
    m_stream = FileStreamPtr();
    bWroteHeader = false;
    bWroteFirstPoint = false;
    return;
}

std::vector<boost::tuple<std::string, std::string> >  Writer::getDimensionOrder(Schema const& schema) const
{
    boost::char_separator<char> separator(",");
    std::string dimension_order = getOptions().getValueOrDefault<std::string>("order", "");
    
    boost::erase_all(dimension_order, " "); // Wipe off spaces
    std::vector<boost::tuple<std::string, std::string> >  output;
    if (dimension_order.size())
    {   
        
        // Put all the names in a map. We'll add the names in order to our 
        // output list as we find them, and we'll remove those that we've added 
        // from the map as we go. We'll then add what's left at the end based on 
        // whether or not the user wants to do so.
        std::map<std::string, bool> all_names;
        schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();
        schema::index_by_index::const_iterator iter = dims.begin();
        schema::index_by_name const& name_index = schema.getDimensions().get<schema::name>();
        while (iter != dims.end())
        {
            all_names.insert(std::pair<std::string, bool>(iter->getName(), true));
            ++iter;
        }
                
        tokenizer parameters(dimension_order, separator);
        for (tokenizer::iterator t = parameters.begin(); t != parameters.end(); ++t)
        {
            boost::optional<Dimension const&> d = schema.getDimensionOptional(*t);
            if (d)
            {
                if (boost::equals(d->getName(), *t))
                {
                    output.push_back( boost::tuple<std::string, std::string>(d->getName(), d->getNamespace()));

                    std::map<std::string, bool>::iterator i = all_names.find(d->getName());
                    all_names.erase(i);
                }
            } 
            else
            {
                std::ostringstream oss;
                oss << "Dimension not found with name '" << *t <<"'";
                throw pdal::dimension_not_found(oss.str());
            }
        }
        
        bool bKeep = getOptions().getValueOrDefault<bool>("keep_unspecified", true);
        if (bKeep)
        {
            std::map<std::string, bool>::const_iterator i = all_names.begin();
            while (i!= all_names.end())
            {
                output.push_back( boost::tuple<std::string, std::string>(i->first, ""));
                ++i;
            }
        } 
        else
        {
            return output;
        }
    }
    else
    {
        // No order was specified, just use the order of the schema
        schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();
        schema::index_by_index::const_iterator iter = dims.begin();
        while (iter != dims.end())
        {
            if (!iter->isIgnored())
                output.push_back(iter->getName());
            ++iter;
        }        
    }
    
    return output;
    
}

void Writer::WriteGeoJSONHeader(pdal::Schema const& schema)
{
    std::string callback = getOptions().getValueOrDefault<std::string>("jscallback", "");
    
    if (callback.size())
    {
        *m_stream << callback <<"(";
    }
    *m_stream << "{ \"type\": \"FeatureCollection\", \"features\": [";
    return;
}

void Writer::WriteCSVHeader(pdal::Schema const& schema)
{
    bool isQuoted = getOptions().getValueOrDefault<bool>("quote_header", true);
    std::string newline = getOptions().getValueOrDefault<std::string>("newline", "\n");
    std::string delimiter = getOptions().getValueOrDefault<std::string>("delimiter",",");
    
    // Spaces get stripped off, so if the user set a space, 
    // we'll get a 0-sized value. We'll set to space in that 
    // instance. This means you can't use \n as a delimiter.
    if (delimiter.size() == 0)
        delimiter = " ";    

    std::string order = getOptions().getValueOrDefault<std::string>("order", "");
    boost::erase_all(order, " "); // Wipe off spaces
    
    log()->get(logDEBUG) << "Dimension order specified '" << order << "'" << std::endl;
    
    std::vector<boost::tuple<std::string, std::string> > dimensions = getDimensionOrder(schema);
    log()->get(logDEBUG) << "dimensions.size(): " << dimensions.size( ) <<std::endl;
    std::ostringstream oss;
    oss << "Dimension order obtained '";
    for (std::vector<boost::tuple<std::string, std::string> >::const_iterator i = dimensions.begin(); i != dimensions.end(); i++)
    {
        std::string name = i->get<0>();
        std::string namespc = i->get<1>();
        
        Dimension const& d = schema.getDimension(name, namespc);
        if (d.isIgnored())
        {
            i++;
            if (i == dimensions.end())
                break;
            else
                continue;
        }
                
        if (i != dimensions.begin())
            oss <<  ",";
        oss <<i->get<0>();
    }
    log()->get(logDEBUG) << oss.str() << std::endl;


    std::vector<boost::tuple<std::string, std::string> >::const_iterator iter = dimensions.begin();

    bool bWroteProperty(false);    
    while (iter != dimensions.end())
    {
        if (bWroteProperty)
            *m_stream << delimiter;
                    
        Dimension const& d = schema.getDimension(iter->get<0>(), iter->get<1>());
        if (d.isIgnored())
        {
            iter++;
            bWroteProperty = false;            
            if (iter == dimensions.end())
                break;
            else
                continue;
        }
        if (isQuoted)
            *m_stream << "\"";
        *m_stream << iter->get<0>();
        if (isQuoted)
            *m_stream<< "\"";
        iter++;
        bWroteProperty = true;        
    }
    *m_stream << newline;

    
}

void Writer::WritePCDHeader(pdal::Schema const& schema)
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

void Writer::WriteHeader(pdal::Schema const& schema)
{

    schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();

    bool isQuoted = getOptions().getValueOrDefault<bool>("quote_header", true);
    bool bWriteHeader = getOptions().getValueOrDefault<bool>("write_header", true);
    std::string newline = getOptions().getValueOrDefault<std::string>("newline", "\n");
    std::string delimiter = getOptions().getValueOrDefault<std::string>("delimiter",",");
    
    // Spaces get stripped off, so if the user set a space, 
    // we'll get a 0-sized value. We'll set to space in that 
    // instance. This means you can't use \n as a delimiter.
    if (delimiter.size() == 0)
        delimiter = " ";    

    if (!bWriteHeader)
    {
        log()->get(logDEBUG) << "Not writing header" << std::endl;
        bWroteHeader = true;
        return;
        
    }
    
    log()->get(logDEBUG) << "Writing header to filename: " << getOptions().getValueOrThrow<std::string>("filename") << std::endl;
    

    
    // If we're bGeoJSON, we're just going to write the preamble for FeatureCollection, 
    // and let the rest happen in writeBuffer
    std::string outputType = getOptions().getValueOrDefault<std::string>("format", "csv");
    bool bGeoJSON = boost::iequals(outputType, "GEOJSON");
    if (bGeoJSON)
    {
        WriteGeoJSONHeader(schema);
        return;
    }

    bool bCSV = boost::iequals(outputType, "CSV");
    if (bCSV)
    {
        WriteCSVHeader(schema);
        return;
    }
        


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

    bool bHaveScaling = !Utils::compare_distance(d.getNumericScale(), 1.0);
    
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


void Writer::WriteCSVBuffer(const PointBuffer& data)
{
    std::string newline = getOptions().getValueOrDefault<std::string>("newline", "\n");
    std::string delimiter = getOptions().getValueOrDefault<std::string>("delimiter",",");
    if (delimiter.size() == 0)
        delimiter = " ";
        
    boost::uint32_t pointIndex(0);

    pdal::Schema const& schema = data.getSchema();

    std::string order = getOptions().getValueOrDefault<std::string>("order", "");
    boost::erase_all(order, " "); // Wipe off spaces
    
    std::vector<boost::tuple<std::string, std::string> > dimensions = getDimensionOrder(schema);
    
    while (pointIndex != data.getNumPoints())
    {
        std::vector<boost::tuple<std::string, std::string> >::const_iterator iter =  dimensions.begin();

        bool bFirstProperty(true);
        while (iter != dimensions.end())
        {
            if (!bFirstProperty)
                *m_stream << delimiter;
                
            Dimension const& d = schema.getDimension(iter->get<0>(), iter->get<1>());
            if (d.isIgnored())
            {
                iter++;
                continue;
            }

            *m_stream << getStringRepresentation(data, d, pointIndex);
            
            bFirstProperty = false;
            iter++;

        }
        *m_stream << newline;

        pointIndex++;
        if (!bWroteFirstPoint)
        {
            bWroteFirstPoint = true;
        }

    }    
}

void Writer::WritePCDBuffer(const PointBuffer& data)
{
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

}

void Writer::WriteGeoJSONBuffer(const PointBuffer& data)
{
    boost::uint32_t pointIndex(0);    
    pdal::Schema const& schema = data.getSchema();
    while (pointIndex != data.getNumPoints())
    {
        Dimension const& dimX = schema.getDimension("X");
        Dimension const& dimY = schema.getDimension("Y");
        Dimension const& dimZ = schema.getDimension("Z");
        
        if (bWroteFirstPoint)
            *m_stream << ",";
            
        *m_stream << "{ \"type\":\"Feature\",\"geometry\": { \"type\": \"Point\", \"coordinates\": [";
        *m_stream << getStringRepresentation(data, dimX, pointIndex) << ",";
        *m_stream << getStringRepresentation(data, dimY, pointIndex) << ",";
        *m_stream << getStringRepresentation(data, dimZ, pointIndex) << "]},";
        
        *m_stream << "\"properties\": {";

        std::vector<boost::tuple<std::string, std::string> > dimensions = getDimensionOrder(schema);
        std::vector<boost::tuple<std::string, std::string> >::const_iterator iter =  dimensions.begin();
        
        bool bFirstProperty(true);
        while (iter != dimensions.end())
        {
            Dimension const& d = schema.getDimension(iter->get<0>(), iter->get<1>());
            if (d.isIgnored())
            {
                iter++;
                continue;
            }

            if (!bFirstProperty)
                *m_stream << ",";

            *m_stream << "\"" << iter->get<0>() << "\":";
            

            *m_stream << "\"" << getStringRepresentation(data, d, pointIndex) <<"\"";
            
            iter++;
            bFirstProperty = false;
        }
        
        
        *m_stream << "}"; // end properties

        *m_stream << "}"; // end feature
        
        pointIndex++;
        if (!bWroteFirstPoint)
        {
            bWroteFirstPoint = true;
        }
        
        
    }    
}

boost::uint32_t Writer::writeBuffer(const PointBuffer& data)
{

    std::string outputType = getOptions().getValueOrDefault<std::string>("format", "csv");
    bool bCSV = boost::iequals(outputType, "CSV");
    bool bGeoJSON = boost::iequals(outputType, "GEOJSON");
    bool bPCD = boost::iequals(outputType, "PCD");    

    if (!bWroteHeader)
    {
        WriteHeader(data.getSchema());
        bWroteHeader = true;
    }
        
    if (bCSV)
    {
        WriteCSVBuffer(data);
    
    } else if (bGeoJSON)
    {
        WriteGeoJSONBuffer(data);
                    
    } else if (bPCD)
    {
        WritePCDBuffer(data);
    }



    return data.getNumPoints();
}


}
}
} // namespaces
