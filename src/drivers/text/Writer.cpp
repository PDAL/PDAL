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

Writer::Writer(const Options& options)
    : pdal::Writer(options)
    , bWroteHeader(false)
    , bWroteFirstPoint(false)
{}


void Writer::processOptions(const Options& options)
{
    std::string filename = options.getValueOrThrow<std::string>("filename");

    // This is so the stream gets closed down if we throw any sort of
    // exception
    m_stream = FileStreamPtr(FileUtils::createFile(filename, true),
        FileStreamDeleter());
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
        std::vector<DimensionPtr> dims = schema.getDimensions();
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            DimensionPtr d = *di;
            all_names.insert(std::make_pair(d->getName(), true));
        }

        tokenizer parameters(dimension_order, separator);
        for (tokenizer::iterator t = parameters.begin(); t != parameters.end(); ++t)
        {
            DimensionPtr d = schema.getDimension(*t);
            if (d)
            {
                if (boost::equals(d->getName(), *t))
                {
                    output.push_back(boost::tuple<std::string, std::string>(d->getName(), d->getNamespace()));

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
                output.push_back(boost::tuple<std::string, std::string>(i->first, ""));
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
        std::vector<DimensionPtr> dims = schema.getDimensions();
        for (auto di = dims.begin(); di != dims.end(); ++di)
        {
            DimensionPtr d = *di;
            if (!d->isIgnored())
                output.push_back(d->getName());
        }
    }

    return output;
}

void Writer::WriteGeoJSONHeader(pdal::Schema const&)
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
    log()->get(logDEBUG) << "dimensions.size(): " << dimensions.size() <<std::endl;
    std::ostringstream oss;
    oss << "Dimension order obtained '";
    for (std::vector<boost::tuple<std::string, std::string> >::const_iterator i = dimensions.begin(); i != dimensions.end(); i++)
    {
        std::string name = i->get<0>();
        std::string namespc = i->get<1>();

        DimensionPtr d = schema.getDimension(name, namespc);
        if (d->isIgnored())
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

        DimensionPtr d = schema.getDimension(iter->get<0>(), iter->get<1>());
        if (d->isIgnored())
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
    DimensionPtr dimRed = schema.getDimension("Red");
    DimensionPtr dimGreen = schema.getDimension("Green");
    DimensionPtr dimBlue = schema.getDimension("Blue");

    bool bHaveColor(false);
    bool bRGBPacked = getOptions().getValueOrDefault<bool>("pack_rgb", true);

    if (dimRed && dimGreen && dimBlue)
        bHaveColor = true;

    *m_stream << "# .PCD v.7 - Point Cloud Data file format" << std::endl;

    *m_stream << "VERSION 0.7" << std::endl;

    *m_stream << "FIELDS x y z";
    if (bHaveColor)
    {
        if (bRGBPacked)
            *m_stream << " rgb";
        else
            *m_stream << " r g b";
    }
    *m_stream << std::endl;

    *m_stream << "SIZE 4 4 4";
    if (bHaveColor)
    {
        if (bRGBPacked)
            *m_stream << " 1";
        else
            *m_stream << " 1 1 1";
    }
    *m_stream << std::endl;

    *m_stream << "TYPE f f f";
    if (bHaveColor)
    {
        if (bRGBPacked)
            *m_stream << " f";
        else
            *m_stream << " u u u";
    }
    *m_stream << std::endl;

    *m_stream << "COUNT 1 1 1";
    if (bHaveColor)
    {
        if (bRGBPacked)
            *m_stream << " 1";
        else
            *m_stream << " 1 1 1";
    }
    *m_stream << std::endl;

//ABELL - Get point count from PointBuffer
//    boost::uint64_t width = getPrevStage().getNumPoints();
boost::uint64_t width = 0;
    *m_stream << "WIDTH " << width << std::endl;

    *m_stream << "HEIGHT 1" << std::endl;
    *m_stream << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    *m_stream << "POINTS " << width << std::endl;

    *m_stream << "DATA ascii" << std::endl;
    return;
}

void Writer::WriteHeader(pdal::Schema const& schema)
{

    //schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();

//    bool isQuoted = getOptions().getValueOrDefault<bool>("quote_header", true);
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

    bool bPCD = boost::iequals(outputType, "PCD");
    if (bPCD)
    {
        WritePCDHeader(schema);
        return;
    }


    return;
}

void Writer::putStringRepresentation(PointBuffer const& data,
    DimensionPtr d, std::size_t pointIndex, std::ostream& output)
{
    std::streamsize old_precision = output.precision();

    bool bHaveScaling = !Utils::compare_distance(d->getNumericScale(), 1.0);

    // FIXME: Allow selective scaling of requested dimensions
    if (bHaveScaling)
    {
        output.setf(std::ios::fixed, std::ios::floatfield);
        output.precision(Utils::getStreamPrecision(d->getNumericScale()));
    }
    switch (d->getInterpretation())
    {
        case dimension::Float:
        case dimension::SignedInteger:
        case dimension::UnsignedInteger:
            output << data.getFieldAs<double>(d, pointIndex);
            break;
        case dimension::Undefined:
            break;
    }

    if (bHaveScaling)
    {
        output.precision(old_precision);
        output.unsetf(std::ios::fixed);
        output.unsetf(std::ios::floatfield);
    }
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

    while (pointIndex != data.size())
    {
        std::vector<boost::tuple<std::string, std::string> >::const_iterator iter =  dimensions.begin();

        bool bFirstProperty(true);
        while (iter != dimensions.end())
        {
            if (!bFirstProperty)
                *m_stream << delimiter;

            DimensionPtr d = schema.getDimension(iter->get<0>(), iter->get<1>());
            if (d->isIgnored())
            {
                iter++;
                continue;
            }

            putStringRepresentation(data, d, pointIndex, *m_stream);

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

    DimensionPtr x = schema.getDimension("X");
    DimensionPtr y = schema.getDimension("Y");
    DimensionPtr z = schema.getDimension("Z");

    DimensionPtr dimRed = schema.getDimension("Red");
    DimensionPtr dimGreen = schema.getDimension("Green");
    DimensionPtr dimBlue = schema.getDimension("Blue");

    bool bHaveColor(false);
    if (dimRed && dimGreen && dimBlue)
        bHaveColor = true;

    bool bRGBPacked = getOptions().getValueOrDefault<bool>("pack_rgb", true);

    boost::uint32_t idx(0);
    while (idx != data.size())
    {
        putStringRepresentation(data, x, idx, *m_stream);
        *m_stream << " ";
        putStringRepresentation(data, y, idx, *m_stream);
        *m_stream << " ";
        putStringRepresentation(data, z, idx, *m_stream);
        *m_stream << " ";

        std::string color;
        if (bHaveColor)
        {
            if (bRGBPacked)
            {
                uint16_t r = data.getFieldAs<uint16_t>(dimRed, idx, false);
                uint16_t g = data.getFieldAs<uint16_t>(dimGreen, idx, false);
                uint16_t b = data.getFieldAs<uint16_t>(dimBlue, idx, false);
                int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
                m_stream->precision(8);
                *m_stream << static_cast<float>(rgb);
            }
            else
            {
                putStringRepresentation(data, dimRed, idx, *m_stream);
                *m_stream << " ";

                putStringRepresentation(data, dimGreen, idx, *m_stream);
                *m_stream << " ";

                putStringRepresentation(data, dimBlue, idx, *m_stream);
                *m_stream << " ";
            }
        }

        *m_stream << "\n";
        idx++;
    }
}

void Writer::WriteGeoJSONBuffer(const PointBuffer& data)
{
    boost::uint32_t pointIndex(0);
    pdal::Schema const& schema = data.getSchema();
    std::vector<boost::tuple<std::string, std::string> > ordering = getDimensionOrder(schema);
    std::vector<boost::tuple<std::string, std::string> >::const_iterator ord =  ordering.begin();

    std::vector<DimensionPtr> dimensions;
//    bool bFirstProperty(true);
    while (ord != ordering.end())
    {
        DimensionPtr d = schema.getDimension(ord->get<0>(), ord->get<1>());
        dimensions.push_back(d);
        ord++;
    }
    DimensionPtr dimX = schema.getDimension("X");
    DimensionPtr dimY = schema.getDimension("Y");
    DimensionPtr dimZ = schema.getDimension("Z");

    while (pointIndex != data.size())
    {
        if (bWroteFirstPoint)
            *m_stream << ",";

        *m_stream << "{ \"type\":\"Feature\",\"geometry\": { \"type\": \"Point\", \"coordinates\": [";
        putStringRepresentation(data, dimX, pointIndex, *m_stream);
        *m_stream << ",";
        putStringRepresentation(data, dimY, pointIndex, *m_stream);
        *m_stream << ",";
        putStringRepresentation(data, dimZ, pointIndex, *m_stream);
        *m_stream << "]},";

        *m_stream << "\"properties\": {";

        auto di = dimensions.begin();

        bool bFirstProperty(true);
        while (di != dimensions.end())
        {
            DimensionPtr d = *di;
            if (d->isIgnored())
            {
                di++;
                continue;
            }

            if (!bFirstProperty)
                *m_stream << ",";

            *m_stream << "\"" << d->getName() << "\":";

            *m_stream << "\"";
            putStringRepresentation(data, d, pointIndex, *m_stream);
            *m_stream <<"\"";

            di++;
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

    }
    else if (bGeoJSON)
    {
        WriteGeoJSONBuffer(data);

    }
    else if (bPCD)
    {
        WritePCDBuffer(data);
    }
    return data.size();
}


}
}
} // namespaces
