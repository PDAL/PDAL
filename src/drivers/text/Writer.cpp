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
#include <pdal/Algorithm.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/pdal_internal.hpp>

#include <iostream>
#include <algorithm>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/tokenizer.hpp>

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


Options Writer::getDefaultOptions()
{
    Options options;

    options.add("delimiter", ",", "Delimiter to use for writing text");
    options.add("newline", "\n", "Newline character to use for additional "
        "lines");
    options.add("quote_header", true, "Write dimension names in quotes");
    options.add("filename", "", "Filename to write CSV file to");

    return options;
}


void Writer::processOptions(const Options& ops)
{
    m_filename = ops.getValueOrThrow<std::string>("filename");
    m_stream = FileStreamPtr(FileUtils::createFile(m_filename, true),
        FileStreamDeleter());
    m_outputType = ops.getValueOrDefault<std::string>("format", "csv");
    boost::to_upper(m_outputType);
    m_callback = ops.getValueOrDefault<std::string>("jscallback", "");
    m_writeAllDims = ops.getValueOrDefault<bool>("keep_unspecified", true);
    m_dimOrder = ops.getValueOrDefault<std::string>("order", "");
    m_writeHeader = ops.getValueOrDefault<bool>("write_header", true);
    m_newline = ops.getValueOrDefault<std::string>("newline", "\n");
    m_delimiter = ops.getValueOrDefault<std::string>("delimiter", ",");
    if (m_delimiter.empty())
        m_delimiter = " ";
    m_quoteHeader = ops.getValueOrDefault<bool>("quote_header", true);
    m_packRgb = ops.getValueOrDefault<bool>("pack_rgb", true);
}


void Writer::ready(PointContext ctx)
{
    typedef boost::tokenizer<boost::char_separator<char>> tokenizer;

    // Find the dimensions listed and put them on the id list.
    boost::char_separator<char> separator(",");
    boost::erase_all(m_dimOrder, " "); // Wipe off spaces
    tokenizer sdims(m_dimOrder, separator);
    for (tokenizer::iterator ti = sdims.begin(); ti != sdims.end(); ++ti)
    {
        Dimension::Id::Enum d = ctx.findDim(*ti);
        if (d == Dimension::Id::Unknown)
        {
            std::ostringstream oss;
            oss << "Dimension not found with name '" << *ti <<"'";
            throw pdal::dimension_not_found(oss.str());
        }
        m_dims.push_back(d);
    }
    // Add the rest of the dimensions to the list if we're doing that.
    // Yes, this isn't efficient when, but it's simple.
    if (m_dimOrder.empty() || m_writeAllDims)
    {
        Dimension::IdList all = ctx.dims();
        for (auto di = all.begin(); di != all.end(); ++di)
            if (!Algorithm::contains(m_dims, *di))
                m_dims.push_back(*di);
    }

    if (!m_writeHeader)
        log()->get(LogLevel::Debug) << "Not writing header" << std::endl;
    else
        writeHeader(ctx);
}


void Writer::writeHeader(PointContext ctx)
{
    log()->get(LogLevel::Debug) << "Writing header to filename: " <<
        m_filename << std::endl;
    if (m_outputType == "GEOJSON")
        writeGeoJSONHeader();
    else if (m_outputType == "CSV")
        writeCSVHeader(ctx);
    else if (m_outputType == "PCD")
        writePCDHeader();
}


void Writer::writeFooter()
{
    if (m_outputType == "GEOJSON")
    {
        *m_stream << "]}";
        if (m_callback.size())
            *m_stream  <<")";
    }
    m_stream.reset();
}


void Writer::writeGeoJSONHeader()
{
    if (m_callback.size())
        *m_stream << m_callback <<"(";
    *m_stream << "{ \"type\": \"FeatureCollection\", \"features\": [";
}


void Writer::writeCSVHeader(PointContext ctx)
{
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        if (di != m_dims.begin())
            *m_stream << m_delimiter;

        if (m_quoteHeader)
            *m_stream << "\"" << ctx.dimName(*di) << "\"";
        else
            *m_stream << ctx.dimName(*di);
    }
    *m_stream << m_newline;
}


void Writer::writePCDHeader()
{
    bool haveColor = (Algorithm::contains(m_dims, Dimension::Id::Red) &&
        Algorithm::contains(m_dims, Dimension::Id::Green) &&
        Algorithm::contains(m_dims, Dimension::Id::Blue));

    *m_stream << "# .PCD v.7 - Point Cloud Data file format" << std::endl;
    *m_stream << "VERSION 0.7" << std::endl;
    *m_stream << "FIELDS x y z";
    if (haveColor)
        *m_stream << (m_packRgb ? "rgb" : "r g b");
    *m_stream << std::endl;

    *m_stream << "SIZE 4 4 4";
    if (haveColor)
        *m_stream << (m_packRgb ? " 1" : " 1 1 1");
    *m_stream << std::endl;

    *m_stream << "TYPE f f f";
    if (haveColor)
        *m_stream << (m_packRgb ? " f" : " u u u");
    *m_stream << std::endl;

    *m_stream << "COUNT 1 1 1";
    if (haveColor)
        *m_stream << (m_packRgb ? " 1" : " 1 1 1");
    *m_stream << std::endl;
}


void Writer::writeCSVBuffer(const PointBuffer& data)
{
    boost::uint32_t pointIndex(0);

    for (PointId idx = 0; idx < data.size(); ++idx)
    {
        for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
        {
            if (di != m_dims.begin())
                *m_stream << m_delimiter;
            *m_stream << data.getFieldAs<double>(*di, idx);
        }
        *m_stream << m_newline;
    }
}

void Writer::writePCDBuffer(const PointBuffer& data)
{
    using namespace Dimension;

    *m_stream << "WIDTH " << data.size() << std::endl;
    *m_stream << "HEIGHT 1" << std::endl;
    *m_stream << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    *m_stream << "POINTS " << data.size() << std::endl;
    *m_stream << "DATA ascii" << std::endl;

    bool haveColor = (Algorithm::contains(m_dims, Id::Red) &&
        Algorithm::contains(m_dims, Id::Green) &&
        Algorithm::contains(m_dims, Id::Blue));

    for (PointId idx = 0; idx < data.size(); ++idx)
    {
        *m_stream << data.getFieldAs<double>(Id::X, idx) << " ";
        *m_stream << data.getFieldAs<double>(Id::Y, idx) << " ";
        *m_stream << data.getFieldAs<double>(Id::Z, idx) << " ";

        std::string color;
        if (haveColor)
        {
            if (m_packRgb)
            {
                auto r = data.getFieldAs<uint16_t>(Id::Red, idx);
                auto g = data.getFieldAs<uint16_t>(Id::Green, idx);
                auto b = data.getFieldAs<uint16_t>(Id::Blue, idx);
                int rgb = ((int)r) << 16 | ((int)g) << 8 | ((int)b);
                m_stream->precision(8);
                *m_stream << static_cast<float>(rgb);
            }
            else
            {
                *m_stream << data.getFieldAs<double>(Id::Red, idx) << " ";
                *m_stream << data.getFieldAs<double>(Id::Green, idx) << " ";
                *m_stream << data.getFieldAs<double>(Id::Blue, idx) << " ";
            }
        }
        *m_stream << "\n";
    }
}


void Writer::writeGeoJSONBuffer(const PointBuffer& data)
{
    using namespace Dimension;

    for (PointId idx = 0; idx < data.size(); ++idx)
    {
        if (idx)
            *m_stream << ",";

        *m_stream << "{ \"type\":\"Feature\",\"geometry\": "
            "{ \"type\": \"Point\", \"coordinates\": [";
        *m_stream << data.getFieldAs<double>(Id::X, idx) << ",";
        *m_stream << data.getFieldAs<double>(Id::Y, idx) << ",";
        *m_stream << data.getFieldAs<double>(Id::Z, idx) << "]},";

        *m_stream << "\"properties\": {";

        for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
        {
            if (di != m_dims.begin())
                *m_stream << ",";

            *m_stream << "\"" << data.dimName(*di) << "\":";
            *m_stream << "\"";
            *m_stream << data.getFieldAs<double>(*di, idx);
            *m_stream <<"\"";
        }
        *m_stream << "}"; // end properties
        *m_stream << "}"; // end feature
    }
}


void Writer::write(const PointBuffer& data)
{
    if (m_outputType == "CSV")
        writeCSVBuffer(data);
    else if (m_outputType == "GEOJSON")
        writeGeoJSONBuffer(data);
    else if (m_outputType == "PCD")
        writePCDBuffer(data);
}


void Writer::done(PointContext ctx)
{
    writeFooter();
}

}
}
} // namespaces
