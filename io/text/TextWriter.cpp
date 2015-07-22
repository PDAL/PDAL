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

#include "TextWriter.hpp"

#include <pdal/pdal_export.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/Algorithm.hpp>

#include <algorithm>
#include <iostream>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/tokenizer.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.text",
    "Text Writer",
    "http://pdal.io/stages/writers.text.html" );

CREATE_STATIC_PLUGIN(1, 0, TextWriter, Writer, s_info)

std::string TextWriter::getName() const { return s_info.name; }

struct FileStreamDeleter
{

    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
        {
            ptr->flush();
            FileUtils::closeFile(ptr);
        }
    }
};


Options TextWriter::getDefaultOptions()
{
    Options options;

    options.add("delimiter", ",", "Delimiter to use for writing text");
    options.add("newline", "\n", "Newline character to use for additional "
        "lines");
    options.add("quote_header", true, "Write dimension names in quotes");
    options.add("filename", "", "Filename to write CSV file to");

    return options;
}


void TextWriter::processOptions(const Options& ops)
{
    m_filename = ops.getValueOrThrow<std::string>("filename");
    m_stream = FileStreamPtr(FileUtils::createFile(m_filename, true),
        FileStreamDeleter());
    if (!m_stream)
    {
        std::stringstream out;
        out << "writers.text couldn't open '" << m_filename <<
            "' for output.";
        throw pdal_error(out.str());
    }
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
    m_precision = ops.getValueOrDefault<int>("precision", 3);
}


void TextWriter::ready(PointTableRef table)
{
    m_stream->precision(m_precision);
    *m_stream << std::fixed;

    typedef boost::tokenizer<boost::char_separator<char>> tokenizer;

    // Find the dimensions listed and put them on the id list.
    boost::char_separator<char> separator(",");
    boost::erase_all(m_dimOrder, " "); // Wipe off spaces
    tokenizer sdims(m_dimOrder, separator);
    for (tokenizer::iterator ti = sdims.begin(); ti != sdims.end(); ++ti)
    {
        Dimension::Id::Enum d = table.layout()->findDim(*ti);
        if (d == Dimension::Id::Unknown)
        {
            std::ostringstream oss;
            oss << "Dimension not found with name '" << *ti <<"'";
            throw pdal_error(oss.str());
        }
        m_dims.push_back(d);
    }
    // Add the rest of the dimensions to the list if we're doing that.
    // Yes, this isn't efficient when, but it's simple.
    if (m_dimOrder.empty() || m_writeAllDims)
    {
        Dimension::IdList all = table.layout()->dims();
        for (auto di = all.begin(); di != all.end(); ++di)
            if (!contains(m_dims, *di))
                m_dims.push_back(*di);
    }

    if (!m_writeHeader)
        log()->get(LogLevel::Debug) << "Not writing header" << std::endl;
    else
        writeHeader(table);
}


void TextWriter::writeHeader(PointTableRef table)
{
    log()->get(LogLevel::Debug) << "Writing header to filename: " <<
        m_filename << std::endl;
    if (m_outputType == "GEOJSON")
        writeGeoJSONHeader();
    else if (m_outputType == "CSV")
        writeCSVHeader(table);
}


void TextWriter::writeFooter()
{
    if (m_outputType == "GEOJSON")
    {
        *m_stream << "]}";
        if (m_callback.size())
            *m_stream  <<")";
    }
    m_stream.reset();
}


void TextWriter::writeGeoJSONHeader()
{
    if (m_callback.size())
        *m_stream << m_callback <<"(";
    *m_stream << "{ \"type\": \"FeatureCollection\", \"features\": [";
}


void TextWriter::writeCSVHeader(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        if (di != m_dims.begin())
            *m_stream << m_delimiter;

        if (m_quoteHeader)
            *m_stream << "\"" << layout->dimName(*di) << "\"";
        else
            *m_stream << layout->dimName(*di);
    }
    *m_stream << m_newline;
}

void TextWriter::writeCSVBuffer(const PointViewPtr view)
{
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
        {
            if (di != m_dims.begin())
                *m_stream << m_delimiter;
            *m_stream << view->getFieldAs<double>(*di, idx);
        }
        *m_stream << m_newline;
    }
}

void TextWriter::writeGeoJSONBuffer(const PointViewPtr view)
{
    using namespace Dimension;

    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        if (idx)
            *m_stream << ",";

        *m_stream << "{ \"type\":\"Feature\",\"geometry\": "
            "{ \"type\": \"Point\", \"coordinates\": [";
        *m_stream << view->getFieldAs<double>(Id::X, idx) << ",";
        *m_stream << view->getFieldAs<double>(Id::Y, idx) << ",";
        *m_stream << view->getFieldAs<double>(Id::Z, idx) << "]},";

        *m_stream << "\"properties\": {";

        for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
        {
            if (di != m_dims.begin())
                *m_stream << ",";

            *m_stream << "\"" << view->dimName(*di) << "\":";
            *m_stream << "\"";
            *m_stream << view->getFieldAs<double>(*di, idx);
            *m_stream <<"\"";
        }
        *m_stream << "}"; // end properties
        *m_stream << "}"; // end feature
    }
}

void TextWriter::write(const PointViewPtr view)
{
    if (m_outputType == "CSV")
        writeCSVBuffer(view);
    else if (m_outputType == "GEOJSON")
        writeGeoJSONBuffer(view);
}


void TextWriter::done(PointTableRef /*table*/)
{
    writeFooter();
}

} // namespace pdal
