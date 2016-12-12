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
#include <pdal/PDALUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/Algorithm.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <iostream>

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
            Utils::closeFile(ptr);
        }
    }
};


void TextWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename);
    args.add("format", "Output format", m_outputType, "csv");
    args.add("jscallback", "", m_callback);
    args.add("keep_unspecified", "Write all dimensions", m_writeAllDims, true);
    args.add("order", "Dimension order", m_dimOrder);
    args.add("write_header", "Whether a header should be written",
        m_writeHeader, true);
    args.add("newline", "String to use as newline", m_newline, "\n");
    args.add("delimiter", "Dimension delimiter", m_delimiter, ",");
    args.add("quote_header", "Whether a header should be quoted",
        m_quoteHeader, true);
    args.add("precision", "Output precision", m_precision, 3);
}


void TextWriter::initialize(PointTableRef table)
{
    m_stream = FileStreamPtr(Utils::createFile(m_filename, true),
        FileStreamDeleter());
    if (!m_stream)
    {
        std::stringstream out;
        out << "writers.text couldn't open '" << m_filename <<
            "' for output.";
        throw pdal_error(out.str());
    }
    m_outputType = Utils::toupper(m_outputType);
}


void TextWriter::ready(PointTableRef table)
{
    m_stream->precision(m_precision);
    *m_stream << std::fixed;

    // Find the dimensions listed and put them on the id list.
    StringList dimNames = Utils::split2(m_dimOrder, ',');
    for (std::string dim : dimNames)
    {
        Utils::trim(dim);
        Dimension::Id d = table.layout()->findDim(dim);
        if (d == Dimension::Id::Unknown)
        {
            std::ostringstream oss;
            oss << getName() << ": Dimension not found with name '" <<
                dim << "'.";
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
            if (!Utils::contains(m_dims, *di))
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
    getMetadata().addList("filename", m_filename);
}

} // namespace pdal
