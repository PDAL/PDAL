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
#include <pdal/util/Algorithm.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <iostream>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "writers.text",
    "Text Writer",
    "http://pdal.io/stages/writers.text.html",
    { "csv", "txt", "json", "xyz", "" }
};

CREATE_STATIC_STAGE(TextWriter, s_info)

std::string TextWriter::getName() const { return s_info.name; }

std::istream& operator >> (std::istream& in, TextWriter::OutputType& type)
{
    std::string s;
    in >> s;
    s = Utils::toupper(s);
    if (s == "CSV")
        type = TextWriter::OutputType::CSV;
    else if (s == "GEOJSON")
        type = TextWriter::OutputType::GEOJSON;
    else
        in.setstate(std::ios_base::failbit);
    return in;
}

std::ostream& operator << (std::ostream& out,
    const TextWriter::OutputType& type)
{
    if (type == TextWriter::OutputType::CSV)
        out << "CSV";
    else if (type == TextWriter::OutputType::GEOJSON)
        out << "GEOJSON";
    return out;
}

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
    args.add("format", "Output format", m_outputType, OutputType::CSV);
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
        throwError("Couldn't open '" + m_filename + "' for output.");
}


TextWriter::DimSpec TextWriter::extractDim(std::string dim, PointTableRef table)
{
    Utils::trim(dim);

    size_t precision(0);
    StringList s = Utils::split(dim, ':');
    if (s.size() == 1)
        precision = m_precision;
    else if (s.size() == 2)
    {
        try
        {
            size_t pos;
            int i = std::stoi(s[1], &pos);
            if (i < 0 || pos != s[1].size())
                throw pdal_error("Dummy");  // Throw to be caught below.
            precision = static_cast<size_t>(i);
        }
        catch (...)
        {
            throwError("Can't convert dimension precision for '" + dim +
                "'.");
        }
    }
    else
        throwError("Invalid dimension specification '" + dim + "'.");
    Dimension::Id d = table.layout()->findDim(s[0]);
    if (d == Dimension::Id::Unknown)
        throwError("Dimension not found with name '" + dim + "'.");
    return { d, precision, table.layout()->dimName(d) };
}


bool TextWriter::findDim(Dimension::Id id, DimSpec& ds)
{
    auto it = std::find_if(m_dims.begin(), m_dims.end(),
        [id](const DimSpec& tds){ return tds.id == id; });
    if (it == m_dims.end())
        return false;
    ds = *it;
    return true;
}


void TextWriter::ready(PointTableRef table)
{
    *m_stream << std::fixed;

    m_xDim = { Dimension::Id::X, static_cast<size_t>(m_precision),
        table.layout()->dimName(Dimension::Id::X) };
    m_yDim = { Dimension::Id::Y, static_cast<size_t>(m_precision),
        table.layout()->dimName(Dimension::Id::Y) };
    m_zDim = { Dimension::Id::Z, static_cast<size_t>(m_precision),
        table.layout()->dimName(Dimension::Id::Z) };

    // Find the dimensions listed and put them on the id list.
    StringList dimNames = Utils::split2(m_dimOrder, ',');
    for (std::string dim : dimNames)
    {
        const DimSpec& spec = extractDim(dim, table);
        if (spec.id == Dimension::Id::X)
            m_xDim = spec;
        else if (spec.id == Dimension::Id::Y)
            m_yDim = spec;
        else if (spec.id == Dimension::Id::Z)
            m_zDim = spec;
        m_dims.push_back(spec);
    }

    // Add the rest of the dimensions to the list if we're doing that.
    // Yes, this isn't efficient when, but it's simple.
    if (m_dimOrder.empty() || m_writeAllDims)
    {
        Dimension::IdList all = table.layout()->dims();
        for (auto id : all)
        {
            DimSpec ds { id, static_cast<size_t>(m_precision),
                table.layout()->dimName(id) };
            if (!findDim(id, ds))
                m_dims.push_back(ds);
        }
    }

    if (!m_writeHeader)
        log()->get(LogLevel::Debug) << "Not writing header" << std::endl;
    else
        writeHeader(table);
    m_idx = 0;
}


void TextWriter::writeHeader(PointTableRef table)
{
    log()->get(LogLevel::Debug) << "Writing header to filename: " <<
        m_filename << std::endl;
    if (m_outputType == OutputType::GEOJSON)
        writeGeoJSONHeader();
    else if (m_outputType == OutputType::CSV)
        writeCSVHeader(table);
}


void TextWriter::writeFooter()
{
    if (m_outputType == OutputType::GEOJSON)
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
            *m_stream << "\"" << layout->dimName(di->id) << "\"";
        else
            *m_stream << layout->dimName(di->id);
    }
    *m_stream << m_newline;
}


void TextWriter::processOneCSV(PointRef& point)
{
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        if (di != m_dims.begin())
            *m_stream << m_delimiter;
        m_stream->precision(di->precision);
        *m_stream << point.getFieldAs<double>(di->id);
    }
    *m_stream << m_newline;
}

void TextWriter::processOneGeoJSON(PointRef& point)
{
    if (m_idx > 0)
        *m_stream << ",";
    *m_stream << "{ \"type\":\"Feature\",\"geometry\": "
        "{ \"type\": \"Point\", \"coordinates\": [";

    m_stream->precision(m_xDim.precision);
    *m_stream << point.getFieldAs<double>(Dimension::Id::X) << ",";
    m_stream->precision(m_yDim.precision);
    *m_stream << point.getFieldAs<double>(Dimension::Id::Y) << ",";
    m_stream->precision(m_zDim.precision);
    *m_stream << point.getFieldAs<double>(Dimension::Id::Z) << "]},";

    *m_stream << "\"properties\": {";

    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        if (di != m_dims.begin())
            *m_stream << ",";

        *m_stream << "\"" << di->name << "\":";
        *m_stream << "\"";
        m_stream->precision(di->precision);
        *m_stream << point.getFieldAs<double>(di->id);
        *m_stream <<"\"";
    }
    *m_stream << "}"; // end properties
    *m_stream << "}"; // end feature
}


bool TextWriter::processOne(PointRef& point)
{
    if (m_outputType == OutputType::CSV)
        processOneCSV(point);
    else
        processOneGeoJSON(point);
    m_idx++;
    return true;
}


void TextWriter::write(const PointViewPtr view)
{
    PointRef point(*view, 0);

    if (m_outputType == OutputType::CSV)
        for (PointId idx = 0; idx < view->size(); ++idx)
        {
            point.setPointId(idx);
            processOneCSV(point);
        }
    else if (m_outputType == OutputType::GEOJSON)
        for (PointId idx = 0; idx < view->size(); ++idx)
        {
            point.setPointId(idx);
            processOneGeoJSON(point);
        }
}


void TextWriter::done(PointTableRef /*table*/)
{
    writeFooter();
    getMetadata().addList("filename", m_filename);
}

} // namespace pdal
