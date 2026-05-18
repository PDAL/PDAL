/******************************************************************************
 * Copyright (c) 2019, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "PcdWriter.hpp"
#include "PcdHeader.hpp"

#include <pdal/PDALUtils.hpp>
#include <pdal/util/OStream.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

using namespace Dimension;

static StaticPluginInfo const s_info
{
    "writers.pcd",
    "Write data in the Point Cloud Library (PCL) format.",
    "https://pdal.org/stages/writers.pcd.html",
    {"pcd"}
};

CREATE_STATIC_STAGE(PcdWriter, s_info)

std::string PcdWriter::getName() const
{
    return s_info.name;
}

PcdWriter::PcdWriter()
{}

void PcdWriter::addArgs(ProgramArgs& args)
{
    args.add("compression", "Level of PCD compression to use (ascii, binary, compressed)",
        m_compression_string, "ascii");
    args.add("keep_unspecified", "Write all dimensions", m_writeAllDims, true);
    args.add("order", "Dimension order", m_dimOrder);
    args.add("precision", "ASCII precision", m_precision, static_cast<uint32_t>(2));
}


PcdWriter::DimSpec PcdWriter::extractDim(std::string dim, PointTableRef table)
{
    Utils::trim(dim);

    uint32_t precision(m_precision);
    PcdField field;
    field.m_count = 1;
    StringList s = Utils::split(dim, '=');
    if (s.size() == 1)
    {
        precision = m_precision;
        Id id = table.layout()->findDim(s[0]);
        if (id == Id::X || id == Id::Y || id == Id::Z)
            field.m_size = 4;
        else
            field.m_size = 8;
        field.m_type = PcdFieldType::F;
    }
    else if (s.size() == 2)
    {
        try
        {
            StringList t = Utils::split(s[1], ':');

            if (t[0] == "Unsigned8")
            {
                field.m_type = PcdFieldType::U;
                field.m_size = 1;
            }
            else if (t[0] == "Unsigned16")
            {
                field.m_type = PcdFieldType::U;
                field.m_size = 2;
            }
            else if (t[0] == "Unsigned32")
            {
                field.m_type = PcdFieldType::U;
                field.m_size = 4;
            }
            else if (t[0] == "Unsigned64")
            {
                field.m_type = PcdFieldType::U;
                field.m_size = 8;
            }
            else if (t[0] == "Signed8")
            {
                field.m_type = PcdFieldType::I;
                field.m_size = 1;
            }
            else if (t[0] == "Signed16")
            {
                field.m_type = PcdFieldType::I;
                field.m_size = 2;
            }
            else if (t[0] == "Signed32")
            {
                field.m_type = PcdFieldType::I;
                field.m_size = 4;
            }
            else if (t[0] == "Signed64")
            {
                field.m_type = PcdFieldType::I;
                field.m_size = 8;
            }
            else if (t[0] == "Float")
            {
                field.m_type = PcdFieldType::F;
                field.m_size = 4;
            }
            else if (t[0] == "Double")
            {
                field.m_type = PcdFieldType::F;
                field.m_size = 8;
            }
            else
            {
                field.m_type = PcdFieldType::unknown;
            }

            if (t.size() == 2)
            {
                size_t pos;
                int i = std::stoi(t[1], &pos);
                if (i < 0 || pos != t[1].size())
                    throw pdal_error("Dummy"); // Throw to be caught below.
                precision = static_cast<uint32_t>(i);
            }
        }
        catch (...)
        {
            throwError("Can't convert dimension precision for '" + dim + "'.");
        }
    }
    else
        throwError("Invalid dimension specification '" + dim + "'.");
    Id d = table.layout()->findDim(s[0]);
    if (d == Id::Unknown)
        throwError("Dimension not found with name '" + dim + "'.");

    field.m_label = table.layout()->dimName(d);
    field.m_id = d;

    return DimSpec{field, precision};
}

bool PcdWriter::findDim(Id id, DimSpec& ds)
{
    auto it =
        std::find_if(m_dims.begin(), m_dims.end(), [id](const DimSpec& tds) {
            return tds.m_field.m_id == id;
        });
    if (it == m_dims.end())
        return false;
    ds = *it;
    return true;
}

void PcdWriter::ready(PointTableRef table)
{
    PcdField field;
    field.m_label = table.layout()->dimName(Id::X);
    field.m_id = Id::X;
    field.m_size = 4;
    field.m_type = PcdFieldType::F;
    field.m_count = 1;
    m_xDim = DimSpec{field, m_precision};
    field.m_label = table.layout()->dimName(Id::Y);
    field.m_id = Id::Y;
    m_yDim = {field, m_precision};
    field.m_label = table.layout()->dimName(Id::Z);
    field.m_id = Id::Z;
    m_zDim = {field, m_precision};

    // Find the dimensions listed and put them on the id list.
    StringList dimNames = Utils::split2(m_dimOrder, ',');
    for (std::string dim : dimNames)
    {
        const DimSpec& spec = extractDim(dim, table);
        if (spec.m_field.m_id == Id::X)
            m_xDim = spec;
        else if (spec.m_field.m_id == Id::Y)
            m_yDim = spec;
        else if (spec.m_field.m_id == Id::Z)
            m_zDim = spec;
        m_dims.push_back(spec);
    }

    if (m_dimOrder.empty() || m_writeAllDims)
    {
        IdList all = table.layout()->dims();
        for (auto id : all)
        {
            PcdField field;
            field.m_label = table.layout()->dimName(id);
            field.m_id = id;
            if (id == Id::X || id == Id::Y || id == Id::Z)
                field.m_size = 4;
            else
                field.m_size = 8;
            field.m_type = PcdFieldType::F;
            field.m_count = 1;
            DimSpec ds{field, m_precision};
            if (!findDim(id, ds))
                m_dims.push_back(ds);
        }
    }
}

void PcdWriter::write(const PointViewPtr view)
{
    std::unique_ptr<std::ostream> out(Utils::createFile(filename(), false));
    if (!out)
        throwError("Couldn't open '" + filename() + "' for output.");

    PcdHeader header;

    header.m_version = PcdVersion::PCD_V7;
    header.m_height = 1;
    if (m_compression_string == "ascii")
        header.m_dataStorage = PcdDataStorage::ASCII;
    else if (m_compression_string == "binary")
        header.m_dataStorage = PcdDataStorage::BINARY;
    else
        throwError("Unrecognized compression string '" + m_compression_string + "'. "
                   "Expected 'ASCII' or 'BINARY'.");

    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
        header.m_fields.push_back(di->m_field);

    header.m_width = view->size();
    header.m_pointCount = view->size();

    *out << header;

    if (m_compression_string == "ascii")
        writeAscii(view, *out);
    else
    {
        // Reopen as for binary output, seeking to the end of the header before writing data.
        out.reset(FileUtils::openExisting(filename(), true));
        out->seekp(0, std::ios::end);
        writeBinary(view, *out);
    }
}

void PcdWriter::writeAscii(const PointViewPtr view, std::ostream& out)
{
    for (PointRef point : *view)
    {
        out << std::fixed;
        for (DimSpec& dim : m_dims)
        {
            if (dim.m_field.m_type == PcdFieldType::F && dim.m_field.m_size == 8)
                out << std::setprecision(dim.m_precision)
                     << point.getFieldAs<double>(dim.m_field.m_id)
                     << " ";
            else if (dim.m_field.m_type == PcdFieldType::F && dim.m_field.m_size == 4)
                out << std::setprecision(dim.m_precision)
                    << point.getFieldAs<float>(dim.m_field.m_id)
                    << " ";
            else if (dim.m_field.m_type == PcdFieldType::U && dim.m_field.m_size == 8)
                out << std::setprecision(dim.m_precision)
                    << point.getFieldAs<uint64_t>(dim.m_field.m_id)
                    << " ";
            else if (dim.m_field.m_type == PcdFieldType::U && dim.m_field.m_size == 4)
                out << std::setprecision(dim.m_precision)
                    << point.getFieldAs<uint32_t>(dim.m_field.m_id)
                    << " ";
            else if (dim.m_field.m_type == PcdFieldType::U && dim.m_field.m_size == 2)
                out << std::setprecision(dim.m_precision)
                    << point.getFieldAs<uint16_t>(dim.m_field.m_id)
                    << " ";
            else if (dim.m_field.m_type == PcdFieldType::I && dim.m_field.m_size == 1)
                out << std::setprecision(dim.m_precision)
                    << point.getFieldAs<uint8_t>(dim.m_field.m_id)
                    << " ";
            else if (dim.m_field.m_type == PcdFieldType::I && dim.m_field.m_size == 8)
                out << std::setprecision(dim.m_precision)
                    << point.getFieldAs<int64_t>(dim.m_field.m_id)
                    << " ";
            else if (dim.m_field.m_type == PcdFieldType::I && dim.m_field.m_size == 4)
                out << std::setprecision(dim.m_precision)
                    << point.getFieldAs<int32_t>(dim.m_field.m_id)
                    << " ";
            else if (dim.m_field.m_type == PcdFieldType::I && dim.m_field.m_size == 2)
                out << std::setprecision(dim.m_precision)
                    << point.getFieldAs<int16_t>(dim.m_field.m_id)
                    << " ";
            else if (dim.m_field.m_type == PcdFieldType::I && dim.m_field.m_size == 1)
                out << std::setprecision(dim.m_precision)
                    << point.getFieldAs<int8_t>(dim.m_field.m_id)
                    << " ";
        }
        out << "\n";
    }
}

void PcdWriter::writeBinary(const PointViewPtr view, std::ostream& out)
{
    // Write little-endian binary.
    OLeStream leOut(&out);
    for (PointRef point : *view)
    {
        for (DimSpec& dim : m_dims)
        {
            if (dim.m_field.m_type == PcdFieldType::F && dim.m_field.m_size == 8)
                leOut << point.getFieldAs<double>(dim.m_field.m_id);
            else if (dim.m_field.m_type == PcdFieldType::F && dim.m_field.m_size == 4)
                leOut << point.getFieldAs<float>(dim.m_field.m_id);
            else if (dim.m_field.m_type == PcdFieldType::U && dim.m_field.m_size == 8)
                leOut << point.getFieldAs<uint64_t>(dim.m_field.m_id);
            else if (dim.m_field.m_type == PcdFieldType::U && dim.m_field.m_size == 4)
                leOut << point.getFieldAs<uint32_t>(dim.m_field.m_id);
            else if (dim.m_field.m_type == PcdFieldType::U && dim.m_field.m_size == 2)
                leOut << point.getFieldAs<uint16_t>(dim.m_field.m_id);
            else if (dim.m_field.m_type == PcdFieldType::I && dim.m_field.m_size == 1)
                leOut << point.getFieldAs<uint8_t>(dim.m_field.m_id);
            else if (dim.m_field.m_type == PcdFieldType::I && dim.m_field.m_size == 8)
                leOut << point.getFieldAs<int64_t>(dim.m_field.m_id);
            else if (dim.m_field.m_type == PcdFieldType::I && dim.m_field.m_size == 4)
                leOut << point.getFieldAs<int32_t>(dim.m_field.m_id);
            else if (dim.m_field.m_type == PcdFieldType::I && dim.m_field.m_size == 2)
                leOut << point.getFieldAs<int16_t>(dim.m_field.m_id);
            else if (dim.m_field.m_type == PcdFieldType::I && dim.m_field.m_size == 1)
                leOut << point.getFieldAs<int8_t>(dim.m_field.m_id);
        }
    }
}

void PcdWriter::done(PointTableRef table)
{
    getMetadata().addList("filename", filename());
}

} // namespaces
