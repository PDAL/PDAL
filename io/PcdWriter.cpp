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
    "http://pdal.io/stages/writers.pcd.html",
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
    args.add("filename", "PCD output filename", m_filename).setPositional();
    args.add("compression",
             "Level of PCD compression to use "
             "(ascii, binary, compressed)",
             m_compression_string, "ascii");
    args.add("keep_unspecified", "Write all dimensions", m_writeAllDims, true);
    args.add("order", "Dimension order", m_dimOrder);
    args.add("precision", "ASCII precision", m_precision,
             static_cast<uint32_t>(2));
}

void PcdWriter::initialize(PointTableRef table)
{
    m_ostream = Utils::createFile(m_filename, false);
    if (!m_ostream)
        throwError("Couldn't open '" + m_filename + "' for output.");
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
    m_header.m_version = PcdVersion::PCD_V7;
    m_header.m_height = 1;
    if (m_compression_string == "ascii")
        m_header.m_dataStorage = PcdDataStorage::ASCII;
    else if (m_compression_string == "binary")
        m_header.m_dataStorage = PcdDataStorage::BINARY;
    else if (m_compression_string == "compressed")
        m_header.m_dataStorage = PcdDataStorage::COMPRESSED;
    else
        throwError("Unrecognized compression string");

    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        m_header.m_fields.push_back(di->m_field);
    }

    m_header.m_width = view->size();
    m_header.m_pointCount = view->size();

    *m_ostream << m_header;

    PointRef point(*view, 0);

    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        point.setPointId(idx);
        if (m_compression_string == "ascii")
        {
            *m_ostream << std::fixed;
            for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
            {
                if (di->m_field.m_type == PcdFieldType::F &&
                    di->m_field.m_size == 8)
                    *m_ostream << std::setprecision(di->m_precision)
                               << point.getFieldAs<double>(di->m_field.m_id)
                               << " ";
                else if (di->m_field.m_type == PcdFieldType::F &&
                         di->m_field.m_size == 4)
                    *m_ostream << std::setprecision(di->m_precision)
                               << point.getFieldAs<float>(di->m_field.m_id)
                               << " ";
                else if (di->m_field.m_type == PcdFieldType::U &&
                         di->m_field.m_size == 8)
                    *m_ostream << std::setprecision(di->m_precision)
                               << point.getFieldAs<uint64_t>(di->m_field.m_id)
                               << " ";
                else if (di->m_field.m_type == PcdFieldType::U &&
                         di->m_field.m_size == 4)
                    *m_ostream << std::setprecision(di->m_precision)
                               << point.getFieldAs<uint32_t>(di->m_field.m_id)
                               << " ";
                else if (di->m_field.m_type == PcdFieldType::U &&
                         di->m_field.m_size == 2)
                    *m_ostream << std::setprecision(di->m_precision)
                               << point.getFieldAs<uint16_t>(di->m_field.m_id)
                               << " ";
                else if (di->m_field.m_type == PcdFieldType::I &&
                         di->m_field.m_size == 1)
                    *m_ostream << std::setprecision(di->m_precision)
                               << point.getFieldAs<uint8_t>(di->m_field.m_id)
                               << " ";
                else if (di->m_field.m_type == PcdFieldType::I &&
                         di->m_field.m_size == 8)
                    *m_ostream << std::setprecision(di->m_precision)
                               << point.getFieldAs<int64_t>(di->m_field.m_id)
                               << " ";
                else if (di->m_field.m_type == PcdFieldType::I &&
                         di->m_field.m_size == 4)
                    *m_ostream << std::setprecision(di->m_precision)
                               << point.getFieldAs<int32_t>(di->m_field.m_id)
                               << " ";
                else if (di->m_field.m_type == PcdFieldType::I &&
                         di->m_field.m_size == 2)
                    *m_ostream << std::setprecision(di->m_precision)
                               << point.getFieldAs<int16_t>(di->m_field.m_id)
                               << " ";
                else if (di->m_field.m_type == PcdFieldType::I &&
                         di->m_field.m_size == 1)
                    *m_ostream << std::setprecision(di->m_precision)
                               << point.getFieldAs<int8_t>(di->m_field.m_id)
                               << " ";
            }
            *m_ostream << "\n";
        }
        else if (m_compression_string == "binary")
        {
            OLeStream out(m_ostream);
            for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
            {
                if (di->m_field.m_type == PcdFieldType::F &&
                    di->m_field.m_size == 8)
                    out << point.getFieldAs<double>(di->m_field.m_id);
                else if (di->m_field.m_type == PcdFieldType::F &&
                         di->m_field.m_size == 4)
                    out << point.getFieldAs<float>(di->m_field.m_id);
                else if (di->m_field.m_type == PcdFieldType::U &&
                         di->m_field.m_size == 8)
                    out << point.getFieldAs<uint64_t>(di->m_field.m_id);
                else if (di->m_field.m_type == PcdFieldType::U &&
                         di->m_field.m_size == 4)
                    out << point.getFieldAs<uint32_t>(di->m_field.m_id);
                else if (di->m_field.m_type == PcdFieldType::U &&
                         di->m_field.m_size == 2)
                    out << point.getFieldAs<uint16_t>(di->m_field.m_id);
                else if (di->m_field.m_type == PcdFieldType::I &&
                         di->m_field.m_size == 1)
                    out << point.getFieldAs<uint8_t>(di->m_field.m_id);
                else if (di->m_field.m_type == PcdFieldType::I &&
                         di->m_field.m_size == 8)
                    out << point.getFieldAs<int64_t>(di->m_field.m_id);
                else if (di->m_field.m_type == PcdFieldType::I &&
                         di->m_field.m_size == 4)
                    out << point.getFieldAs<int32_t>(di->m_field.m_id);
                else if (di->m_field.m_type == PcdFieldType::I &&
                         di->m_field.m_size == 2)
                    out << point.getFieldAs<int16_t>(di->m_field.m_id);
                else if (di->m_field.m_type == PcdFieldType::I &&
                         di->m_field.m_size == 1)
                    out << point.getFieldAs<int8_t>(di->m_field.m_id);
            }
        }
    }
}

void PcdWriter::done(PointTableRef table)
{
    Utils::closeFile(m_ostream);
    m_ostream = nullptr;
    getMetadata().addList("filename", m_filename);
}

} // namespaces
