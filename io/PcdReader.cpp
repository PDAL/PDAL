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

#include <pdal/PDALUtils.hpp>
#include <pdal/util/Algorithm.hpp>

#include "PcdHeader.hpp"
#include "PcdReader.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "readers.pcd",
    "Read data in the Point Cloud Library (PCL) format.",
    "http://pdal.io/stages/readers.pcd.html",
    {"pcd"}
};

CREATE_STATIC_STAGE(PcdReader, s_info)

std::string PcdReader::getName() const
{
    return s_info.name;
}

PcdReader::PcdReader()
{}

QuickInfo PcdReader::inspect()
{
    QuickInfo qi;

    initialize();

    for (auto i : m_header.m_fields)
        qi.m_dimNames.push_back(i.m_label);
    qi.m_pointCount = m_header.m_pointCount;
    qi.m_valid = true;

    return qi;
}

void PcdReader::ready(PointTableRef table)
{
    m_index = 0;
    switch (m_header.m_dataStorage)
    {
    case PcdDataStorage::ASCII:
        m_istreamPtr = Utils::openFile(m_filename, true);
        if (!m_istreamPtr)
            throwError("Unable to open ASCII PCD file '" + m_filename + "'.");
        m_istreamPtr->seekg(m_header.m_dataOffset);
        break;
    case PcdDataStorage::BINARY:
        m_istreamPtr = Utils::openFile(m_filename, true);
        if (!m_istreamPtr)
            throwError("Unable to open binary PCD file '" + m_filename + "'.");
        m_stream = ILeStream(m_istreamPtr);
        m_stream.seek(m_header.m_dataOffset);
        break;
    case PcdDataStorage::COMPRESSED:
        throwError("Binary compressed PCD is not supported at this time.");
        break;
    case PcdDataStorage::unknown:
    default:
        throwError("Unrecognized data storage.");
    }
}

void PcdReader::addDimensions(PointLayoutPtr layout)
{
    m_dims.clear();
    for (auto i : m_header.m_fields)
    {
        Dimension::BaseType base = Dimension::BaseType::None;
        if (i.m_type == PcdFieldType::U)
            base = Dimension::BaseType::Unsigned;
        else if (i.m_type == PcdFieldType::I)
            base = Dimension::BaseType::Signed;
        else if (i.m_type == PcdFieldType::F)
            base = Dimension::BaseType::Floating;
        Dimension::Type t =
            static_cast<Dimension::Type>(unsigned(base) | i.m_size);
        Utils::trim(i.m_label);
        i.m_label = Utils::toupper(i.m_label);
        if (i.m_label == "X" || i.m_label == "Y" || i.m_label == "Z")
            t = Dimension::Type::Double;
        Dimension::Id id = layout->registerOrAssignDim(i.m_label, t);
        if (Utils::contains(m_dims, id) && id != pdal::Dimension::Id::Unknown)
            throwError("Duplicate dimension '" + i.m_label +
                       "' detected in input file '" + m_filename + "'.");
        m_dims.push_back(id);
    }
}

bool PcdReader::fillFields()
{
    while (true)
    {
        if (!m_istreamPtr->good())
            return false;

        std::string buf;

        std::getline(*m_istreamPtr, buf);
        m_line++;
        if (buf.empty())
            continue;

        Utils::trim(buf);
        m_fields = Utils::split(buf, [](char c) { return c == '\t' || c == '\r' || c == ' '; });
        if (m_fields.size() != m_dims.size())
        {
            log()->get(LogLevel::Error)
                << "Line " << m_line << " in '" << m_filename << "' contains "
                << m_fields.size() << " fields when " << m_dims.size()
                << " were expected.  "
                   "Ignoring."
                << std::endl;
            continue;
        }
        return true;
    }
}

bool PcdReader::processOne(PointRef& point)
{
    switch (m_header.m_dataStorage)
    {
    case PcdDataStorage::ASCII:
        if (!fillFields())
            return false;

        double d;
        for (size_t i = 0; i < m_fields.size(); ++i)
        {
            if (!Utils::fromString(m_fields[i], d))
            {
                log()->get(LogLevel::Error)
                    << "Can't convert field '" << m_fields[i]
                    << "' to numeric value on line " << m_line << " in '"
                    << m_filename << "'.  Setting to 0." << std::endl;
                d = 0;
            }
            point.setField(m_dims[i], d);
        }
        return true;
    case PcdDataStorage::BINARY:
        if (!m_stream.good())
            return false;

        if ((m_index >= m_count) ||
            (m_index >= (point_count_t)m_header.m_pointCount))
            return false;

        for (auto const& i : m_header.m_fields)
        {
            switch (i.m_type)
            {
            case PcdFieldType::I:
                int32_t ival;
                m_stream >> ival;
                point.setField(i.m_id, ival);
                break;
            case PcdFieldType::F:
                float fval;
                m_stream >> fval;
                point.setField(i.m_id, fval);
                break;
            case PcdFieldType::U:
                uint32_t uval;
                m_stream >> uval;
                point.setField(i.m_id, uval);
                break;
            case PcdFieldType::unknown:
            default:
                throwError("Unsupported field type.");
            }
        }
        m_index++;
        return true;
    case PcdDataStorage::COMPRESSED:
        throwError("Binary compressed PCD is not supported at this time.");
        return false;
    case PcdDataStorage::unknown:
    default:
        throwError("Unrecognized data storage.");
        return false;
    }
}

point_count_t PcdReader::read(PointViewPtr view, point_count_t count)
{
    PointId idx = view->size();
    point_count_t cnt = 0;
    PointRef point(*view, idx);
    while (cnt < count)
    {
        point.setPointId(idx);
        if (!processOne(point))
            break;
        cnt++;
        idx++;
    }
    return cnt;
}

void PcdReader::initialize()
{
    if (m_filename.empty())
        throwError("Can't read PCD file without filename.");

    m_istreamPtr = Utils::openFile(m_filename, true);
    if (!m_istreamPtr)
        throwError("Can't open file '" + m_filename + "'.");
    try
    {
        m_header.clear();
        *m_istreamPtr >> m_header;
    }
    catch( ... )
    {
        Utils::closeFile(m_istreamPtr);
        m_istreamPtr = nullptr;
        throw;
    }
    m_line = m_header.m_numLines;

    Utils::closeFile(m_istreamPtr);
}

void PcdReader::done(PointTableRef table)
{
    m_stream.close();
    Utils::closeFile(m_istreamPtr);
}

} // namespace pdal
