/******************************************************************************
 * Copyright (c) 2019, Kirk McKelvey (kirkoman@gmail.com)
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

#include "PcdHeader.hpp"

namespace pdal
{

std::istream& operator>>(std::istream& in, PcdVersion& version)
{
    float f;
    in >> f;
    if (Utils::compare_approx(f, 0.6, 0.01))
        version = PcdVersion::PCD_V6;
    else if (Utils::compare_approx(f, 0.7, 0.01))
        version = PcdVersion::PCD_V7;
    else
    {
        version = PcdVersion::unknown;
        throw pdal_error("failed parsing PCD file version");
    }
    return in;
}

std::ostream& operator<<(std::ostream& out, PcdVersion& version)
{
    switch (version)
    {
    case PcdVersion::PCD_V6:
        out << std::fixed << std::setprecision(1) << 0.6f;
        break;
    case PcdVersion::PCD_V7:
        out << std::fixed << std::setprecision(1) << 0.7f;
        break;
    case PcdVersion::unknown:
    default:
        throw pdal_error("failed writing invalid PCD file version");
    }
    return out;
}

std::istream& operator>>(std::istream& in, PcdFieldType& type)
{
    std::string s;
    in >> s;
    s = Utils::toupper(s);
    if (s == "I")
        type = PcdFieldType::I;
    else if (s == "U")
        type = PcdFieldType::U;
    else if (s == "F")
        type = PcdFieldType::F;
    else
    {
        type = PcdFieldType::unknown;
        throw pdal_error("failed parsing PCD field type (\"" + s + "\")");
    }
    return in;
}

std::ostream& operator<<(std::ostream& out, PcdFieldType& type)
{
    switch (type)
    {
    case PcdFieldType::I:
        out << "I";
        break;
    case PcdFieldType::U:
        out << "U";
        break;
    case PcdFieldType::F:
        out << "F";
        break;
    case PcdFieldType::unknown:
    default:
        throw pdal_error("failed writing PCD field type");
    }
    return out;
}

std::istream& operator>>(std::istream& in, PcdDataStorage& compression)
{
    std::string s;
    in >> s;
    s = Utils::toupper(s);
    if (s == "ASCII")
        compression = PcdDataStorage::ASCII;
    else if (s == "BINARY")
        compression = PcdDataStorage::BINARY;
    else if (s == "BINARY_COMPRESSED")
        compression = PcdDataStorage::COMPRESSED;
    else
    {
        compression = PcdDataStorage::unknown;
        throw pdal_error("failed parsing PCD data storage scheme (\"" + s +
                         "\")");
    }
    return in;
}

std::ostream& operator<<(std::ostream& out, PcdDataStorage& compression)
{
    switch (compression)
    {
    case PcdDataStorage::ASCII:
        out << "ascii";
        break;
    case PcdDataStorage::BINARY:
        out << "binary";
        break;
    case PcdDataStorage::COMPRESSED:
        out << "binary_compressed";
        break;
    case PcdDataStorage::unknown:
    default:
        throw pdal_error("failed writing PCD data storage scheme");
    }
    return out;
}

std::istream& operator>>(std::istream& in, PcdHeader& header)
{
    std::string line;
    size_t lines(0);
    while (!in.eof())
    {
        std::getline(in, line);
        lines++;
        Utils::trim(line);

        if (line.empty() || line.substr(0, 1) == "#")
            continue;
        std::stringstream line_stream(line);
        line_stream.imbue(std::locale::classic());

        std::string line_type;
        line_stream >> line_type;

        if (line_type == "VERSION")
        {
            line_stream >> header.m_version;
            continue;
        }

        else if (line_type == "FIELDS" || line_type == "COLUMNS")
        {
            std::string field_id;
            if (header.m_fields.size() > 0)
                throw pdal_error("FIELDS were specified more than once");
            while (!line_stream.eof())
            {
                line_stream >> field_id;
                header.m_fields.push_back(PcdField(field_id));
            }
            continue;
        }

        else if (line_type == "SIZE")
        {
            auto i = header.m_fields.begin();
            while (i != header.m_fields.end() && !line_stream.eof())
                line_stream >> (i++)->m_size;
            if (i != header.m_fields.end() || !line_stream.eof())
                throw pdal_error(
                    "number of SIZE values does not match number of FIELDS");
            continue;
        }

        else if (line_type == "TYPE")
        {
            auto i = header.m_fields.begin();
            while (i != header.m_fields.end() && !line_stream.eof())
                line_stream >> (i++)->m_type;
            if (i != header.m_fields.end() || !line_stream.eof())
                throw pdal_error(
                    "number of TYPE values does not match number of FIELDS");
            continue;
        }

        else if (line_type == "COUNT")
        {
            auto i = header.m_fields.begin();
            while (i != header.m_fields.end() && !line_stream.eof())
                line_stream >> (i++)->m_count;
            if (i != header.m_fields.end() || !line_stream.eof())
                throw pdal_error(
                    "number of COUNT values does not match number of FIELDS");
            continue;
        }

        else if (line_type == "WIDTH")
        {
            line_stream >> header.m_width;
            continue;
        }

        else if (line_type == "HEIGHT")
        {
            line_stream >> header.m_height;
            continue;
        }

        else if (line_type == "VIEWPOINT")
        {
            if (header.m_version != PcdVersion::PCD_V7)
                throw pdal_error("VIEWPOINT should only be in PCD_V7 files");
            float w, x, y, z;
            line_stream >> x >> y >> z;
            header.m_origin = Eigen::Vector4f(x, y, z, 0.0f);
            line_stream >> w >> x >> y >> z;
            header.m_orientation = Eigen::Quaternionf(w, x, y, z);
            continue;
        }

        else if (line_type == "POINTS")
        {
            line_stream >> header.m_pointCount;
            continue;
        }

        else if (line_type == "DATA")
        {
            line_stream >> header.m_dataStorage;
            header.m_dataOffset = in.tellg();
            header.m_numLines = lines;
            // data starts immediately following so this needs to be last
            break;
        }

        throw pdal_error("unrecognized PCD header, or missing DATA marker");
    }

    return in;
}

std::ostream& operator<<(std::ostream& out, PcdHeader& header)
{
    out << "VERSION " << header.m_version << std::endl;

    out << "FIELDS";
    for (auto i : header.m_fields)
        out << " " << Utils::tolower(i.m_label);
    out << std::endl;

    out << "SIZE";
    for (auto i : header.m_fields)
        out << " " << i.m_size;
    out << std::endl;

    out << "TYPE";
    for (auto i : header.m_fields)
        out << " " << i.m_type;
    out << std::endl;

    out << "COUNT";
    for (auto i : header.m_fields)
        out << " " << i.m_count;
    out << std::endl;

    out << "WIDTH " << header.m_width << std::endl;

    out << "HEIGHT " << header.m_height << std::endl;

    if (header.m_version == PcdVersion::PCD_V7)
    {
        auto& orig = header.m_origin;
        auto& orient = header.m_orientation;
        // TODO: noshowpoint does not seem to work
        out << "VIEWPOINT " << std::fixed << std::noshowpoint;
        out << orig.x() << " " << orig.y() << " " << orig.z() << " ";
        out << orient.w() << " " << orient.x() << " " << orient.y() << " "
            << orient.z() << std::endl;
    }

    out << "POINTS " << header.m_pointCount << std::endl;

    out << "DATA " << header.m_dataStorage << std::endl;

    return out;
}

OLeStream& operator<<(OLeStream& out, PcdHeader& header)
{
    out.put("VERSION ", 8);
    switch (header.m_version)
    {
    case PcdVersion::PCD_V6:
        out.put("0.6", 3);
        break;
    case PcdVersion::PCD_V7:
        out.put("0.7", 3);
        break;
    case PcdVersion::unknown:
    default:
        throw pdal_error("Unrecognized PcdVersion");
    }
    out.put("\n", 1);

    out.put("FIELDS", 6);
    for (auto i : header.m_fields)
    {
        out.put(" ", 1);
        out.put(i.m_label);
    }
    out.put("\n", 1);

    out.put("SIZE", 4);
    for (auto i : header.m_fields)
    {
        out.put(" ", 1);
        std::stringstream ss;
        ss << i.m_size;
        out.put(ss.str());
    }
    out.put("\n", 1);

    out.put("TYPE", 4);
    for (auto i : header.m_fields)
    {
        out.put(" ", 1);
        switch (i.m_type)
        {
        case PcdFieldType::I:
            out.put("I", 1);
            break;
        case PcdFieldType::U:
            out.put("U", 1);
            break;
        case PcdFieldType::F:
            out.put("F", 1);
            break;
        case PcdFieldType::unknown:
        default:
            throw pdal_error("Unrecognized PcdFieldType");
        }
    }
    out.put("\n", 1);

    out.put("COUNT", 5);
    for (auto i : header.m_fields)
    {
        out.put(" ", 1);
        std::stringstream ss;
        ss << i.m_count;
        out.put(ss.str());
    }
    out.put("\n", 1);

    out.put("WIDTH ", 6);
    std::stringstream ss;
    ss << header.m_width;
    out.put(ss.str());
    out.put("\n", 1);

    out.put("HEIGHT ", 7);
    ss.str(std::string());
    ss << header.m_height;
    out.put(ss.str());
    out.put("\n", 1);

    if (header.m_version == PcdVersion::PCD_V7)
    {
        auto& orig = header.m_origin;
        auto& orient = header.m_orientation;
        // TODO: noshowpoint does not seem to work
        out.put("VIEWPOINT ", 10);
        ss.str(std::string());
        ss << std::fixed << std::noshowpoint << orig.x() << " " << orig.y()
           << " " << orig.z() << " " << orient.w() << " " << orient.x() << " "
           << orient.y() << " " << orient.z() << "\n";
        out.put(ss.str());
    }

    out.put("POINTS ", 7);
    ss.str(std::string());
    ss << header.m_pointCount;
    out.put(ss.str());
    out.put("\n", 1);

    out.put("DATA ", 5);
    switch (header.m_dataStorage)
    {
    case PcdDataStorage::ASCII:
        out.put("ascii", 5);
        break;
    case PcdDataStorage::BINARY:
        out.put("binary", 6);
        break;
    case PcdDataStorage::COMPRESSED:
        out.put("compressed", 10);
        break;
    case PcdDataStorage::unknown:
    default:
        throw pdal_error("Unrecognized PcdDataStorage");
    }
    out.put("\n", 1);

    return out;
}
}
