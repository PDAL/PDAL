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

#pragma once

#include <Eigen/Dense>

#include <pdal/Dimension.hpp>
#include <pdal/util/OStream.hpp>

namespace pdal
{

enum class PcdFieldType
{
    unknown,
    I,
    U,
    F
};
std::istream& operator>>(std::istream& in, PcdFieldType& type);
std::ostream& operator<<(std::ostream& out, PcdFieldType& type);

enum class PcdVersion
{
    unknown,
    PCD_V6,
    PCD_V7
};
std::istream& operator>>(std::istream& in, PcdVersion& version);
std::ostream& operator<<(std::ostream& out, PcdVersion& version);

enum class PcdDataStorage
{
    unknown,
    ASCII,
    BINARY,
    COMPRESSED
};
std::istream& operator>>(std::istream& in, PcdDataStorage& storage);
std::ostream& operator<<(std::ostream& out, PcdDataStorage& storage);

struct PcdField
{
    PcdField()
        : m_id(Dimension::Id::Unknown), m_size(4),
          m_type(PcdFieldType::unknown), m_count(1)
    {
    }

    PcdField(std::string& label) : PcdField()
    {
        m_id = Dimension::id(label);
        m_label = label;
    }

    std::string m_label;
    Dimension::Id m_id;
    uint32_t m_size;
    PcdFieldType m_type;
    uint32_t m_count;
};
typedef std::vector<PcdField> PcdFieldList;

struct PcdHeader
{
    PcdHeader();

    void clear();

    PcdVersion m_version;
    PcdFieldList m_fields;
    point_count_t m_width;
    point_count_t m_height;
    point_count_t m_pointCount;

    Eigen::Vector4f m_origin;
    Eigen::Quaternionf m_orientation;

    PcdDataStorage m_dataStorage;
    std::istream::pos_type m_dataOffset;
    size_t m_numLines;
};

std::istream& operator>>(std::istream& in, PcdHeader& header);
std::ostream& operator<<(std::ostream& out, PcdHeader& header);
OLeStream& operator<<(OLeStream& out, PcdHeader& header);

} // namespace pdal

