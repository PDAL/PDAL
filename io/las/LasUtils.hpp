/******************************************************************************
 * Copyright (c) 2015, Hobu Inc.
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <pdal/Dimension.hpp>
#include <string>

namespace pdal
{

struct ExtraDim
{
    ExtraDim(const std::string name, Dimension::Type::Enum type,
            double scale = 1.0, double offset = 0.0) :
        m_name(name), m_dimType(Dimension::Id::Unknown, type, scale, offset),
        m_size(0)
    {}

    friend bool operator == (const ExtraDim& ed1, const ExtraDim& ed2);

    std::string m_name;
    DimType m_dimType;
    size_t m_size;  // Only set when type is None.
};

inline bool operator == (const ExtraDim& ed1, const ExtraDim& ed2)
{
    // This is an incomplete comparison, but it should suffice since we
    // only use it to compare an ExtraDim specified in an option with
    // one created from a VLR entry.
    return (ed1.m_name == ed2.m_name &&
        ed1.m_dimType.m_type == ed2.m_dimType.m_type &&
        ed1.m_size == ed2.m_size);
}

// This is the structure of each record in the extra bytes spec.  Not used
// directly for storage, but here mostly for reference.
struct ExtraBytesSpec
{
    char m_reserved[2];
    uint8_t m_dataType;
    uint8_t m_options;
    char m_name[32];
    char m_reserved2[4];
    uint64_t m_noData[3]; // 24 = 3*8 bytes
    double m_min[3]; // 24 = 3*8 bytes
    double m_max[3]; // 24 = 3*8 bytes
    double m_scale[3]; // 24 = 3*8 bytes
    double m_offset[3]; // 24 = 3*8 bytes
    char m_description[32];
};

class ExtraBytesIf
{
public:
    ExtraBytesIf() : m_type(Dimension::Type::None), m_fieldCnt(0), m_size(0)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            m_scale[i] = 1.0;
            m_offset[i] = 0.0;
        }
    }

    ExtraBytesIf(const std::string& name, Dimension::Type::Enum type,
            const std::string& description) :
        m_type(type), m_name(name), m_description(description), m_size(0)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            // Setting the scale to 0 looks wrong, but it isn't.  If the
            // scale option flag isn't set, the scale is supposed to be 0.
            // When we write the VLR, we always clear the scale flag.
            m_scale[i] = 0.0;
            m_offset[i] = 0.0;
        }
        m_fieldCnt = (m_type == Dimension::Type::None ? 0 : 1);
    }

    void appendTo(std::vector<uint8_t>& ebBytes);
    void readFrom(const char *buf);
    uint8_t lasType();
    void setType(uint8_t lastype);
    std::vector<ExtraDim> toExtraDims();

private:
    Dimension::Type::Enum m_type;
    unsigned m_fieldCnt; // Must be 0 - 3;
    double m_scale[3];
    double m_offset[3];
    std::string m_name;
    std::string m_description;
    size_t m_size;
};

namespace LasUtils
{

std::vector<ExtraDim> parse(const StringList& dimString);

} // namespace LasUtils

} // namespace pdal
