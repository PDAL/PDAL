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

#include "LasUtils.hpp"

#include <pdal/util/Extractor.hpp>
#include <pdal/util/Inserter.hpp>
#include <pdal/util/Utils.hpp>

#include <string>

namespace pdal
{

uint8_t ExtraBytesIf::lasType()
{
    using namespace Dimension::Type;
    
    uint8_t lastype = 0;

    Dimension::Type::Enum lastypes[] = {
        None, Unsigned8, Signed8, Unsigned16, Signed16,
        Unsigned32, Signed32, Unsigned64, Signed64, Float, Double
    };
    for (size_t i = 0; i < sizeof(lastypes) / sizeof(lastypes[0]); ++i)
        if (m_type == lastypes[i])
        {
            lastype = i;
            break;
        }
    if (m_fieldCnt == 0 || lastype == 0)
        return 0;
    return 10 * (m_fieldCnt - 1) + lastype;
}


void ExtraBytesIf::setType(uint8_t lastype)
{
    using namespace Dimension::Type;

    m_fieldCnt = 1;
    while (lastype > 10)
    {
        m_fieldCnt++;
        lastype -= 10;
    }

    Dimension::Type::Enum lastypes[] = {
        None, Unsigned8, Signed8, Unsigned16, Signed16,
        Unsigned32, Signed32, Unsigned64, Signed64, Float, Double
    };
    m_type = lastypes[lastype];
    if (m_type == None)
        m_fieldCnt = 0;
}


void ExtraBytesIf::appendTo(std::vector<uint8_t>& ebBytes)
{
    size_t offset = ebBytes.size();
    ebBytes.resize(ebBytes.size() + sizeof(ExtraBytesSpec));
    LeInserter inserter(ebBytes.data() + offset, sizeof(ExtraBytesSpec));

    uint8_t lastype = lasType();
    uint8_t options = lastype ? 0 : m_size;

    inserter << (uint16_t)0 << lastype << options;
    inserter.put(m_name, 32);
    inserter << (uint32_t)0;  // Reserved.
    for (size_t i = 0; i < 3; ++i)
        inserter << (uint64_t)0;  // No data field.
    for (size_t i = 0; i < 3; ++i)
        inserter << (double)0.0; // Min.
    for (size_t i = 0; i < 3; ++i)
        inserter << (double)0.0; // Max.
    for (size_t i = 0; i < 3; ++i)
        inserter << m_scale[i];
    for (size_t i = 0; i < 3; ++i)
        inserter << m_offset[i];
    inserter.put(m_description, 32);
}


void ExtraBytesIf::readFrom(const char *buf)
{
    LeExtractor extractor(buf, sizeof(ExtraBytesSpec));
    uint16_t dummy16;
    uint32_t dummy32;
    uint64_t dummy64;
    double dummyd;
    uint8_t options;
    uint8_t type;

    uint8_t SCALE_MASK = 1 << 3;
    uint8_t OFFSET_MASK = 1 << 4;

    extractor >> dummy16 >> type >> options;
    extractor.get(m_name, 32);
    extractor >> dummy32;
    for (size_t i = 0; i < 3; ++i)
        extractor >> dummy64;  // No data field.
    for (size_t i = 0; i < 3; ++i)
        extractor >> dummyd;  // Min.
    for (size_t i = 0; i < 3; ++i)
        extractor >> dummyd;  // Max.
    for (size_t i = 0; i < 3; ++i)
        extractor >> m_scale[i];
    for (size_t i = 0; i < 3; ++i)
        extractor >> m_offset[i];
    extractor.get(m_description, 32);

    setType(type);
    if (m_type == 0)
        m_size = options;
    if (!(options & SCALE_MASK))
        for (size_t i = 0; i < 3; ++i)
            m_scale[i] = 1.0;
    if (!(options & OFFSET_MASK))
        for (size_t i = 0; i < 3; ++i)
            m_offset[i] = 0.0;
}


std::vector<ExtraDim> ExtraBytesIf::toExtraDims()
{
    std::vector<ExtraDim> eds;

    if (m_type == Dimension::Type::None)
    {
        ExtraDim ed(m_name, Dimension::Type::None);
        ed.m_size = m_size;
        eds.push_back(ed);
    }
    else if (m_fieldCnt == 1)
    {
        ExtraDim ed(m_name, m_type, m_scale[0], m_offset[0]);
        eds.push_back(ed);
    }
    else
    {
        for (size_t i = 0; i < m_fieldCnt; ++i)
        {
            ExtraDim ed(m_name + std::to_string(i), m_type,
                m_scale[i], m_offset[i]);
            eds.push_back(ed);
        }
    }
    return eds;
}

namespace LasUtils
{

std::vector<ExtraDim> parse(const StringList& dimString)
{
    std::vector<ExtraDim> extraDims;

    for (auto& dim : dimString)
    {
        StringList s = Utils::split2(dim, '=');
        if (s.size() != 2)
        {
            std::ostringstream oss;
            oss << "Invalid extra dimension specified: '" << dim <<
                "'.  Need <dimension>=<type>.  See documentation "
                " for details.";
            throw pdal_error(oss.str());
        }
        Utils::trim(s[0]);
        Utils::trim(s[1]);
        Dimension::Type::Enum type = Dimension::type(s[1]);
        if (type == Dimension::Type::None)
        {
            std::ostringstream oss;
            oss << "Invalid extra dimension type specified: '" <<
                dim << "'.  Need <dimension>=<type>.  See documentations "
                " for details.";
            throw pdal_error(oss.str());
        }
        ExtraDim ed(s[0], type);
        extraDims.push_back(ed);
    }
    return extraDims;
}

} // namespace LasUtils

} // namespace pdal
