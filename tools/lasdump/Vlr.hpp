/******************************************************************************
 * Copyright (c) 2014, Hobu Inc. (hobu@hobu.co)
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

#include "Lasdump.hpp"

#include <pdal/util/IStream.hpp>

namespace pdal
{
namespace lasdump
{

class Vlr
{
public:
    bool matches(std::string userId, uint16_t recordId)
        { return userId == userId && recordId == m_recordId; }
    const char *data() const
        { return (const char *)m_data.data(); }
    uint64_t dataLen() const
        { return m_data.size(); }

protected:
    uint16_t m_recordSig;
    std::string m_userId;
    uint16_t m_recordId;
    std::string m_description;
    std::vector<uint8_t> m_data;

    friend ILeStream& operator>>(ILeStream& in, Vlr& v);
    friend std::ostream& operator<<(std::ostream& out, Vlr& v);
};

class EVlr : public Vlr
{
    friend ILeStream& operator>>(ILeStream& in, EVlr& v);
};

inline std::ostream& operator<<(std::ostream& out, Vlr& v)
{
    out << "Record Signature: " << v.m_recordSig << "\n";
    out << "User ID: " << v.m_userId << "\n";
    out << "Record ID: " << v.m_recordId << "\n";
    out << "Description: " << v.m_description << "\n";
    out << "Data checksum: " << cksum(v.m_data) << "\n";

    return out;
}

inline ILeStream& operator>>(ILeStream& in, Vlr& v)
{
    uint16_t dataLen;

    in >> v.m_recordSig;
    in.get(v.m_userId, 16);
    in >> v.m_recordId >> dataLen;
    in.get(v.m_description, 32);
    v.m_data.resize(dataLen);
    in.get(v.m_data);

    return in;
}

inline ILeStream& operator>>(ILeStream& in, EVlr& v)
{
    uint64_t dataLen;

    in >> v.m_recordSig;
    in.get(v.m_userId, 16);
    in >> v.m_recordId >> dataLen;
    in.get(v.m_description, 32);
    v.m_data.resize(dataLen);
    in.get(v.m_data);

    return in;
}

} // namespace lasdump
} // namespace pdal

