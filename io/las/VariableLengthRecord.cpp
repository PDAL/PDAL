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

#include "VariableLengthRecord.hpp"

namespace pdal
{

const uint16_t VariableLengthRecord::MAX_DATA_SIZE =
    (std::numeric_limits<uint16_t>::max)();

ILeStream& operator>>(ILeStream& in, VariableLengthRecord& v)
{
    uint16_t reserved;
    uint16_t dataLen;

    in >> reserved;
    in.get(v.m_userId, 16);
    in >> v.m_recordId >> dataLen;
    in.get(v.m_description, 32);
    v.m_data.resize(dataLen);
    in.get(v.m_data);

    return in;
}


OLeStream& operator<<(OLeStream& out, const VariableLengthRecord& v)
{
    out << v.m_recordSig;
    out.put(v.m_userId, 16);
    out << v.m_recordId << (uint16_t)v.dataLen();
    out.put(v.m_description, 32);
    out.put(v.data(), v.dataLen());

    return out;
}


ILeStream& operator>>(ILeStream& in, ExtVariableLengthRecord& v)
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


OLeStream& operator<<(OLeStream& out, const ExtVariableLengthRecord& v)
{
    out << (uint16_t)0;
    out.put(v.userId(), 16);
    out << v.recordId() << v.dataLen();
    out.put(v.description(), 32);
    out.put(v.data(), v.dataLen());

    return out;
}

    void VariableLengthRecord::write(OLeStream& out, uint16_t recordSig)
    {
        m_recordSig = recordSig;
        out << *this;
    }

} // namespace pdal
