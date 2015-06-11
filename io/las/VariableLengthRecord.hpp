/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <limits>
#include <string>
#include <vector>

#include <pdal/SpatialReference.hpp>
#include <pdal/util/IStream.hpp>
#include <pdal/util/OStream.hpp>

namespace pdal
{

static const int WKT_RECORD_ID = 2112;
static const uint16_t GEOTIFF_DIRECTORY_RECORD_ID = 34735;
static const uint16_t GEOTIFF_DOUBLES_RECORD_ID = 34736;
static const uint16_t GEOTIFF_ASCII_RECORD_ID = 34737;
static const uint16_t LASZIP_RECORD_ID = 22204;
static const uint16_t EXTRA_BYTES_RECORD_ID = 4;

static const char TRANSFORM_USER_ID[] = "LASF_Projection";
static const char SPEC_USER_ID[] = "LASF_Spec";
static const char LIBLAS_USER_ID[] = "liblas";
static const char LASZIP_USER_ID[] = "laszip encoded";

class VariableLengthRecord;
typedef std::vector<VariableLengthRecord> VlrList;

class PDAL_DLL VariableLengthRecord
{
public:
    static const uint16_t MAX_DATA_SIZE;

    VariableLengthRecord(const std::string& userId, uint16_t recordId,
            const std::string& description, std::vector<uint8_t>& data) :
        m_userId(userId), m_recordId(recordId), m_description(description),
        m_data(std::move(data)), m_recordSig(0)
    {}
    VariableLengthRecord() : m_recordId(0), m_recordSig(0)
    {}

    std::string userId() const
        { return m_userId;}
    uint16_t recordId() const
        { return m_recordId; }
    std::string description() const
        { return m_description; }
    
    bool matches(const std::string& userId) const
        { return userId == m_userId; }
    bool matches(const std::string& userId, uint16_t recordId) const
        { return matches(userId) && (recordId == m_recordId); }

    const char* data() const
        { return (const char *)m_data.data(); }
    char* data()
        { return (char *)m_data.data(); }
    uint64_t dataLen() const
        { return m_data.size(); }
    void setDataLen(uint64_t size)
        { m_data.resize((size_t)size); }
    void write(OLeStream& out, uint16_t recordSig);

    friend ILeStream& operator>>(ILeStream& in, VariableLengthRecord& v);
    friend OLeStream& operator<<(OLeStream& out, const VariableLengthRecord& v);

protected:
    std::string m_userId;
    uint16_t m_recordId;
    std::string m_description;
    std::vector<uint8_t> m_data;
    uint16_t m_recordSig;
};

class ExtVariableLengthRecord : public VariableLengthRecord
{
public:
    ExtVariableLengthRecord(const std::string& userId, uint16_t recordId,
            const std::string& description, std::vector<uint8_t>& data) :
        VariableLengthRecord(userId, recordId, description, data)
    {}
    ExtVariableLengthRecord()
    {}

    friend ILeStream& operator>>(ILeStream& in, ExtVariableLengthRecord& v);
    friend OLeStream& operator<<(OLeStream& out,
        const ExtVariableLengthRecord& v);
};

} // namespace pdal
