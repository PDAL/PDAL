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

#include "LasVLR.hpp"

#include <limits>
#include <nlohmann/json.hpp>

#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>

namespace pdal
{

const uint16_t LasVLR::MAX_DATA_SIZE = 65535;

bool LasVLR::read(ILeStream& in, size_t limit)
{
    uint16_t reserved;
    uint16_t dataLen;

    in >> reserved;
    in.get(m_userId, 16);
    in >> m_recordId >> dataLen;
    if ((size_t)in.position() + dataLen > limit)
        return false;
    in.get(m_description, 32);
    m_data.resize(dataLen);
    if (m_data.size() > 0)
        in.get(m_data);

    return true;
}


std::istream& operator>>(std::istream& in, LasVLR& v)
{
    NL::json j;
    in >> j;

    // Make sure there isn't stuff in the input stream after the JSON
    // object.
    char c;
    do
    {
        in >> c;
    } while (in.good() && std::isspace(c));
    if (!in.eof())
        throw pdal_error("Invalid characters following LAS VLR JSON object.");

    // We forced an EOF above, so clear the error state.
    in.clear();
    if (!j.is_object())
        throw pdal_error("LAS VLR must be specified as a JSON object.");

    std::string description;
    std::string b64data;
    std::string userId;
    std::vector<uint8_t> data;
    double recordId(std::numeric_limits<double>::quiet_NaN());
    for (auto& el : j.items())
    {
        if (el.key() == "description")
        {
            if (!el.value().is_string()) 
                throw pdal_error("LAS VLR description must be specified "
                    "as a string.");
            description = el.value().get<std::string>();
            if (description.size() > 32)
                throw pdal_error("LAS VLR description must be 32 characters "
                    "or less.");
        }
        else if (el.key() == "record_id")
        {
            if (!el.value().is_number())
                throw pdal_error("LAS VLR record ID must be specified as "
                    "a number.");
            recordId = el.value().get<double>();
            if (recordId < 0 ||
                recordId > (std::numeric_limits<uint16_t>::max)() ||
                recordId != (uint16_t)recordId)
                throw pdal_error("LAS VLR record ID must be an non-negative "
                    "integer less than 65536.");
        }
        else if (el.key() == "user_id")
        {
            if (!el.value().is_string()) 
                throw pdal_error("LAS VLR user ID must be specified "
                    "as a string.");
            userId = el.value().get<std::string>();
            if (userId.size() > 16)
                throw pdal_error("LAS VLR user ID must be 16 characters "
                    "or less.");
        }
        else if (el.key() == "data")
        {
            if (data.size())
                throw pdal_error("Can't specify both 'data' and 'filename' "
                    "in VLR specification.");
            if (!el.value().is_string())
                throw pdal_error("LAS VLR data must be specified as "
                    "a base64-encoded string.");
            data = Utils::base64_decode(el.value().get<std::string>());
        }
        else if (el.key() == "filename")
        {
            if (data.size())
                throw pdal_error("Can't specify both 'data' and 'filename' "
                    "in VLR specification.");
            if (!el.value().is_string())
                throw pdal_error("LAS VLR filename must be a string.");
            std::string filename = el.value().get<std::string>();
            size_t fileSize = FileUtils::fileSize(filename);
            auto ctx = FileUtils::mapFile(filename, true, 0, fileSize);
            if (ctx.addr())
            {
                uint8_t *addr = reinterpret_cast<uint8_t *>(ctx.addr());
                data.insert(data.begin(), addr, addr + fileSize);
            }
            FileUtils::unmapFile(ctx);
            if (!ctx.addr())
                throw pdal_error("Couldn't open file '" + filename + "' "
                    "from which to read VLR data: " + ctx.what());
        }
        else
            throw pdal_error("Invalid key '" + el.key() + "' in VLR "
                "specification.");
    }
    if (data.size() == 0)
        throw pdal_error("LAS VLR must contain 'data' member.");
    if (userId.empty())
        throw pdal_error("LAS VLR must contain 'user_id' member.");
    if (std::isnan(recordId))
        recordId = 1;

    v.m_userId = userId;
    v.m_recordId = (uint16_t)recordId;
    v.m_description = description;
    v.m_data = std::move(data);
    return in;
}


std::istream& operator>>(std::istream& in, ExtLasVLR& v)
{
    return (in >> (LasVLR&)v);
}


std::ostream& operator<<(std::ostream& out, const LasVLR& v)
{
    const unsigned char *d(reinterpret_cast<const unsigned char *>(v.data()));

    out << "{\n";
    out << "  \"description\": \"" << v.description() << "\",\n";
    out << "  \"record_id\": " << v.recordId() << ",\n";
    out << "  \"user_id\": \"" << v.userId() << "\",\n";
    out << "  \"data\": \"" <<Utils::base64_encode(d, v.dataLen()) << "\"\n";
    out << "}\n";
    return out;
}

std::ostream& operator<<(std::ostream& out, const ExtLasVLR& v)
{
    return (out << static_cast<const LasVLR&>(v));
}


OLeStream& operator<<(OLeStream& out, const LasVLR& v)
{
    out << v.m_recordSig;
    out.put(v.m_userId, 16);
    out << v.m_recordId << (uint16_t)v.dataLen();
    out.put(v.m_description, 32);
    out.put(v.data(), v.dataLen());

    return out;
}


bool ExtLasVLR::read(ILeStream& in, uintmax_t limit)
{
    uint64_t dataLen;

    in >> m_recordSig;
    in.get(m_userId, 16);
    in >> m_recordId >> dataLen;
    if (uintmax_t(in.position()) + dataLen > limit)
        return false;
    in.get(m_description, 32);
    m_data.resize(dataLen);
    if (m_data.size() > 0)
        in.get(m_data);
    return true;
}


OLeStream& operator<<(OLeStream& out, const ExtLasVLR& v)
{
    out << (uint16_t)0;
    out.put(v.userId(), 16);
    out << v.recordId() << v.dataLen();
    out.put(v.description(), 32);
    out.put(v.data(), v.dataLen());

    return out;
}

void LasVLR::write(OLeStream& out, uint16_t recordSig)
{
    m_recordSig = recordSig;
    out << *this;
}

} // namespace pdal
