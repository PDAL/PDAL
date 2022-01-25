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
*     * Neither the name of Hobu, Inc. nor the
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

#include "Vlr.hpp"

#include <nlohmann/json.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Inserter.hpp>
#include <pdal/util/Extractor.hpp>

namespace pdal
{
namespace las
{

VlrList parseIgnoreVlrs(const StringList& ignored)
{
    std::string error;
    return parseIgnoreVlrs(ignored, error);
}

VlrList parseIgnoreVlrs(const StringList& ignored, std::string& error)
{
    error.clear();

    // Always ignore COPC VLRs and superseded VLRs.
    VlrList ignoredVlrs { {"copc", 0, "ALL"}, {"LASF_Spec", 7} };
    for (auto& v: ignored)
    {
        StringList s = Utils::split2(v, '/');
        if (s.size() == 2)
        {
            Utils::trim(s[0]);
            Utils::trim(s[1]);
            int i = std::stoi(s[1]);
            ignoredVlrs.emplace_back(s[0], (uint16_t)i);
        }
        else if (s.size() == 1)
        {
            Utils::trim(s[0]);
            // Stick "ALL" in as a description since 0 is a valid record ID.
            ignoredVlrs.emplace_back(s[0], 0, "ALL");
        }
        else
        {
            error = "Invalid VLR specification (" + v + ") to ignore.";
            break;
        }
    }
    return ignoredVlrs;
}

bool shouldIgnoreVlr(const Vlr& v, const VlrList& vlrs)
{
    // We ignore VLRs when the user IDs match and the test description is "ALL" or
    // the record IDs match.
    for (const Vlr& ignore : vlrs)
    {
        if ((v.userId == ignore.userId) &&
            ((ignore.description == "ALL") || (v.recordId == ignore.recordId)))
                return true;
    }
    return false;
}

const Vlr *findVlr(const std::string& userId, uint16_t recordId, const VlrList& vlrs)
{
    auto it = std::find(vlrs.begin(), vlrs.end(), Vlr(userId, recordId));
    if (it == vlrs.end())
        return nullptr;
    return (&(*it));
}

void Vlr::fillHeader(const char *buf)
{
    LeExtractor in(buf, Vlr::HeaderSize);
    uint16_t dataLen;

    in >> recordSig;
    in.get(userId, 16);
    in >> recordId >> dataLen;
    in.get(description, 32);
    promisedDataSize = dataLen;
}

std::vector<char> Vlr::headerData() const
{
    std::vector<char> buf(HeaderSize);

    assert(dataVec.size() <= (std::numeric_limits<uint16_t>::max)());
    LeInserter out(buf.data(), HeaderSize);
    out << recordSig;
    out.put(userId, 16);
    out << recordId << (uint16_t)(dataVec.size());
    out.put(description, 32);

    return buf;
}

void Evlr::fillHeader(const char *buf)
{
    LeExtractor in(buf, Evlr::HeaderSize);

    in >> recordSig;
    in.get(userId, 16);
    in >> recordId >> promisedDataSize;
    in.get(description, 32);
}

std::vector<char> Evlr::headerData() const
{
    std::vector<char> buf(HeaderSize);

    LeInserter out(buf.data(), HeaderSize);
    out << recordSig;
    out.put(userId, 16);
    out << recordId << (uint64_t)dataVec.size();
    out.put(description, 32);

    return buf;
}

std::istream& operator>>(std::istream& in, las::Evlr& v)
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
    std::vector<char> data;
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
            //ABELL - Fix this.
            std::vector<uint8_t> b64buf;
            b64buf = Utils::base64_decode(el.value().get<std::string>());
            data.insert(data.end(), (char *)b64buf.data(), (char *)(b64buf.data() + b64buf.size()));
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

    v.userId = userId;
    v.recordId = (uint16_t)recordId;
    v.description = description;
    v.dataVec = std::move(data);
    return in;
}

std::ostream& operator<<(std::ostream& out, const las::Evlr& v)
{
    const unsigned char *d(reinterpret_cast<const unsigned char *>(v.data()));

    out << "{\n";
    out << "  \"description\": \"" << v.description << "\",\n";
    out << "  \"record_id\": " << v.recordId << ",\n";
    out << "  \"user_id\": \"" << v.userId << "\",\n";
    out << "  \"data\": \"" <<Utils::base64_encode(d, v.dataSize()) << "\"\n";
    out << "}\n";
    return out;
}

// VLR Catalog

VlrCatalog::VlrCatalog(VlrCatalog::ReadFunc f) : m_fetch(f)
{}

VlrCatalog::VlrCatalog(uint64_t vlrOffset, uint32_t vlrCount,
    uint64_t evlrOffset, uint32_t evlrCount, VlrCatalog::ReadFunc f) : m_fetch(f)
{
    load(vlrOffset, vlrCount, evlrOffset, evlrCount);
}

void VlrCatalog::load(uint64_t vlrOffset, uint32_t vlrCount,
    uint64_t evlrOffset, uint32_t evlrCount)
{
    auto vlrWalker = std::bind(&VlrCatalog::walkVlrs, this, vlrOffset, vlrCount);
    auto evlrWalker = std::bind(&VlrCatalog::walkEvlrs, this, evlrOffset, evlrCount);

    ThreadPool pool(2);

    if (vlrCount)
        pool.add(vlrWalker);
    if (evlrCount)
        pool.add(evlrWalker);
    pool.await();
}

void VlrCatalog::walkVlrs(uint64_t vlrOffset, uint32_t vlrCount)
{
    while (vlrOffset && vlrCount)
    {
        std::vector<char> buf = m_fetch(vlrOffset, Vlr::HeaderSize);

        Vlr vlr;
        vlr.fillHeader(buf.data());
        Entry entry { vlr.userId, vlr.recordId, vlrOffset + Vlr::HeaderSize, vlr.promisedDataSize };
        insert(entry);
        vlrOffset += Vlr::HeaderSize + vlr.promisedDataSize;
        vlrCount--;
    }
}

void VlrCatalog::walkEvlrs(uint64_t evlrOffset, uint32_t evlrCount)
{
    while (evlrOffset && evlrCount)
    {
        std::vector<char> buf = m_fetch(evlrOffset, Evlr::HeaderSize);

        Evlr vlr;
        vlr.fillHeader(buf.data());
        Entry entry { vlr.userId, vlr.recordId, evlrOffset + Evlr::HeaderSize,
            vlr.promisedDataSize };
        insert(entry);
        evlrOffset += Evlr::HeaderSize + vlr.promisedDataSize;
        evlrCount--;
    }
}

void VlrCatalog::insert(const VlrCatalog::Entry& entry)
{
    std::unique_lock<std::mutex> l(m_mutex);

    m_entries.push_back(entry);
}

std::vector<char> VlrCatalog::fetch(const std::string& userId, uint16_t recordId) const
{
    uint64_t offset = 0;
    uint32_t length = 0;

    // We don't lock m_entries because we assume that the load has already occurred at the
    // time you want to fetch.
    std::vector<char> vlrdata;
    for (const Entry& e : m_entries)
        if (e.userId == userId && e.recordId == recordId)
        {
            // We don't support VLRs with size > 4GB)
            if (e.length > (std::numeric_limits<uint32_t>::max)())
                return vlrdata;

            offset = e.offset;
            length = (uint32_t)e.length;
            break;
        }

    if (length > 0)
        vlrdata = m_fetch(offset, length);
    return vlrdata;
}

std::vector<char> VlrCatalog::fetchWithDescription(const std::string& userId, uint16_t recordId,
    std::string& outDescrip) const
{
    uint64_t offset = 0;
    uint32_t length = 0;

    // We don't lock m_entries because we assume that the load has already occurred at the
    // time you want to fetch.
    std::vector<char> vlrdata;
    for (const Entry& e : m_entries)
        if (e.userId == userId && e.recordId == recordId)
        {
            // We don't support VLRs with size > 4GB)
            if (e.length > (std::numeric_limits<uint32_t>::max)())
                return vlrdata;

            offset = e.offset;
            length = (uint32_t)e.length;
            break;
        }
    if (length == 0)
        return vlrdata;

    // Load the data plus the description.
    const int DescripLen = 32;
    assert(offset > DescripLen);
    // Add space for the description that precedes the data
    std::vector<char> v = m_fetch(offset - DescripLen, length + DescripLen);

    // Only make the string with non-null characters.
    int len = DescripLen - 1;
    while (len >= 0)
    {
        if (v[len])
            break;
        len--;
    }
    outDescrip.assign(v.data(), v.data() + len + 1);

    // Assign the payload of the VLR to the output VLR.
    vlrdata.assign(v.data() + DescripLen, v.data() + v.size());
    return vlrdata;
}


} // namespace las
} // namespace pdal
