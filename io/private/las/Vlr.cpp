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
    // Trim all characters after a NULL
    userId = userId.data();
    in >> recordId >> dataLen;
    in.get(description, 32);
    // Trim all characters after a NULL
    description = description.data();
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
    // Trim all characters after a NULL
    userId = userId.data();
    in >> recordId >> promisedDataSize;
    in.get(description, 32);
    // Trim all characters after a NULL
    description = description.data();
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

// Fill VLR data from base-64 encoded data.
struct DataFunc
{
    DataFunc(const std::string& data) : m_data(data)
    {}

    void operator()(las::Evlr& v, MetadataNode)
    {
        std::vector<uint8_t> b64buf = Utils::base64_decode(m_data);
        v.dataVec.insert(v.dataVec.end(), b64buf.data(), (b64buf.data() + b64buf.size()));
    }

    std::string m_data;
};

// Fill VLR data from a file.
struct FileFunc
{
    FileFunc(const std::string& filename) : m_filename(filename)
    {}

    void operator()(las::Evlr& v, MetadataNode)
    {
        size_t fileSize = FileUtils::fileSize(m_filename);
        auto ctx = FileUtils::mapFile(m_filename, true, 0, fileSize);
        if (ctx.addr())
        {
            uint8_t *addr = reinterpret_cast<uint8_t *>(ctx.addr());
            v.dataVec.insert(v.dataVec.end(), addr, addr + fileSize);
        }
        FileUtils::unmapFile(ctx);
        if (!ctx.addr())
            throw pdal_error("Couldn't open file '" + m_filename +
                "' from which to read VLR data: " + ctx.what());
    }

    std::string m_filename;
};

// Fill VLR data from metadata.
struct MetadataFunc
{
    MetadataFunc(const std::string& key) : m_key(key)
    {}

    void operator()(las::Evlr& v, MetadataNode m)
    {
        auto pred = [key=m_key](MetadataNode m)
            { return Utils::iequals(m.name(), key); };

        MetadataNode node = m.find(pred);
        if (!node.valid())
            throw pdal_error("Unable to find metadata entry for key '" + m_key + "'.");

        if (node.type() == "base64Binary")
        {
            std::vector<uint8_t> b64buf = Utils::base64_decode(node.value());
            v.dataVec.insert(v.dataVec.end(), b64buf.data(), (b64buf.data() + b64buf.size()));
        }
        else if (node.type() == "string")
        {
            const std::string& s = node.value();
            v.dataVec.insert(v.dataVec.end(), s.data(), s.data() + s.size());
        }
        else
            throw pdal_error("Metadata for key '" + m_key + "' is not a string or base64 encoded.");
    }

    std::string m_key;
};


std::istream& operator>>(std::istream& in, las::Evlr& v)
{
    nlohmann::json j;
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

    std::string b64data;
    std::vector<char> data;

    v.description.clear();
    v.userId.clear();
    v.recordId = 1;
    for (auto& el : j.items())
    {
        if (el.key() == "description")
        {
            if (!el.value().is_string())
                throw pdal_error("LAS VLR description must be specified as a string.");
            v.description = el.value().get<std::string>();
            if (v.description.size() > 32)
                throw pdal_error("LAS VLR description must be 32 characters or less.");
        }
        else if (el.key() == "record_id")
        {
            if (!el.value().is_number())
                throw pdal_error("LAS VLR record ID must be specified as a number.");
            double d = el.value().get<double>();
            if (d < 0 ||
                d > (std::numeric_limits<uint16_t>::max)() ||
                d != (uint16_t)d)
                throw pdal_error("LAS VLR record ID must be an non-negative "
                    "integer less than 65536.");
            v.recordId = (uint16_t)d;
        }
        else if (el.key() == "user_id")
        {
            if (!el.value().is_string())
                throw pdal_error("LAS VLR user ID must be specified as a string.");
            v.userId = el.value().get<std::string>();
            if (v.userId.size() > 16)
                throw pdal_error("LAS VLR user ID must be 16 characters or less.");
        }
        else if (el.key() == "data")
        {
            if (v.dataFunc)
                throw pdal_error("VLR can only be specified as one of "
                    "'data', 'metadata' or 'filename'.");
            if (!el.value().is_string())
                throw pdal_error("LAS VLR data must be specified as a string.");
            const std::string& data = el.value().get<std::string>();

            v.dataFunc = DataFunc(data);
        }
        else if (el.key() == "filename")
        {
            if (v.dataFunc)
                throw pdal_error("VLR can only be specified as one of "
                    "'data', 'metadata' or 'filename'.");
            if (!el.value().is_string())
                throw pdal_error("LAS VLR filename must be a string.");
            const std::string& filename = el.value().get<std::string>();

            v.dataFunc = FileFunc(filename);
        }
        else if (el.key() == "metadata")
        {
            if (v.dataFunc)
                throw pdal_error("VLR can only be specified as one of "
                    "'data', 'metadata' or 'filename'.");
            if (!el.value().is_string())
                throw pdal_error("LAS VLR metadata key must be specified as a string.");
            const std::string& metadataId = el.value().get<std::string>();

            v.dataFunc = MetadataFunc(metadataId);
        }
        else if (el.key() == "evlr")
        {
            if (!el.value().is_boolean())
                throw pdal_error("LAS VLR  key must be specified as a boolean.");
            v.writeAsEVLR = el.value().get<bool>();
        }
        else
            throw pdal_error("Invalid key '" + el.key() + "' in VLR specification.");
    }

    if (v.userId.empty())
        throw pdal_error("LAS VLR must contain 'user_id' member.");
    if (!v.dataFunc)
        throw pdal_error("LAS VLR must contain a 'data', 'metadata' or 'filename' member.");
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

const VlrCatalog::Entry& VlrCatalog::find(const std::string& userId, uint16_t recordId) const
{
    static const VlrCatalog::Entry nullEntry;

    auto it = std::find_if(m_entries.begin(), m_entries.end(),
        [&](const Entry& e){ return e.userId == userId && e.recordId == recordId; });
    return it == m_entries.end() ? nullEntry : *it;
}

std::vector<char> VlrCatalog::fetchWithDescription(const std::string& userId, uint16_t recordId,
    std::string& outDescrip) const
{
    // We don't lock m_entries because we assume that the load has already occurred at the
    // time you want to fetch.
    std::vector<char> vlrdata;
    const Entry& e = find(userId, recordId);

    // We don't support VLRs with size > 4GB
    if (e.length == 0 || e.length > (std::numeric_limits<uint32_t>::max)())
        return vlrdata;

    uint64_t offset = e.offset;
    uint32_t length = (uint32_t)e.length;

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
