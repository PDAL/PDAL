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

#include <pdal/Metadata.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/util/ThreadPool.hpp>

namespace pdal
{
namespace las
{

static const uint16_t WktRecordId = 2112;
static const uint16_t GeotiffDirectoryRecordId = 34735;
static const uint16_t GeotiffDoublesRecordId = 34736;
static const uint16_t GeotiffAsciiRecordId = 34737;
static const uint16_t LaszipRecordId = 22204;
static const uint16_t ClassLookupRecordId = 0;
static const uint16_t TextDescriptionRecordId = 3;
static const uint16_t ExtraBytesRecordId = 4;
static const uint16_t PdalMetadataRecordId = 12;
static const uint16_t PdalPipelineRecordId = 13;
static const uint16_t LASFWkt2recordId = 4224;
static const uint16_t PdalProjJsonRecordId = 4225;
static const uint16_t CopcInfoRecordId = 1;
static const uint16_t CopcHierarchyRecordId = 1000;

const std::string TransformUserId = "LASF_Projection";
const std::string SpecUserId = "LASF_Spec";
const std::string LiblasUserId = "liblas";
const std::string LaszipUserId = "laszip encoded";
const std::string PdalUserId = "PDAL";
const std::string CopcUserId = "copc";

struct Vlr;
using VlrList = std::vector<Vlr>;

VlrList parseIgnoreVlrs(const StringList& ignored);
VlrList parseIgnoreVlrs(const StringList& ignored, std::string& error);
const Vlr *findVlr(const std::string& userId, uint16_t recordId, const VlrList& vlrs);
bool shouldIgnoreVlr(const Vlr& v, const VlrList& ignoreList);

struct Vlr
{
public:
    static const uint16_t MaxDataSize {(std::numeric_limits<uint16_t>::max)()};
    static const int HeaderSize {54};

    Vlr() = default;
    Vlr(const std::string& userId, uint16_t recordId, const std::string& description) :
        userId(userId), recordId(recordId), description(description), writeAsEVLR(false)
    {}
    Vlr(const std::string& userId, uint16_t recordId) :
        userId(userId), recordId(recordId), writeAsEVLR(false)
    {}
    virtual ~Vlr() = default;

    char *data()
        { return (char *)(dataVec.data()); }
    const char *data() const
        { return (const char *)(dataVec.data()); }
    size_t dataSize() const
        { return dataVec.size(); }
    bool empty() const
        { return dataVec.size() == 0; }

    virtual void fillHeader(const char *buf);
    virtual std::vector<char> headerData() const;

    uint16_t recordSig {};
    std::string userId;
    uint16_t recordId {};
    uint64_t promisedDataSize;
    std::string description;
    std::vector<char> dataVec;
    std::string metadataId;

    // User specified we want to write this at the back of the file
    bool writeAsEVLR = false;
};

inline bool operator==(const Vlr& v1, const Vlr& v2)
{
    return v1.userId == v2.userId && v1.recordId == v2.recordId;
}


struct Evlr : public Vlr
{
    static const int HeaderSize {60};

    Evlr() = default;
    Evlr(const std::string& userId, uint16_t recordId,
            const std::string& description, const std::vector<char>& data) :
        Vlr(userId, recordId, description)
    { dataVec = data; writeAsEVLR = false;}
    Evlr(const std::string& userId, uint16_t recordId,
            const std::string& description, std::vector<char>&& data) :
        Vlr(userId, recordId, description)
    { dataVec = data; writeAsEVLR = false;}

    virtual void fillHeader(const char *buf) override;
    virtual std::vector<char> headerData() const override;

    using DataCreationFunc = std::function<void(Evlr&, MetadataNode)>;
    DataCreationFunc dataFunc;

    // Fill the VLR data using the data fetch function.
    void fillData(MetadataNode n)
    {
        if (dataFunc)
            dataFunc(*this, n);
    }

    // These allow input and output of VLRs as JSON from streams.
    friend std::istream& operator>>(std::istream& in, Evlr& v);
    friend std::ostream& operator<<(std::ostream& out, const Evlr& v);
};

class VlrCatalog
{
public:
    using ReadFunc = std::function<std::vector<char>(uint64_t offset, int32_t size)>;
    struct Entry
    {
        Entry()
        {}

        Entry(const std::string& userId, uint16_t recordId) :
            userId(userId), recordId(recordId)
        {}

        Entry(const std::string& userId, uint16_t recordId, uint64_t offset, uint64_t length) :
            userId(userId), recordId(recordId), offset(offset), length(length)
        {}

        std::string userId;
        uint16_t recordId = 0;
        uint64_t offset = 0;
        uint64_t length = 0;
    };

private:
    std::mutex m_mutex;
    ReadFunc m_fetch;
    std::deque<Entry> m_entries;

public:
    VlrCatalog(ReadFunc f);
    VlrCatalog(uint64_t vlrOffset, uint32_t vlrCount, uint64_t evlrOffset, uint32_t evlrCount,
        ReadFunc f);

    void load(uint64_t vlrOffset, uint32_t vlrCount, uint64_t evlrOffset, uint32_t evlrCount);
    const Entry& find(const std::string& userId, uint16_t recordId) const;
    std::vector<char> fetch(const std::string& userId, uint16_t recordId) const;
    std::vector<char> fetchWithDescription(const std::string& userId, uint16_t recordId,
        std::string& outDescrip) const;

    using iterator = decltype(m_entries)::iterator;
    using const_iterator = decltype(m_entries)::const_iterator;

    iterator begin()
        { return m_entries.begin(); }
    const_iterator begin() const
        { return m_entries.begin(); }
    iterator end()
        { return m_entries.end(); }
    const_iterator end() const
        { return m_entries.end(); }
    size_t size() const
        { return m_entries.size(); }
    bool exists(const std::string& userId, uint16_t recordId) const;

private:
    void walkVlrs(uint64_t vlrOffset, uint32_t vlrCount);
    void walkEvlrs(uint64_t vlrOffset, uint32_t vlrCount);
    void insert(const Entry& entry);
};

inline bool operator==(const VlrCatalog::Entry& e1, const VlrCatalog::Entry& e2)
    { return e1.userId == e2.userId && e1.recordId == e2.recordId; }

inline bool VlrCatalog::exists(const std::string& userId, uint16_t recordId) const
{
    Entry e(userId, recordId);
    return find(e.userId, e.recordId) == e;
}

} // namespace las
} // namespace pdal
