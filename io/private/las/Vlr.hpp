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
namespace las
{

static const int WktRecordId = 2112;
static const uint16_t GeotiffDirectoryRecordId = 34735;
static const uint16_t GeotiffDoublesRecordId = 34736;
static const uint16_t GeotiffAsciiRecordId = 34737;
static const uint16_t LaszipRecordId = 22204;
static const uint16_t ClassLookupRecordId = 0;
static const uint16_t TextDescriptionRecordId = 3;
static const uint16_t ExtraBytesRecordId = 4;
static const uint16_t PdalMetadataRecordId = 12;
static const uint16_t PdalPipelineRecordId = 13;

const std::string TransformUserId = "LASF_Projection";
const std::string SpecUserId = "LASF_Spec";
const std::string LiblasUserId = "liblas";
const std::string LaszipUserId = "laszip encoded";
const std::string PdalUserId = "PDAL";

struct Vlr;
using VlrList = std::vector<Vlr>;

VlrList parseIgnoreVLRs(const StringList& ignored, std::string& error);
const Vlr *findVlr(const std::string& userId, uint16_t recordId, const VlrList& vlrs);
bool shouldIgnoreVlr(const Vlr& v, const VlrList& ignoreList);

struct Vlr
{
public:
//ABELL
//    static const uint16_t MAX_DATA_SIZE;
    static const int HeaderSize {54};

    Vlr() = default;
    Vlr(const std::string& userId, uint16_t recordId, const std::string& description) :
        userId(userId), recordId(recordId), description(description)
    {}
    Vlr(const std::string& userId, uint16_t recordId) :
        userId(userId), recordId(recordId)
    {}

    char *data()
        { return (char *)(dataVec.data()); }
    const char *data() const
        { return (const char *)(dataVec.data()); }
    size_t dataSize() const
        { return dataVec.size(); }
    size_t empty() const
        { return dataVec.size() == 0; }

    void fillHeader(const char *buf);
    /**
    void write(OLeStream& out, uint16_t recordSig);
    bool read(ILeStream& in, size_t limit);

    friend OLeStream& operator<<(OLeStream& out, const LasVLR& v);
    friend std::istream& operator>>(std::istream& in, LasVLR& v);
    friend std::ostream& operator<<(std::ostream& out, const LasVLR& v);
    **/

    uint16_t recordSig {};
    std::string userId;
    uint16_t recordId {};
    uint64_t promisedDataSize;
    std::string description;
    std::vector<uint8_t> dataVec;
};

inline bool operator==(const Vlr& v1, const Vlr& v2)
{
    return v1.userId == v2.userId && v1.recordId == v2.recordId;
}


struct Evlr : public Vlr
{
    static const int HeaderSize {54};

    Evlr() = default;

    void fillHeader(const char *buf);

    /**
    ExtLasVLR(const std::string& userId, uint16_t recordId,
            const std::string& description, std::vector<uint8_t>& data) :
        LasVLR(userId, recordId, description, data)
    {}

    bool read(ILeStream& in, uintmax_t limit);

    friend OLeStream& operator<<(OLeStream& out, const ExtLasVLR& v);
    friend std::istream& operator>>(std::istream& in, ExtLasVLR& v);
    friend std::ostream& operator<<(std::ostream& out, const ExtLasVLR& v);
    **/
};

} // namespace las
} // namespace pdal
