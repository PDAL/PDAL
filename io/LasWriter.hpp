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

#include <pdal/pdal_features.hpp>

#include "HeaderVal.hpp"

#include <pdal/FlexWriter.hpp>
#include <pdal/Streamable.hpp>

#include "private/las/Vlr.hpp"

namespace pdal
{
class LeInserter;
class LasTester;
class NitfWriter;
class GeotiffSupport;
class LazPerfVlrCompressor;

namespace las
{
    struct Header;
    struct Vlr;
    struct Evlr;
    struct ExtraDim;
}

struct VlrOptionInfo
{
    std::string m_name;
    std::string m_value;
    std::string m_userId;
    uint16_t m_recordId;
    std::string m_description;
};

class PDAL_DLL LasWriter : public FlexWriter, public Streamable
{
    friend class LasTester;
    friend class NitfWriter;

    struct Options;
    struct Private;

public:
    std::string getName() const;

    LasWriter();
    ~LasWriter();

protected:
    void prepOutput(std::ostream *out, const SpatialReference& srs);
    void finishOutput();

private:
    std::unique_ptr<Private> d;

    LazPerfVlrCompressor *m_compressor;
    std::ostream *m_ostream;
    std::vector<las::Vlr> m_vlrs;
    std::vector<las::Evlr> m_evlrs;
    std::vector<las::ExtraDim> m_extraDims;
    uint16_t m_extraByteLen;
    SpatialReference m_srs;
    std::set<std::string> m_forwards;
    std::vector<char> m_pointBuf;
    int m_srsCnt;

    MetadataNode m_forwardMetadata;
    bool m_firstPoint;

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void prepared(PointTableRef table);
    virtual void readyTable(PointTableRef table);
    virtual void readyFile(const std::string& filename, const SpatialReference& srs);
    virtual bool srsOverridden() const;
    void prerunFile(const PointViewSet& pvSet);
    virtual void writeView(const PointViewPtr view);
    virtual bool processOne(PointRef& point);
    void spatialReferenceChanged(const SpatialReference& srs);
    virtual void doneFile();

    void handleLaszip(int result);
    void fillForwardList();
    void addUserVlrs(MetadataNode m);
    template <typename T>
    void handleHeaderForward(const std::string& s, T& headerVal,
        const MetadataNode& base);
    void handleHeaderForwards(MetadataNode& forward);
    void fillHeader();
    void validateHeader();
    bool fillPointBuf(PointRef& point, LeInserter& ostream);
    point_count_t fillWriteBuf(const PointView& view, PointId startId,
        std::vector<char>& buf);
    bool writeLasZipBuf(PointRef& point);
    void writeLazPerfBuf(char *data, size_t pointLen, point_count_t numPts);
    void addForwardVlrs();
    void addMetadataVlr(MetadataNode& forward);
    void addPipelineVlr();
    void addExtraBytesVlr();
    void addSpatialRefVlrs();
    MetadataNode findVlrMetadata(MetadataNode node, uint16_t recordId,
        const std::string& userId);
    void readyCompression();
    void readyLasZipCompression();
    void readyLazPerfCompression();
    void openCompression();
    void addVlr(const std::string& userId, uint16_t recordId,
        const std::string& description, const std::vector<char>& data);
    void addVlr(const std::string& userId, uint16_t recordId,
        const std::string& description, const std::vector<uint8_t>& data);
    void addVlr(const std::string& userId, uint16_t recordId,
        const std::string& description, std::vector<char>&& data);
    void addVlr(const las::Evlr& evlr);
    void deleteVlr(const std::string& userId, uint16_t recordId);
    void addGeotiffVlrs();
    bool addWktVlr();
    void finishLasZipOutput();
    void finishLazPerfOutput();
    bool processPoint(PointRef& point);
    const las::Header& header() const;

    LasWriter& operator=(const LasWriter&) = delete;
    LasWriter(const LasWriter&) = delete;
    LasWriter(const LasWriter&&) = delete;
};

} // namespace pdal
