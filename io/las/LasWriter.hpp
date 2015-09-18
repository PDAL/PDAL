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

#include <pdal/FlexWriter.hpp>

#include "HeaderVal.hpp"
#include "LasError.hpp"
#include "LasHeader.hpp"
#include "LasUtils.hpp"
#include "SummaryData.hpp"
#include "ZipPoint.hpp"

extern "C" int32_t LasWriter_ExitFunc();
extern "C" PF_ExitFunc LasWriter_InitPlugin();

namespace pdal
{
class LasTester;
class NitfWriter;
class GeotiffSupport;

struct VlrOptionInfo
{
    std::string m_name;
    std::string m_value;
    std::string m_userId;
    uint16_t m_recordId;
    std::string m_description;
};

class PDAL_DLL LasWriter : public FlexWriter
{
    friend class LasTester;
    friend class NitfWriter;
public:
    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    LasWriter();

    Options getDefaultOptions();

protected:
    void prepOutput(std::ostream *out);
    void finishOutput();

private:
    LasError m_error;
    LasHeader m_lasHeader;
    std::unique_ptr<SummaryData> m_summaryData;
    std::unique_ptr<LASzipper> m_zipper;
    std::unique_ptr<ZipPoint> m_zipPoint;
    bool m_discardHighReturnNumbers;
    std::map<std::string, std::string> m_headerVals;
    std::vector<VlrOptionInfo> m_optionInfos;
    std::ostream *m_ostream;
    std::vector<VariableLengthRecord> m_vlrs;
    std::vector<ExtVariableLengthRecord> m_eVlrs;
    std::vector<ExtraDim> m_extraDims;
    uint16_t m_extraByteLen;
    SpatialReference m_srs;
    std::string m_curFilename;
    std::set<std::string> m_forwards;
    bool m_forwardVlrs;

    NumHeaderVal<uint8_t, 1, 1> m_majorVersion;
    NumHeaderVal<uint8_t, 1, 4> m_minorVersion;
    NumHeaderVal<uint8_t, 0, 10> m_dataformatId;
    // MSVC doesn't see numeric_limits::max() as constexpr do doesn't allow
    // it as defaults for templates.  Remove when possible.
    NumHeaderVal<uint16_t, 0, 65535> m_filesourceId;
    NumHeaderVal<uint16_t, 0, 15> m_globalEncoding;
    UuidHeaderVal m_projectId;
    StringHeaderVal<32> m_systemId;
    StringHeaderVal<32> m_softwareId;
    NumHeaderVal<uint16_t, 0, 366> m_creationDoy;
    // MSVC doesn't see numeric_limits::max() as constexpr do doesn't allow
    // them as defaults for templates.  Remove when possible.
    NumHeaderVal<uint16_t, 0, 65535> m_creationYear;
    StringHeaderVal<20> m_scaleX;
    StringHeaderVal<20> m_scaleY;
    StringHeaderVal<20> m_scaleZ;
    StringHeaderVal<20> m_offsetX;
    StringHeaderVal<20> m_offsetY;
    StringHeaderVal<20> m_offsetZ;

    virtual void processOptions(const Options& options);
    virtual void prepared(PointTableRef table);
    virtual void readyTable(PointTableRef table);
    virtual void readyFile(const std::string& filename);
    virtual void writeView(const PointViewPtr view);
    virtual void doneFile();

    void fillForwardList(const Options& options);
    void getHeaderOptions(const Options& options);
    void getVlrOptions(const Options& opts);
    template <typename T>
    void handleForward(const std::string& s, T& headerVal,
        const MetadataNode& base);
    void handleForwards(MetadataNode& forward);
    void fillHeader(MetadataNode& forward);
    point_count_t fillWriteBuf(const PointView& view, PointId startId,
        std::vector<char>& buf);
    void setVlrsFromMetadata(MetadataNode& forward);
    MetadataNode findVlrMetadata(MetadataNode node, uint16_t recordId,
        const std::string& userId);
    void setExtraBytesVlr();
    void setVlrsFromSpatialRef();
    void readyCompression();
    void openCompression();
    void addVlr(const std::string& userId, uint16_t recordId,
        const std::string& description, std::vector<uint8_t>& data);
    bool addGeotiffVlr(GeotiffSupport& geotiff, uint16_t recordId,
        const std::string& description);
    bool addWktVlr();

    LasWriter& operator=(const LasWriter&); // not implemented
    LasWriter(const LasWriter&); // not implemented
};

} // namespace pdal
