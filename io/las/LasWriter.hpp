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

#include <pdal/Writer.hpp>

#include "LasError.hpp"
#include "LasHeader.hpp"
#include "SummaryData.hpp"
#include "ZipPoint.hpp"

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

#define LASWRITERDOC "ASPRS LAS 1.0 - 1.4 writer. LASzip support is also \n" \
                     "available if enabled at compile-time. Note that LAZ \n" \
                     "does not provide LAS 1.4 support at this time."
class PDAL_DLL LasWriter : public pdal::Writer
{
    friend class LasTester;
    friend class NitfWriter;
public:
    SET_STAGE_NAME("writers.las", LASWRITERDOC)
    SET_STAGE_LINK("http://pdal.io/stages/writers.las.html")

    LasWriter() : m_ostream(NULL)
         { construct(); }
    LasWriter(std::ostream *stream) : m_ostream(stream)
        { construct(); }

    static Options getDefaultOptions();

    void flush();

private:
    LasError m_error;
    LasHeader m_lasHeader;
    uint32_t m_numPointsWritten;
    SummaryData m_summaryData;
    std::unique_ptr<LASzipper> m_zipper;
    std::unique_ptr<ZipPoint> m_zipPoint;
    bool m_discardHighReturnNumbers;
    std::map<std::string, std::string> m_headerVals;
    std::vector<VlrOptionInfo> m_optionInfos;
    uint64_t m_streamOffset; // the first byte of the LAS file
    std::ostream *m_ostream;
    std::vector<VariableLengthRecord> m_vlrs;
    std::vector<ExtVariableLengthRecord> m_eVlrs;

    virtual void processOptions(const Options& options);
    virtual void ready(PointContextRef ctx);
    virtual void write(const PointBuffer& pointBuffer);
    virtual void done(PointContextRef ctx);

    void construct();
    void getHeaderOptions(const Options& options);
    void getVlrOptions(const Options& opts);
    template<typename T>
    T headerVal(const std::string& name);
    void fillHeader(PointContextRef ctx);
    void setVlrsFromMetadata();
    MetadataNode findVlrMetadata(MetadataNode node, uint16_t recordId,
        const std::string& userId);
    void setVlrsFromSpatialRef(const SpatialReference& srs);
    void readyCompression();
    void openCompression();
    void addVlr(const std::string& userId, uint16_t recordId,
        const std::string& description, std::vector<uint8_t>& data);
    bool addGeotiffVlr(GeotiffSupport& geotiff, uint16_t recordId,
        const std::string& description);
    bool addWktVlr(const SpatialReference& srs);

    LasWriter& operator=(const LasWriter&); // not implemented
    LasWriter(const LasWriter&); // not implemented
};

// Find the approriate value for the specified header field.
/// \param  name - Name of header field.
/// \return  Value of header field.
template<typename T>
T LasWriter::headerVal(const std::string& name)
{
    // The header values either come from options, or are overriden in
    // the metadata for options which had the value FORWARD.  For those,
    // grab the value from metadata if it exists, or use the default value,
    // which was stuck on following the FORWARD value when processing options.
    auto pred = [name](MetadataNode n)
    {
        return n.name() == name;
    };

    std::string val = m_headerVals[name];
    if (val.find("FORWARD") == 0)
    {
        MetadataNode m = m_metadata.findChild(pred);
        if (m.valid())
            return m.value<T>();
        val = val.substr(strlen("FORWARD"));
    }
    try
    {
        return boost::lexical_cast<T>(val);
    }
    catch (boost::bad_lexical_cast ex)
    {
        std::stringstream out;
        out << "Couldn't convert option \"" << name << "\" with value \"" <<
            val << "\" from string as necessary.";
        throw pdal_error(out.str());
    }
}

} // namespace pdal
