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

#include <pdal/pdal_features.hpp>

#include <pdal/compression/LazPerfVlrCompression.hpp>

#include "LasWriter.hpp"
#include "private/las/Summary.hpp"
#include "private/las/Utils.hpp"
#include "private/las/Vlr.hpp"

#include <climits>
#include <iostream>
#include <vector>

#include <pdal/pdal_features.hpp>
#include <pdal/DimUtil.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/Algorithm.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Inserter.hpp>
#include <pdal/util/OStream.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "private/las/Geotiff.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "writers.las",
    "ASPRS LAS 1.0 - 1.4 writer",
    "http://pdal.io/stages/writers.las.html",
    { "las", "laz" }
};

CREATE_STATIC_STAGE(LasWriter, s_info)

std::string LasWriter::getName() const { return s_info.name; }

struct LasWriter::Options
{
    SpatialReference aSrs;
    las::Compression compression;
    bool discardHighReturnNumbers;
    StringList extraDimSpec;
    StringList forwardSpec;
    bool writePDALMetadata;
    NumHeaderVal<uint16_t, 0, 65535> filesourceId;
    NumHeaderVal<uint8_t, 1, 1> majorVersion;
    NumHeaderVal<uint8_t, 1, 4> minorVersion;
    NumHeaderVal<uint8_t, 0, 10> dataformatId;
    NumHeaderVal<uint16_t, 0, 31> globalEncoding;
    UuidHeaderVal projectId;
    StringHeaderVal<32> systemId;
    StringHeaderVal<32> softwareId;
    NumHeaderVal<uint16_t, 0, 366> creationDoy;
    NumHeaderVal<uint16_t, 0, 65535> creationYear;
    StringHeaderVal<0> scaleX;
    StringHeaderVal<0> scaleY;
    StringHeaderVal<0> scaleZ;
    StringHeaderVal<0> offsetX;
    StringHeaderVal<0> offsetY;
    StringHeaderVal<0> offsetZ;
    std::vector<las::Evlr> userVlrs;
    bool enhancedSrsVlrs;
};

struct LasWriter::Private
{
    Options opts;
    las::Header header;
    bool forwardVlrs = false;
    std::string curFilename;
    las::Summary summary;
};

LasWriter::LasWriter() : d(new Private), m_compressor(nullptr), m_ostream(NULL), m_srsCnt(0)
{}


LasWriter::~LasWriter()
{
    delete m_compressor;
}


void LasWriter::addArgs(ProgramArgs& args)
{
    std::time_t now;
    std::time(&now);

    uint16_t year = 1900;
    uint16_t doy = 1;
    std::tm *ptm = std::gmtime(&now);
    if (ptm)
    {
        year += ptm->tm_year;
        doy += ptm->tm_yday;
    }

    args.add("a_srs", "Spatial reference to use to write output", d->opts.aSrs);
    args.add("compression", "Use LAZ compression when writing file", d->opts.compression,
        las::Compression::False);
    args.add("discard_high_return_numbers", "Discard points with out-of-spec "
        "return numbers.", d->opts.discardHighReturnNumbers);
    args.add("extra_dims", "List of dimension names to write in addition to those of the "
        "point format or 'all' for all available dimensions", d->opts.extraDimSpec);
    args.add("forward", "Dimensions to forward from LAS reader", d->opts.forwardSpec);

    args.add("filesource_id", "File source ID number.", d->opts.filesourceId,
        decltype(d->opts.filesourceId)(0));
    args.add("major_version", "LAS major version", d->opts.majorVersion,
        decltype(d->opts.majorVersion)(1));
    args.add("minor_version", "LAS minor version", d->opts.minorVersion,
        decltype(d->opts.minorVersion)(4));
    args.add("dataformat_id", "Point format", d->opts.dataformatId,
        decltype(d->opts.dataformatId)(7));
    args.add("format", "Point format", d->opts.dataformatId,
        decltype(d->opts.dataformatId)(7));
    args.add("global_encoding", "Global encoding byte", d->opts.globalEncoding,
        decltype(d->opts.globalEncoding)(0));
    args.add("project_id", "Project ID", d->opts.projectId);
    args.add("system_id", "System ID", d->opts.systemId,
        decltype(d->opts.systemId)(d->header.systemId));
    args.add("software_id", "Software ID", d->opts.softwareId,
        decltype(d->opts.softwareId)(las::generateSoftwareId()));
    args.add("creation_doy", "Creation day of year", d->opts.creationDoy,
        decltype(d->opts.creationDoy)(doy));
    args.add("creation_year", "Creation year", d->opts.creationYear,
        decltype(d->opts.creationYear)(year));
    args.add("pdal_metadata", "Write PDAL metadata as VLR?", d->opts.writePDALMetadata,
        decltype(d->opts.writePDALMetadata)(false));
    args.add("scale_x", "X scale factor", d->opts.scaleX, decltype(d->opts.scaleX)(".01"));
    args.add("scale_y", "Y scale factor", d->opts.scaleY, decltype(d->opts.scaleY)(".01"));
    args.add("scale_z", "Z scale factor", d->opts.scaleZ, decltype(d->opts.scaleZ)(".01"));
    args.add("offset_x", "X offset", d->opts.offsetX);
    args.add("offset_y", "Y offset", d->opts.offsetY);
    args.add("offset_z", "Z offset", d->opts.offsetZ);
    args.add("vlrs", "List of VLRs to set", d->opts.userVlrs);
    args.add("enhanced_srs_vlrs", "Write WKT2 and PROJJSON as VLR?", d->opts.enhancedSrsVlrs,
        decltype(d->opts.enhancedSrsVlrs)(false));
}

void LasWriter::initialize()
{
    std::string ext = FileUtils::extension(filename());
    ext = Utils::tolower(ext);
    if (ext == ".laz")
        d->opts.compression = las::Compression::True;

    if (!d->opts.aSrs.empty())
        setSpatialReference(d->opts.aSrs);
    if (d->opts.compression == las::Compression::True)
        d->header.setDataCompressed();

    try
    {
        m_extraDims = las::parse(d->opts.extraDimSpec, true);
    }
    catch (const las::error& err)
    {
        throwError(err.what());
    }
    fillForwardList();
}


bool LasWriter::srsOverridden() const
{
    return d->opts.aSrs.valid();
}


void LasWriter::spatialReferenceChanged(const SpatialReference&)
{
    if (++m_srsCnt > 1 && d->opts.aSrs.empty())
        log()->get(LogLevel::Error) << getName() <<
            ": Attempting to write '" << filename() << "' with multiple "
            "point spatial references." << std::endl;
}


void LasWriter::prepared(PointTableRef table)
{
    FlexWriter::validateFilename(table);

    PointLayoutPtr layout = table.layout();

    // Make sure the dataformatID is set so that we can get the proper
    // dimensions being written as part of the standard LAS record.
    fillHeader();

    // If we've asked for all dimensions, add to extraDims all dimensions
    // in the layout that aren't already destined for LAS output.
    if (m_extraDims.size() == 1 && m_extraDims[0].m_name == "all")
    {
        m_extraDims.clear();
        Dimension::IdList ids = las::pdrfDims(d->header.pointFormat());
        DimTypeList dimTypes = layout->dimTypes();
        int byteOffset = 0;
        for (auto& dt : dimTypes)
            if (!Utils::contains(ids, dt.m_id))
            {
                m_extraDims.push_back(
                    las::ExtraDim(layout->dimName(dt.m_id), dt.m_type, byteOffset));
                byteOffset += Dimension::size(dt.m_type);
            }
    }

    m_extraByteLen = 0;
    for (auto& dim : m_extraDims)
    {
        dim.m_dimType.m_id = table.layout()->findDim(dim.m_name);
        if (dim.m_dimType.m_id == Dimension::Id::Unknown)
            throwError("Dimension '" + dim.m_name + "' specified in "
                "'extra_dim' option not found.");
        m_extraByteLen += Dimension::size(dim.m_dimType.m_type);
        log()->get(LogLevel::Info) << getName() << ": Writing dimension " <<
            dim.m_name <<
            "(" << Dimension::interpretationName(dim.m_dimType.m_type) <<
            ") " << " to LAS extra bytes." << std::endl;
    }
}


// Capture user-specified VLRs
void LasWriter::addUserVlrs(MetadataNode m)
{
    for (las::Evlr& v : d->opts.userVlrs)
    {
        v.fillData(m);
        addVlr(v);
    }
}


// Get header info from options and store in map for processing with
// metadata.
void LasWriter::fillForwardList()
{
    static const StringList header = {
        "dataformat_id", "major_version", "minor_version", "filesource_id",
        "global_encoding", "project_id", "system_id", "software_id",
        "creation_doy", "creation_year"
    };

    static const StringList scale = { "scale_x", "scale_y", "scale_z" };

    static const StringList offset = { "offset_x", "offset_y", "offset_z" };

    // Build the forward list, replacing special keywords with the proper
    // field names.
    for (auto& name : d->opts.forwardSpec)
    {
        if (name == "all")
        {
            m_forwards.insert(header.begin(), header.end());
            m_forwards.insert(scale.begin(), scale.end());
            m_forwards.insert(offset.begin(), offset.end());
            d->forwardVlrs = true;
        }
        else if (name == "header")
            m_forwards.insert(header.begin(), header.end());
        else if (name == "scale")
            m_forwards.insert(scale.begin(), scale.end());
        else if (name == "offset")
            m_forwards.insert(offset.begin(), offset.end());
        else if (name == "format")
            m_forwards.insert("dataformat_id");
        else if (name == "vlr")
            d->forwardVlrs = true;
        else if (
            Utils::contains(header, name) ||
            Utils::contains(scale, name) ||
            Utils::contains(offset, name)
        )
            m_forwards.insert(name);
        else
            throwError("Error in 'forward' option.  Unknown field for "
                "forwarding: '" + name + "'.");
    }
}


void LasWriter::readyTable(PointTableRef table)
{
    m_firstPoint = true;
    m_forwardMetadata = table.privateMetadata("lasforward");
    MetadataNode m = table.metadata();
    if(d->opts.writePDALMetadata)
    {
        addMetadataVlr(m);
        addPipelineVlr();
    }
    addExtraBytesVlr();
    addUserVlrs(m);
    addForwardVlrs();
}


void LasWriter::readyFile(const std::string& filename, const SpatialReference& srs)
{
    std::ostream *out = Utils::createFile(filename, true);
    if (!out)
        throwError("Couldn't open file '" + filename + "' for output.");
    d->curFilename = filename;
    Utils::writeProgress(m_progressFd, "READYFILE", filename);
    prepOutput(out, srs);
}


void LasWriter::prepOutput(std::ostream *outStream, const SpatialReference& srs)
{
    // Use stage SRS if provided.
    m_srs = getSpatialReference().empty() ? srs : getSpatialReference();

    handleHeaderForwards(m_forwardMetadata);

    // Filling the header here gives the VLR functions below easy access to
    // the version information and so on.
    fillHeader();

    // Spatial reference can potentially change for multiple output files.
    addSpatialRefVlrs();

    d->summary.clear();
    m_ostream = outStream;
    if (d->header.dataCompressed())
        readyCompression();

    // Compression should cause the last of the VLRs to get filled.  We now
    // have a valid count, so fill the header again.
    fillHeader();

    validateHeader();

    // Write the header.
    std::vector<char> headerBuf = d->header.data();
    m_ostream->write(headerBuf.data(), headerBuf.size());
    d->header.vlrOffset = (uint32_t)m_ostream->tellp();

    for (const las::Vlr& vlr : m_vlrs)
    {
        std::vector<char> buf = vlr.headerData();
        m_ostream->write(buf.data(), buf.size());
        m_ostream->write(vlr.data(), vlr.dataSize());
    }
    d->header.pointOffset = (uint32_t)m_ostream->tellp();

    // Set the point buffer size here in case we're using the streaming
    // interface.
    m_pointBuf.resize(d->header.pointSize);
}


/// Search for metadata associated with the provided recordId and userId.
/// \param  node - Top-level node to use for metadata search.
/// \param  recordId - Record ID to match.
/// \param  userId - User ID to match.
MetadataNode LasWriter::findVlrMetadata(MetadataNode node,
    uint16_t recordId, const std::string& userId)
{
    std::string sRecordId = std::to_string(recordId);

    // Find a node whose name starts with vlr and that has child nodes
    // with the name and recordId we're looking for.
    auto pred = [sRecordId,userId](MetadataNode n)
    {
        auto recPred = [sRecordId](MetadataNode n)
        {
            return n.name() == "record_id" &&
                n.value() == sRecordId;
        };
        auto userPred = [userId](MetadataNode n)
        {
            return n.name() == "user_id" &&
                n.value() == userId;
        };
        return (Utils::startsWith(n.name(), "vlr") &&
            !n.findChild(recPred).empty() &&
            !n.findChild(userPred).empty());
    };
    return node.find(pred);
}


void LasWriter::addMetadataVlr(MetadataNode& forward)
{
    std::string json = Utils::toJSON(forward);
    if ((json.size() > las::Vlr::MaxDataSize) && !d->header.versionAtLeast(1, 4))
    {
        log()->get(LogLevel::Debug) << "pdal metadata VLR too large "
            "to write in VLR for files < LAS 1.4";
    }
    else
    {
        std::vector<char> data(json.begin(), json.end());
        addVlr(las::PdalUserId, las::PdalMetadataRecordId, "PDAL metadata", data);
    }
}

void LasWriter::addPipelineVlr()
{
    // Write a VLR with the PDAL pipeline.
    std::ostringstream ostr;
    PipelineWriter::writePipeline(this, ostr);
    std::string json = ostr.str();
    if (json.size() > las::Vlr::MaxDataSize && !d->header.versionAtLeast(1, 4))
    {
        log()->get(LogLevel::Debug) << "pdal pipeline VLR too large "
            "to write in VLR for files < LAS 1.4";
    }
    else
    {
        std::vector<char> data(json.begin(), json.end());
        addVlr(las::PdalUserId, las::PdalPipelineRecordId, "PDAL pipeline", data);
    }
}


/// Add VLRs forwarded from the input.
void LasWriter::addForwardVlrs()
{
    if (!d->forwardVlrs)
        return;

    auto pred = [](MetadataNode n)
        { return Utils::startsWith(n.name(), "vlr_"); };

    MetadataNodeList nodes = m_forwardMetadata.findChildren(pred);
    for (auto& n : nodes)
    {
        const MetadataNode& userIdNode = n.findChild("user_id");
        const MetadataNode& recordIdNode = n.findChild("record_id");
        if (recordIdNode.valid() && userIdNode.valid())
        {
            const MetadataNode& dataNode = n.findChild("data");
            uint16_t recordId = (uint16_t)std::stoi(recordIdNode.value());
            addVlr(userIdNode.value(), recordId, n.description(),
                Utils::base64_decode(dataNode.value()));
        }
    }
}

/// Set VLRs from the active spatial reference.
/// \param  srs - Active spatial reference.
void LasWriter::addSpatialRefVlrs()
{
    // Delete any existing spatial ref VLRs.  This can be an issue if we're
    // using the reader to write multiple output files via a filename template.
    deleteVlr(las::TransformUserId, las::GeotiffDirectoryRecordId);
    deleteVlr(las::TransformUserId, las::GeotiffDoublesRecordId);
    deleteVlr(las::TransformUserId, las::GeotiffAsciiRecordId);
    deleteVlr(las::TransformUserId, las::WktRecordId);
    deleteVlr(las::LiblasUserId, las::WktRecordId);

    if (d->header.versionAtLeast(1, 4))
        addWktVlr();
    else
        addGeotiffVlrs();
}

void LasWriter::addGeotiffVlrs()
{
    if (m_srs.empty())
        return;

    try
    {
        las::GeotiffTags tags(m_srs);

        if (tags.directoryData().empty())
            throwError("Invalid spatial reference for writing GeoTiff VLR.");

        addVlr(las::TransformUserId, las::GeotiffDirectoryRecordId,
                "GeoTiff GeoKeyDirectoryTag", tags.directoryData());
        if (tags.doublesData().size())
            addVlr(las::TransformUserId, las::GeotiffDoublesRecordId,
                "GeoTiff GeoDoubleParamsTag", tags.doublesData());
        if (tags.asciiData().size())
            addVlr(las::TransformUserId, las::GeotiffAsciiRecordId,
                "GeoTiff GeoAsciiParamsTag", tags.asciiData());
    }
    catch (las::Geotiff::error& err)
    {
        throwError(err.what());
    }
}


/// Add a Well-known Text VLR associated with the spatial reference.
/// \return  Whether the VLR was added.
bool LasWriter::addWktVlr()
{
    // WKT2 and PROJJSON can be writen in PDAL VLRs
    if (d->opts.enhancedSrsVlrs) {
        const std::string wkt2 = m_srs.getWKT2();
        if (!wkt2.empty()) {
            std::vector<char> wktBytes(wkt2.begin(), wkt2.end());
            wktBytes.resize(wktBytes.size() + 1, 0);
            addVlr(las::TransformUserId, las::LASFWkt2recordId, "PDAL WKT2 Record", wktBytes);
        }

        const std::string projjson = m_srs.getPROJJSON();
        if (!projjson.empty()) {
            std::vector<char> wktBytes(projjson.begin(), projjson.end());
            wktBytes.resize(wktBytes.size() + 1, 0);
            addVlr(las::PdalUserId, las::PdalProjJsonRecordId, "PDAL PROJJSON Record", wktBytes);
        }
    }

    // LAS 1.4 requires WKTv1
    std::string wkt;
    try
    {
        wkt = m_srs.getWKT1();
    }
    catch(const std::exception&)
    {
        if (!d->opts.enhancedSrsVlrs)
            throw;
    }

    if (wkt.empty())
        return false;

    std::vector<char> wktBytes(wkt.begin(), wkt.end());
    // This tacks a NULL to the end of the data, which is required by the spec.
    wktBytes.resize(wktBytes.size() + 1, 0);
    addVlr(las::TransformUserId, las::WktRecordId, "OGC Transformation Record", wktBytes);

    // The data in the vector gets moved to the VLR, so we have to recreate it.
    std::vector<char> wktBytes2(wkt.begin(), wkt.end());
    wktBytes2.resize(wktBytes2.size() + 1, 0);
    addVlr(las::LiblasUserId, las::WktRecordId, "OGR variant of OpenGIS WKT SRS", wktBytes2);
    return true;
}


void LasWriter::addExtraBytesVlr()
{
    if (m_extraDims.empty())
        return;

    std::vector<char> ebBytes;
    for (auto& dim : m_extraDims)
    {
        las::ExtraBytesIf eb(dim.m_name, dim.m_dimType.m_type,
            Dimension::description(dim.m_dimType.m_id));
        eb.appendTo(ebBytes);
    }

    addVlr(las::SpecUserId, las::ExtraBytesRecordId, "Extra Bytes Record", ebBytes);
}


/// Add a standard or extended VLR depending on the data size.
/// \param  userId - VLR user ID
/// \param  recordId - VLR record ID
/// \param  description - VLR description
/// \param  data - Raw VLR data
void LasWriter::addVlr(const std::string& userId, uint16_t recordId,
   const std::string& description, const std::vector<uint8_t>& data)
{
    std::vector<char> v((const char *)data.data(), (const char *)(data.data() + data.size()));
    las::Evlr vlr(userId, recordId, description, std::move(v));
    addVlr(vlr);
}

/// Add a standard or variable-length VLR depending on the data size.
/// \param  userId - VLR user ID
/// \param  recordId - VLR record ID
/// \param  description - VLR description
/// \param  data - Raw VLR data
void LasWriter::addVlr(const std::string& userId, uint16_t recordId,
   const std::string& description, const std::vector<char>& data)
{
    las::Evlr vlr(userId, recordId, description, data);
    addVlr(vlr);
}

/// Add a standard or variable-length VLR depending on the data size.
/// \param  userId - VLR user ID
/// \param  recordId - VLR record ID
/// \param  description - VLR description
/// \param  data - Raw VLR data
void LasWriter::addVlr(const std::string& userId, uint16_t recordId,
   const std::string& description, std::vector<char>&& data)
{
    las::Evlr vlr(userId, recordId, description, data);
    addVlr(vlr);
}

/// Add a standard or variable-length VLR depending on the data size.
/// \param  evlr  VLR to add.
void LasWriter::addVlr(const las::Evlr& evlr)
{
    if (evlr.dataSize() > las::Vlr::MaxDataSize)
    {
        if (d->header.versionAtLeast(1, 4))
            m_evlrs.push_back(std::move(evlr));
        else
            throwError("Can't write VLR with user ID/record ID = " +
                evlr.userId + "/" + std::to_string(evlr.recordId) +
                ".  The data size exceeds the maximum supported.");
    } else if (evlr.writeAsEVLR)
    {
        if (d->header.versionAtLeast(1, 4))
            m_evlrs.push_back(std::move(evlr));
        else
            throwError("User specified writing as EVLR but the file is not a 1.4+ file!");
    }
    else
        m_vlrs.push_back(std::move(evlr));
}

/// Delete a VLR from the vlr list.
///
void LasWriter::deleteVlr(const std::string& userId, uint16_t recordId)
{
    las::Vlr v{userId, recordId};

    Utils::remove(m_vlrs, v);
    Utils::remove(m_evlrs, v);
}


template <typename T>
void LasWriter::handleHeaderForward(const std::string& s, T& headerVal, const MetadataNode& base)
{
    if (Utils::contains(m_forwards, s) && !headerVal.valSet())
    {
        MetadataNode invalid = base.findChild(s + "INVALID");
        MetadataNode m = base.findChild(s);
        if (!invalid.valid() && m.valid())
            headerVal.setVal(m.value<typename T::type>());
    }
}


void LasWriter::handleHeaderForwards(MetadataNode& forward)
{
    handleHeaderForward("major_version", d->opts.majorVersion, forward);
    handleHeaderForward("minor_version", d->opts.minorVersion, forward);
    handleHeaderForward("dataformat_id", d->opts.dataformatId, forward);
    handleHeaderForward("filesource_id", d->opts.filesourceId, forward);
    handleHeaderForward("global_encoding", d->opts.globalEncoding, forward);
    handleHeaderForward("project_id", d->opts.projectId, forward);
    handleHeaderForward("system_id", d->opts.systemId, forward);
    handleHeaderForward("software_id", d->opts.softwareId, forward);
    handleHeaderForward("creation_doy", d->opts.creationDoy, forward);
    handleHeaderForward("creation_year", d->opts.creationYear, forward);

    handleHeaderForward("scale_x", d->opts.scaleX, forward);
    handleHeaderForward("scale_y", d->opts.scaleY, forward);
    handleHeaderForward("scale_z", d->opts.scaleZ, forward);
    handleHeaderForward("offset_x", d->opts.offsetX, forward);
    handleHeaderForward("offset_y", d->opts.offsetY, forward);
    handleHeaderForward("offset_z", d->opts.offsetZ, forward);

    m_scaling.m_xXform.m_scale.set(d->opts.scaleX.val());
    m_scaling.m_yXform.m_scale.set(d->opts.scaleY.val());
    m_scaling.m_zXform.m_scale.set(d->opts.scaleZ.val());
    m_scaling.m_xXform.m_offset.set(d->opts.offsetX.val());
    m_scaling.m_yXform.m_offset.set(d->opts.offsetY.val());
    m_scaling.m_zXform.m_offset.set(d->opts.offsetZ.val());
}

/// Fill the LAS header with values as provided in options or forwarded
/// metadata.
void LasWriter::fillHeader()
{
    const uint16_t WKT_MASK = (1 << 4);

    d->header.scale.x = m_scaling.m_xXform.m_scale.m_val;
    d->header.scale.y = m_scaling.m_yXform.m_scale.m_val;
    d->header.scale.z = m_scaling.m_zXform.m_scale.m_val;
    d->header.offset.x = m_scaling.m_xXform.m_offset.m_val;
    d->header.offset.y = m_scaling.m_yXform.m_offset.m_val;
    d->header.offset.z = m_scaling.m_zXform.m_offset.m_val;
    d->header.vlrCount = m_vlrs.size();
    d->header.evlrCount = m_evlrs.size();
    d->header.setPointFormat(d->opts.dataformatId.val());
    d->header.pointSize = d->header.baseCount() + m_extraByteLen;
    d->header.versionMinor = d->opts.minorVersion.val();
    d->header.creationDoy = d->opts.creationDoy.val();
    d->header.creationYear = d->opts.creationYear.val();
    d->header.softwareId = d->opts.softwareId.val();
    d->header.systemId = d->opts.systemId.val();
    d->header.projectGuid = d->opts.projectId.val();
    d->header.fileSourceId = d->opts.filesourceId.val();

    // We always write a WKT VLR for version 1.4 and later.
    d->header.globalEncoding = d->opts.globalEncoding.val();
    if (d->header.versionAtLeast(1, 4))
        d->header.globalEncoding |= las::Header::WktMask;
}

void LasWriter::validateHeader()
{
    if (m_scaling.m_xXform.m_scale.m_val == 0.0)
        throwError("X scale of 0.0 is invalid.");
    if (m_scaling.m_yXform.m_scale.m_val == 0.0)
        throwError("Y scale of 0.0 is invalid.");
    if (m_scaling.m_zXform.m_scale.m_val == 0.0)
        throwError("Z scale of 0.0 is invalid.");
    if (d->header.hasWave())
        throwError("PDAL does not support point formats with waveform data (4, 5, 9 and 10)");
    if (d->header.versionAtLeast(1, 4))
    {
        if (d->header.pointFormat() > 10)
            throwError("LAS version 1." + std::to_string(d->header.versionMinor) +
                " only supports point formats 0-10.");
    }
    else if (d->header.pointFormat() > 5)
        throwError("LAS version 1." + std::to_string(d->header.versionMinor) +
            " only supports point formats 0-5.");
}

void LasWriter::readyCompression()
{
    deleteVlr(las::LaszipUserId, las::LaszipRecordId);
    m_compressor = new LazPerfVlrCompressor(*m_ostream, d->header.pointFormat(),
        d->header.ebCount());
    std::vector<char> lazVlrData = m_compressor->vlrData();
    std::vector<char> vlrdata(lazVlrData.begin(), lazVlrData.end());
    addVlr(las::LaszipUserId, las::LaszipRecordId, "http://laszip.org", vlrdata);
}


// This is only called in stream mode.
bool LasWriter::processOne(PointRef& point)
{
    if (m_firstPoint)
    {
        auto doScale = [this](const XForm::XFormComponent& scale,
            const std::string& name)
        {
            if (scale.m_auto)
                log()->get(LogLevel::Warning) << "Auto scale for " << name <<
                " requested in stream mode.  Using value of 1.0." << std::endl;
        };

        doScale(m_scaling.m_xXform.m_scale, "X");
        doScale(m_scaling.m_yXform.m_scale, "Y");
        doScale(m_scaling.m_zXform.m_scale, "Z");

        auto doOffset = [this](XForm::XFormComponent& offset, double val,
            const std::string name)
        {
            if (offset.m_auto)
            {
                offset.m_val = val;
                log()->get(LogLevel::Warning) << "Auto offset for '" << name <<
                    "' requested in stream mode.  Using value of " <<
                    offset.m_val << "." << std::endl;
            }
        };

        doOffset(m_scaling.m_xXform.m_offset, point.getFieldAs<double>(Dimension::Id::X), "X");
        doOffset(m_scaling.m_yXform.m_offset, point.getFieldAs<double>(Dimension::Id::Y), "Y");
        doOffset(m_scaling.m_zXform.m_offset, point.getFieldAs<double>(Dimension::Id::Z), "Z");
        m_firstPoint = false;
    }
    return processPoint(point);
}


// This is separated from processOne so that we're sure when processOne is
// called we know we're in stream mode.
bool LasWriter::processPoint(PointRef& point)
{
    if (d->opts.compression == las::Compression::True)
    {
        LeInserter ostream(m_pointBuf.data(), m_pointBuf.size());
        if (!fillPointBuf(point, ostream))
            return false;
        writeLazPerfBuf(m_pointBuf.data(), d->header.pointSize, 1);
    }
    else
    {
        LeInserter ostream(m_pointBuf.data(), d->header.pointSize);
        if (!fillPointBuf(point, ostream))
            return false;
        m_ostream->write(m_pointBuf.data(), d->header.pointSize);
    }
    return true;
}


void LasWriter::prerunFile(const PointViewSet& pvSet)
{
    m_scaling.setAutoXForm(pvSet);
}


void LasWriter::writeView(const PointViewPtr view)
{
    Utils::writeProgress(m_progressFd, "READYVIEW", std::to_string(view->size()));

    point_count_t pointLen = d->header.pointSize;

    // Make a buffer of at most a meg.
    m_pointBuf.resize((std::min)((point_count_t)1000000, pointLen * view->size()));

    const PointView& viewRef(*view.get());
    point_count_t remaining = view->size();
    PointId idx = 0;
    while (remaining)
    {
        point_count_t filled = fillWriteBuf(viewRef, idx, m_pointBuf);
        idx += filled;
        remaining -= filled;

        if (d->opts.compression == las::Compression::True)
            writeLazPerfBuf(m_pointBuf.data(), pointLen, filled);
        else
            m_ostream->write(m_pointBuf.data(), filled * pointLen);
    }
    Utils::writeProgress(m_progressFd, "DONEVIEW", std::to_string(view->size()));
}


void LasWriter::writeLazPerfBuf(char *pos, size_t pointLen, point_count_t numPts)
{
    for (point_count_t i = 0; i < numPts; i++)
    {
        m_compressor->compress(pos);
        pos += pointLen;
    }
}


bool LasWriter::fillPointBuf(PointRef& point, LeInserter& ostream)
{
    bool has14PointFormat = d->header.has14PointFormat();
    static const size_t maxReturnCount = d->header.maxReturnCount();

    // we always write the base fields
    using namespace Dimension;

    uint8_t returnNumber(1);
    uint8_t numberOfReturns(1);
    if (point.hasDim(Id::ReturnNumber))
        returnNumber = point.getFieldAs<uint8_t>(Id::ReturnNumber);
    if (point.hasDim(Id::NumberOfReturns))
        numberOfReturns = point.getFieldAs<uint8_t>(Id::NumberOfReturns);
    if (numberOfReturns > maxReturnCount)
    {
        if (d->opts.discardHighReturnNumbers)
        {
            // If this return number is too high, pitch the point.
            if (returnNumber > maxReturnCount)
                return false;
            numberOfReturns = maxReturnCount;
        }
    }

    auto converter = [this](double val, Dimension::Id dim) -> int32_t
    {
        int32_t i(0);

        if (!Utils::numericCast(val, i))
            throwError("Unable to convert scaled value (" +
                Utils::toString(val) + ") to "
                "int32 for dimension '" + Dimension::name(dim) +
                "' when writing LAS/LAZ file " + d->curFilename + ".");
        return i;
    };

    double xOrig = point.getFieldAs<double>(Id::X);
    double yOrig = point.getFieldAs<double>(Id::Y);
    double zOrig = point.getFieldAs<double>(Id::Z);
    int32_t x = converter(m_scaling.m_xXform.toScaled(xOrig), Id::X);
    int32_t y = converter(m_scaling.m_yXform.toScaled(yOrig), Id::Y);
    int32_t z = converter(m_scaling.m_zXform.toScaled(zOrig), Id::Z);

    ostream << x;
    ostream << y;
    ostream << z;

    uint16_t intensity(0);
    if (point.hasDim(Id::Intensity))
        intensity = point.getFieldAs<uint16_t>(Id::Intensity);
    ostream << intensity;

    uint8_t scanChannel(0);
    if (point.hasDim(Id::ScanChannel))
        scanChannel = point.getFieldAs<uint8_t>(Id::ScanChannel);

    uint8_t scanDirectionFlag(0);
    if (point.hasDim(Id::ScanDirectionFlag))
        scanDirectionFlag = point.getFieldAs<uint8_t>(Id::ScanDirectionFlag);

    uint8_t edgeOfFlightLine(0);
    if (point.hasDim(Id::EdgeOfFlightLine))
        edgeOfFlightLine = point.getFieldAs<uint8_t>(Id::EdgeOfFlightLine);

    uint8_t classification(0);
    if (point.hasDim(Id::Classification))
        classification = point.getFieldAs<uint8_t>(Id::Classification);

    uint8_t synthetic(0);
    if (point.hasDim(Id::Synthetic))
        synthetic = point.getFieldAs<uint8_t>(Id::Synthetic);
    uint8_t keypoint(0);
    if (point.hasDim(Id::KeyPoint))
        keypoint = point.getFieldAs<uint8_t>(Id::KeyPoint);
    uint8_t withheld(0);
    if (point.hasDim(Id::Withheld))
        withheld = point.getFieldAs<uint8_t>(Id::Withheld);
    uint8_t overlap(0);
    if (point.hasDim(Id::Overlap))
        overlap = point.getFieldAs<uint8_t>(Id::Overlap);

    if (has14PointFormat)
    {
        uint8_t returnbits = returnNumber | (numberOfReturns << 4);
        ostream << returnbits;

        uint8_t otherbits =
            ((synthetic & 0x01) << 0) |
            ((keypoint & 0x01) << 1) |
            ((withheld & 0x01) << 2) |
            ((overlap & 0x01) << 3) |
            ((scanChannel & 0x03) << 4) |
            ((scanDirectionFlag & 0x01) << 6) |
            ((edgeOfFlightLine & 0x01) << 7);
        ostream << otherbits << classification;
    }
    else
    {
        uint8_t bits = returnNumber | (numberOfReturns << 3) |
            (scanDirectionFlag << 6) | (edgeOfFlightLine << 7);
        ostream << bits;

        if (overlap)
        {
            // In the V10 PDRFs, we do not have a dedicated Overlap bit, instead
            // this was encoded as Classification=12.
            if (classification == ClassLabel::CreatedNeverClassified)
            {
                // If the Overlap flag is set and the point is marked as "Never
                // Classified", then set Classification=12 to mark the point as
                // Overlap.
                classification = ClassLabel::LegacyOverlap;
            }
            else
            {
                // Source file is PDRF 6+, which supports a dedicated Overlap
                // bit which can be set independently of Classification, but
                // we're writing to PDRF < 6 and can't support Overlap=1
                // alongside another Classification.  Keep the Classification
                // and we'll lose the Overlap bit.
                log()->get(LogLevel::Warning)
                    << "Point is marked as Overlap but also has a Classification - "
                    << "ignoring overlap for LAS "
                    << std::to_string(d->header.versionMajor) << "."
                    << std::to_string(d->header.versionMinor) << "." << std::endl;
            }
        }

        if (classification > 31)
        {
            // Source file is PDRF 6+, which supports classification values up
            // to 255, but we're writing to PDRF < 6 and can't write values
            // over 31.
            log()->get(LogLevel::Warning)
                << "The source file Classification " << (int)classification
                << " can't be written to LAS with Point Data Record Format "
                << std::to_string(d->header.pointFormat()) << " and LAS version "
                << std::to_string(d->header.versionMajor) << "."
                << std::to_string(d->header.versionMinor)
                << ". It was replaced with value 1. "
                "Use a different PDRF if you need to support classifications greater than 31" << std::endl;
            classification = ClassLabel::Unclassified;
        }

        uint8_t classificationWithFlags =
            (classification & 0x1F) |
            ((synthetic & 0x01) << 5) |
            ((keypoint & 0x01) << 6) |
            ((withheld & 0x01) << 7);

        ostream << classificationWithFlags;
    }

    uint8_t userData(0);
    if (point.hasDim(Id::UserData))
        userData = point.getFieldAs<uint8_t>(Id::UserData);

    if (has14PointFormat)
    {
         // Guaranteed to fit if scan angle rank isn't wonky.
        int16_t scanAngleRank(0);
        if (point.hasDim(Id::ScanAngleRank) )
            scanAngleRank =
                static_cast<int16_t>(std::round(
                    point.getFieldAs<float>(Id::ScanAngleRank) / .006f));
        ostream << userData << scanAngleRank;
    }
    else
    {
        int8_t scanAngleRank(0);
        if (point.hasDim(Id::ScanAngleRank) )
            scanAngleRank = point.getFieldAs<int8_t>(Id::ScanAngleRank);
        ostream << scanAngleRank << userData;
    }

    uint16_t pointSourceId(0);
    if (point.hasDim(Id::PointSourceId))
        pointSourceId = point.getFieldAs<uint16_t>(Id::PointSourceId);
    ostream << pointSourceId;

    if (d->header.hasTime())
        ostream << point.getFieldAs<double>(Id::GpsTime);

    if (d->header.hasColor())
    {
        ostream << point.getFieldAs<uint16_t>(Id::Red);
        ostream << point.getFieldAs<uint16_t>(Id::Green);
        ostream << point.getFieldAs<uint16_t>(Id::Blue);
    }

    if (d->header.hasInfrared())
        ostream << point.getFieldAs<uint16_t>(Id::Infrared);

    Everything e;
    for (auto& dim : m_extraDims)
    {
        point.getField((char *)&e, dim.m_dimType.m_id, dim.m_dimType.m_type);
        Utils::insertDim(ostream, dim.m_dimType.m_type, e);
    }

    double xConverted = m_scaling.m_xXform.fromScaled(x);
    double yConverted = m_scaling.m_yXform.fromScaled(y);
    double zConverted = m_scaling.m_zXform.fromScaled(z);
    d->summary.addPoint(xConverted, yConverted, zConverted, returnNumber);
    return true;
}


point_count_t LasWriter::fillWriteBuf(const PointView& view,
    PointId startId, std::vector<char>& buf)
{
    point_count_t blocksize = buf.size() / d->header.pointSize;
    blocksize = (std::min)(blocksize, view.size() - startId);
    PointId lastId = startId + blocksize;

    LeInserter ostream(buf.data(), buf.size());
    PointRef point = (const_cast<PointView&>(view)).point(0);
    for (PointId idx = startId; idx < lastId; idx++)
    {
        point.setPointId(idx);
        fillPointBuf(point, ostream);
    }
    return blocksize;
}


void LasWriter::doneFile()
{
    finishOutput();
    Utils::writeProgress(m_progressFd, "DONEFILE", d->curFilename);
    getMetadata().addList("filename", d->curFilename);
    delete m_ostream;
    m_ostream = NULL;
}


void LasWriter::finishOutput()
{
    if (d->opts.compression == las::Compression::True)
        finishLazPerfOutput();
    log()->get(LogLevel::Debug) << "Wrote " << (int)d->summary.getTotalNumPoints() <<
        " points to the LAS file" << std::endl;

    // addVlr prevents any evlrs from being added before version 1.4.
    d->header.evlrOffset = (m_evlrs.size() ? (uint32_t)m_ostream->tellp() : 0);
    for (const las::Evlr& evlr : m_evlrs)
    {
        std::vector<char> buf = evlr.headerData();
        m_ostream->write((const char *)buf.data(), buf.size());
        m_ostream->write((const char *)evlr.data(), evlr.dataSize());
    }

    // Refill the header since the offset/scale may have been auto-computed.
    fillHeader();

    // The summary is calculated as points are written.
    las::setSummary(d->header, d->summary);

    std::vector<char> buf = d->header.data();
    m_ostream->seekp(0);
    m_ostream->write(buf.data(), buf.size());
    //ABELL - This seemingly unnecessary line may have been necessary for NITF or LASzip.
    m_ostream->seekp(d->header.pointOffset);
    m_ostream->flush();
}


void LasWriter::finishLazPerfOutput()
{
    m_compressor->done();
}

const las::Header& LasWriter::header() const
{
    return d->header;
}

} // namespace pdal
