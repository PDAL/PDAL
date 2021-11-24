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
#include "private/las/Utils.hpp"

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

#include "GeotiffSupport.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "writers.las",
    "ASPRS LAS 1.0 - 1.4 writer. LASzip support is also \n" \
        "available if enabled at compile-time.",
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
    std::vector<ExtLasVLR> userVLRs;
};

struct LasWriter::Private
{
    Options opts;
};

LasWriter::LasWriter() : d(new Private), m_compressor(nullptr), m_ostream(NULL), m_srsCnt(0)
{}


LasWriter::~LasWriter()
{
#ifdef PDAL_HAVE_LAZPERF
    delete m_compressor;
#endif
}


void LasWriter::addArgs(ProgramArgs& args)
{
    std::time_t now;
    std::time(&now);
    std::tm* ptm = std::gmtime(&now);
    uint16_t year = ptm->tm_year + 1900;
    uint16_t doy = ptm->tm_yday;

    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("a_srs", "Spatial reference to use to write output", d->opts.aSrs);
    args.add("compression", "Compression to use for output ('LASZIP' or "
        "'LAZPERF')", d->opts.compression, las::Compression::None);
    args.add("discard_high_return_numbers", "Discard points with out-of-spec "
        "return numbers.", d->opts.discardHighReturnNumbers);
    args.add("extra_dims", "Dimensions to write above those in point format", d->opts.extraDimSpec);
    args.add("forward", "Dimensions to forward from LAS reader", d->opts.forwardSpec);

    args.add("filesource_id", "File source ID number.", d->opts.filesourceId,
        decltype(d->opts.filesourceId)(0));
    args.add("major_version", "LAS major version", d->opts.majorVersion,
        decltype(d->opts.majorVersion)(1));
    args.add("minor_version", "LAS minor version", d->opts.minorVersion,
        decltype(d->opts.minorVersion)(2));
    args.add("dataformat_id", "Point format", d->opts.dataformatId,
        decltype(d->opts.dataformatId)(3));
    args.add("format", "Point format", d->opts.dataformatId,
        decltype(d->opts.dataformatId)(3));
    args.add("global_encoding", "Global encoding byte", d->opts.globalEncoding,
        decltype(d->opts.globalEncoding)(0));
    args.add("project_id", "Project ID", d->opts.projectId);
    args.add("system_id", "System ID", d->opts.systemId,
        decltype(d->opts.systemId)(m_lasHeader.getSystemIdentifier()));
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
    args.add("vlrs", "List of VLRs to set", d->opts.userVLRs);
}

void LasWriter::initialize()
{
    std::string ext = FileUtils::extension(m_filename);
    ext = Utils::tolower(ext);
    if ((ext == ".laz") && (d->opts.compression == las::Compression::None))
    {
#if defined(PDAL_HAVE_LASZIP)
        d->opts.compression = las::Compression::LasZip;
#elif defined(PDAL_HAVE_LAZPERF)
        d->opts.compression = las::Compression::LazPerf;
#endif
    }

    if (!d->opts.aSrs.empty())
        setSpatialReference(d->opts.aSrs);
    if (d->opts.compression != las::Compression::None)
        m_lasHeader.setCompressed(true);

#if !defined(PDAL_HAVE_LASZIP)
    if (d->opts.compression == las::Compression::LasZip)
        throwError("Can't write LAZ output. PDAL not built with LASzip.");
#endif
#if !defined(PDAL_HAVE_LAZPERF)
    if (d->opts.compression == las::Compression::LazPerf)
        throwError("Can't write LAZ output. PDAL not built with LAZperf.");
#endif

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
            ": Attempting to write '" << m_filename << "' with multiple "
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
        Dimension::IdList ids = las::pdrfDims(m_lasHeader.pointFormat());
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
void LasWriter::addUserVlrs()
{
    for (const auto& v : d->opts.userVLRs)
        addVlr(v);
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
            m_forwardVlrs = true;
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
            m_forwardVlrs = true;
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
    if(d->opts.writePDALMetadata)
    {
        MetadataNode m = table.metadata();
        addMetadataVlr(m);
        addPipelineVlr();
    }
    addExtraBytesVlr();
    addUserVlrs();
    addForwardVlrs();
}


void LasWriter::readyFile(const std::string& filename,
    const SpatialReference& srs)
{
    std::ostream *out = Utils::createFile(filename, true);
    if (!out)
        throwError("Couldn't open file '" + filename + "' for output.");
    m_curFilename = filename;
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

    m_summaryData.reset(new LasSummaryData());
    m_ostream = outStream;
    if (m_lasHeader.compressed())
        readyCompression();

    // Compression should cause the last of the VLRs to get filled.  We now
    // have a valid count, so fill the header again.
    fillHeader();

    // Write the header.
    OLeStream out(m_ostream);
    out << m_lasHeader;

    m_lasHeader.setVlrOffset((uint32_t)m_ostream->tellp());

    for (auto vi = m_vlrs.begin(); vi != m_vlrs.end(); ++vi)
    {
        LasVLR& vlr = *vi;
        vlr.write(out, m_lasHeader.versionEquals(1, 0) ? 0xAABB : 0);
    }

    // Write the point data start signature for version 1.0.
    if (m_lasHeader.versionEquals(1, 0))
        out << (uint16_t)0xCCDD;
    m_lasHeader.setPointOffset((uint32_t)m_ostream->tellp());
    if (d->opts.compression == las::Compression::LasZip)
        openCompression();

    // Set the point buffer size here in case we're using the streaming
    // interface.
    m_pointBuf.resize(m_lasHeader.pointLen());
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
    if ((json.size() > LasVLR::MAX_DATA_SIZE) &&
        !m_lasHeader.versionAtLeast(1, 4))
    {
        log()->get(LogLevel::Debug) << "pdal metadata VLR too large "
            "to write in VLR for files < LAS 1.4";
    } else
    {
        std::vector<uint8_t> data(json.begin(), json.end());
        addVlr(PDAL_USER_ID, PDAL_METADATA_RECORD_ID, "PDAL metadata", data);
    }
}

void LasWriter::addPipelineVlr()
{
    // Write a VLR with the PDAL pipeline.
    std::ostringstream ostr;
    PipelineWriter::writePipeline(this, ostr);
    std::string json = ostr.str();
    if (json.size() > LasVLR::MAX_DATA_SIZE &&
        !m_lasHeader.versionAtLeast(1, 4))
    {
        log()->get(LogLevel::Debug) << "pdal pipeline VLR too large "
            "to write in VLR for files < LAS 1.4";
    } else
    {
        std::vector<uint8_t> data(json.begin(), json.end());
        addVlr(PDAL_USER_ID, PDAL_PIPELINE_RECORD_ID, "PDAL pipeline", data);
    }
}


/// Add VLRs forwarded from the input.
void LasWriter::addForwardVlrs()
{
    std::vector<uint8_t> data;

    if (!m_forwardVlrs)
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
            data = Utils::base64_decode(dataNode.value());
            uint16_t recordId = (uint16_t)std::stoi(recordIdNode.value());
            addVlr(userIdNode.value(), recordId, n.description(), data);
        }
    }
}

/// Set VLRs from the active spatial reference.
/// \param  srs - Active spatial reference.
void LasWriter::addSpatialRefVlrs()
{
    // Delete any existing spatial ref VLRs.  This can be an issue if we're
    // using the reader to write multiple output files via a filename template.
    deleteVlr(TRANSFORM_USER_ID, GEOTIFF_DIRECTORY_RECORD_ID);
    deleteVlr(TRANSFORM_USER_ID, GEOTIFF_DOUBLES_RECORD_ID);
    deleteVlr(TRANSFORM_USER_ID, GEOTIFF_ASCII_RECORD_ID);
    deleteVlr(TRANSFORM_USER_ID, WKT_RECORD_ID);
    deleteVlr(LIBLAS_USER_ID, WKT_RECORD_ID);

    if (m_lasHeader.versionAtLeast(1, 4))
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
        GeotiffTags tags(m_srs);

        if (tags.directoryData().empty())
            throwError("Invalid spatial reference for writing GeoTiff VLR.");

        addVlr(TRANSFORM_USER_ID, GEOTIFF_DIRECTORY_RECORD_ID,
                "GeoTiff GeoKeyDirectoryTag", tags.directoryData());
        if (tags.doublesData().size())
            addVlr(TRANSFORM_USER_ID, GEOTIFF_DOUBLES_RECORD_ID,
                "GeoTiff GeoDoubleParamsTag", tags.doublesData());
        if (tags.asciiData().size())
            addVlr(TRANSFORM_USER_ID, GEOTIFF_ASCII_RECORD_ID,
                "GeoTiff GeoAsciiParamsTag", tags.asciiData());
    }
    catch (Geotiff::error& err)
    {
        throwError(err.what());
    }
}


/// Add a Well-known Text VLR associated with the spatial reference.
/// \return  Whether the VLR was added.
bool LasWriter::addWktVlr()
{
    // LAS 1.4 requires WKTv1
    std::string wkt = m_srs.getWKT1();
    if (wkt.empty())
        return false;

    std::vector<uint8_t> wktBytes(wkt.begin(), wkt.end());
    // This tacks a NULL to the end of the data, which is required by the spec.
    wktBytes.resize(wktBytes.size() + 1, 0);
    addVlr(TRANSFORM_USER_ID, WKT_RECORD_ID, "OGC Transformation Record",
        wktBytes);

    // The data in the vector gets moved to the VLR, so we have to recreate it.
    std::vector<uint8_t> wktBytes2(wkt.begin(), wkt.end());
    wktBytes2.resize(wktBytes2.size() + 1, 0);
    addVlr(LIBLAS_USER_ID, WKT_RECORD_ID,
        "OGR variant of OpenGIS WKT SRS", wktBytes2);
    return true;
}


void LasWriter::addExtraBytesVlr()
{
    if (m_extraDims.empty())
        return;

    std::vector<uint8_t> ebBytes;
    for (auto& dim : m_extraDims)
    {
        las::ExtraBytesIf eb(dim.m_name, dim.m_dimType.m_type,
            Dimension::description(dim.m_dimType.m_id));
        eb.appendTo(ebBytes);
    }

    addVlr(SPEC_USER_ID, EXTRA_BYTES_RECORD_ID, "Extra Bytes Record", ebBytes);
}


/// Add a standard or variable-length VLR depending on the data size.
/// \param  userId - VLR user ID
/// \param  recordId - VLR record ID
/// \param  description - VLR description
/// \param  data - Raw VLR data
void LasWriter::addVlr(const std::string& userId, uint16_t recordId,
   const std::string& description, std::vector<uint8_t>& data)
{
    addVlr(ExtLasVLR(userId, recordId, description, data));
}

/// Add a standard or variable-length VLR depending on the data size.
/// \param  evlr  VLR to add.
void LasWriter::addVlr(const ExtLasVLR& evlr)
{
    if (evlr.dataLen() > LasVLR::MAX_DATA_SIZE)
    {
        if (m_lasHeader.versionAtLeast(1, 4))
            m_eVlrs.push_back(std::move(evlr));
        else
            throwError("Can't write VLR with user ID/record ID = " +
                evlr.userId() + "/" + std::to_string(evlr.recordId()) +
                ".  The data size exceeds the maximum supported.");
    }
    else
        m_vlrs.push_back(std::move(evlr));
}

/// Delete a VLR from the vlr list.
///
void LasWriter::deleteVlr(const std::string& userId, uint16_t recordId)
{
    auto matches = [&userId, recordId](const LasVLR& vlr)
    {
        return vlr.matches(userId, recordId);
    };

    Utils::remove_if(m_vlrs, matches);
    Utils::remove_if(m_eVlrs, matches);
}


template <typename T>
void LasWriter::handleHeaderForward(const std::string& s, T& headerVal,
    const MetadataNode& base)
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

    try
    {
        m_lasHeader.setScaling(m_scaling);
    }
    catch (const LasHeader::error& err)
    {
        throwError(err.what());
    }
    m_lasHeader.setVlrCount(m_vlrs.size());
    m_lasHeader.setEVlrCount(m_eVlrs.size());

    m_lasHeader.setPointFormat(d->opts.dataformatId.val());
    m_lasHeader.setPointLen(m_lasHeader.basePointLen() + m_extraByteLen);
    m_lasHeader.setVersionMinor(d->opts.minorVersion.val());
    m_lasHeader.setCreationYear(d->opts.creationYear.val());
    m_lasHeader.setCreationDOY(d->opts.creationDoy.val());
    m_lasHeader.setSoftwareId(d->opts.softwareId.val());
    m_lasHeader.setSystemId(d->opts.systemId.val());
    m_lasHeader.setProjectId(d->opts.projectId.val());
    m_lasHeader.setFileSourceId(d->opts.filesourceId.val());

    // We always write a WKT VLR for version 1.4 and later.
    uint16_t globalEncoding = d->opts.globalEncoding.val();
    if (m_lasHeader.versionAtLeast(1, 4))
        globalEncoding |= WKT_MASK;
    m_lasHeader.setGlobalEncoding(globalEncoding);

    auto ok = m_lasHeader.pointFormatSupported();
    if (!ok)
        throwError(ok.what());
}


void LasWriter::readyCompression()
{
    deleteVlr(LASZIP_USER_ID, LASZIP_RECORD_ID);
    if (d->opts.compression == las::Compression::LasZip)
        readyLasZipCompression();
    else if (d->opts.compression == las::Compression::LazPerf)
        readyLazPerfCompression();
}


void LasWriter::handleLaszip(int result)
{
#ifdef PDAL_HAVE_LASZIP
    if (result)
    {
        char *buf;
        laszip_get_error(m_laszip, &buf);
        throwError(buf);
    }
#endif
}


void LasWriter::readyLasZipCompression()
{
#ifdef PDAL_HAVE_LASZIP
    handleLaszip(laszip_create(&m_laszip));
    handleLaszip(laszip_set_point_type_and_size(m_laszip,
        m_lasHeader.pointFormat(), m_lasHeader.pointLen()));

    laszip_U8* data;
    laszip_U32 size;
    handleLaszip(laszip_create_laszip_vlr(m_laszip, &data, &size));

    // A VLR has 54 header bytes that we skip in order to get to the payload.
    std::vector<laszip_U8> vlrData(data + 54, data + size);

    addVlr(LASZIP_USER_ID, LASZIP_RECORD_ID, "http://laszip.org", vlrData);
#endif
}


void LasWriter::readyLazPerfCompression()
{
#ifdef PDAL_HAVE_LAZPERF
    int ebCount = m_lasHeader.pointLen() - m_lasHeader.basePointLen();
    m_compressor = new LazPerfVlrCompressor(*m_ostream, m_lasHeader.pointFormat(), ebCount);
    std::vector<char> lazVlrData = m_compressor->vlrData();
    std::vector<uint8_t> vlrdata(lazVlrData.begin(), lazVlrData.end());
    addVlr(LASZIP_USER_ID, LASZIP_RECORD_ID, "http://laszip.org", vlrdata);
#endif
}


/// Prepare the compressor to write points.
/// \param  pointFormat - Formt of points we're writing.
void LasWriter::openCompression()
{
#ifdef PDAL_HAVE_LASZIP
    handleLaszip(laszip_open_writer_stream(m_laszip, *m_ostream, true, true));
#endif
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
                log()->get(LogLevel::Warning) << "Auto offset for " << name <<
                    " requested in stream mode.  Using value of " <<
                    offset.m_val << "." << std::endl;
            }
        };

        doOffset(m_scaling.m_xXform.m_offset,
            point.getFieldAs<double>(Dimension::Id::X), "X");
        doOffset(m_scaling.m_yXform.m_offset,
            point.getFieldAs<double>(Dimension::Id::Y), "Y");
        doOffset(m_scaling.m_zXform.m_offset,
            point.getFieldAs<double>(Dimension::Id::Z), "Z");

        m_firstPoint = false;
    }
    return processPoint(point);
}


// This is separated from processOne so that we're sure when processOne is
// called we know we're in stream mode.
bool LasWriter::processPoint(PointRef& point)
{
    if (d->opts.compression == las::Compression::LasZip)
    {
        if (!writeLasZipBuf(point))
            return false;
    }
    else if (d->opts.compression == las::Compression::LazPerf)
    {
        LeInserter ostream(m_pointBuf.data(), m_pointBuf.size());
        if (!fillPointBuf(point, ostream))
            return false;
        writeLazPerfBuf(m_pointBuf.data(), m_lasHeader.pointLen(), 1);
    }
    else
    {
        LeInserter ostream(m_pointBuf.data(), m_pointBuf.size());
        if (!fillPointBuf(point, ostream))
            return false;
        m_ostream->write(m_pointBuf.data(), m_lasHeader.pointLen());
    }
    return true;
}


void LasWriter::prerunFile(const PointViewSet& pvSet)
{
    m_scaling.setAutoXForm(pvSet);
}


void LasWriter::writeView(const PointViewPtr view)
{
    Utils::writeProgress(m_progressFd, "READYVIEW",
        std::to_string(view->size()));

    point_count_t pointLen = m_lasHeader.pointLen();

    // Since we use the LASzip API, we can't benefit from building
    // a buffer of multiple points, so loop.
    if (d->opts.compression == las::Compression::LasZip)
    {
        PointRef point(*view, 0);
        for (PointId idx = 0; idx < view->size(); ++idx)
        {
            point.setPointId(idx);
            processPoint(point);
        }
    }
    else
    {
        // Make a buffer of at most a meg.
        m_pointBuf.resize((std::min)((point_count_t)1000000,
                    pointLen * view->size()));

        const PointView& viewRef(*view.get());

        point_count_t remaining = view->size();
        PointId idx = 0;
        while (remaining)
        {
            point_count_t filled = fillWriteBuf(viewRef, idx, m_pointBuf);
            idx += filled;
            remaining -= filled;

            if (d->opts.compression == las::Compression::LazPerf)
                writeLazPerfBuf(m_pointBuf.data(), pointLen, filled);
            else
                m_ostream->write(m_pointBuf.data(), filled * pointLen);
        }
    }
    Utils::writeProgress(m_progressFd, "DONEVIEW",
        std::to_string(view->size()));
}


bool LasWriter::writeLasZipBuf(PointRef& point)
{
#ifdef PDAL_HAVE_LASZIP
    const bool has14PointFormat = m_lasHeader.has14PointFormat();
    const size_t maxReturnCount = m_lasHeader.maxReturnCount();

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
        if (m_discardHighReturnNumbers)
        {
            // If this return number is too high, pitch the point.
            if (returnNumber > maxReturnCount)
                return false;
            numberOfReturns = maxReturnCount;
        }
    }

    auto converter = [this](double d, Dimension::Id dim) -> int32_t
    {
        int32_t i(0);

        if (!Utils::numericCast(d, i))
            throwError("Unable to convert scaled value (" +
                Utils::toString(d) + ") to "
                "int32 for dimension '" + Dimension::name(dim) +
                "' when writing LAS/LAZ file " + m_curFilename + ".");
        return i;
    };

    double xOrig = point.getFieldAs<double>(Id::X);
    double yOrig = point.getFieldAs<double>(Id::Y);
    double zOrig = point.getFieldAs<double>(Id::Z);
    double x = m_scaling.m_xXform.toScaled(xOrig);
    double y = m_scaling.m_yXform.toScaled(yOrig);
    double z = m_scaling.m_zXform.toScaled(zOrig);

    uint8_t scanChannel = point.getFieldAs<uint8_t>(Id::ScanChannel);
    uint8_t scanDirectionFlag =
        point.getFieldAs<uint8_t>(Id::ScanDirectionFlag);
    uint8_t edgeOfFlightLine =
        point.getFieldAs<uint8_t>(Id::EdgeOfFlightLine);
    uint8_t classification = point.getFieldAs<uint8_t>(Id::Classification);
    uint8_t classFlags = 0;
    if (point.hasDim(Id::ClassFlags))
        classFlags = point.getFieldAs<uint8_t>(Id::ClassFlags);
    else
        classFlags = classification >> 5;

    laszip_point_struct p;
    p.X = converter(x, Id::X);
    p.Y = converter(y, Id::Y);
    p.Z = converter(z, Id::Z);
    p.intensity = point.getFieldAs<uint16_t>(Id::Intensity);
    p.scan_direction_flag = scanDirectionFlag;
    p.edge_of_flight_line = edgeOfFlightLine;
    p.synthetic_flag = classFlags & 0x1;
    p.keypoint_flag = (classFlags >> 1) & 0x1;
    p.withheld_flag = (classFlags >> 2) & 0x1;
    p.user_data = point.getFieldAs<uint8_t>(Id::UserData);
    p.point_source_ID = point.getFieldAs<uint16_t>(Id::PointSourceId);

    if (has14PointFormat)
    {
        if (classification > 31)
            p.classification = 0;
        else
            p.classification = classification;
        p.extended_classification = classification;
        p.extended_classification_flags = classFlags;
        // The API takes care of writing scan_angle_rank.
        p.number_of_returns = (std::min)((uint8_t)7, numberOfReturns);
        p.return_number = (std::min)((uint8_t)7, returnNumber);

        // This should always work if ScanAngleRank isn't wonky.
        p.extended_scan_angle = static_cast<laszip_I16>(
            std::round(point.getFieldAs<float>(Id::ScanAngleRank) / .006f));
        p.extended_point_type = 1;
        p.extended_scanner_channel = scanChannel;
        p.extended_return_number = returnNumber;
        p.extended_number_of_returns = numberOfReturns;
    }
    else
    {
        p.return_number = returnNumber;
        p.number_of_returns = numberOfReturns;
        p.scan_angle_rank = point.getFieldAs<int8_t>(Id::ScanAngleRank);
        p.classification = classification;
        p.extended_point_type = 0;
    }

    if (m_lasHeader.hasTime())
        p.gps_time = point.getFieldAs<double>(Id::GpsTime);

    if (m_lasHeader.hasColor())
    {
        p.rgb[0] = point.getFieldAs<uint16_t>(Id::Red);
        p.rgb[1] = point.getFieldAs<uint16_t>(Id::Green);
        p.rgb[2] = point.getFieldAs<uint16_t>(Id::Blue);
    }

    if (m_lasHeader.hasInfrared())
        p.rgb[3] = point.getFieldAs<uint16_t>(Id::Infrared);

    if (m_extraDims.size())
    {
        LeInserter ostream(m_pointBuf.data(), m_pointBuf.size());
        Everything e;
        for (auto& dim : m_extraDims)
        {
            point.getField((char *)&e, dim.m_dimType.m_id,
                dim.m_dimType.m_type);
            Utils::insertDim(ostream, dim.m_dimType.m_type, e);
        }
        assert(m_extraByteLen == ostream.position());
    }
    p.extra_bytes = (laszip_U8 *)m_pointBuf.data();
    p.num_extra_bytes = m_extraByteLen;

    m_summaryData->addPoint(xOrig, yOrig, zOrig, returnNumber);

    handleLaszip(laszip_set_point(m_laszip, &p));
    handleLaszip(laszip_write_point(m_laszip));
#endif
    return true;
}


void LasWriter::writeLazPerfBuf(char *pos, size_t pointLen,
    point_count_t numPts)
{
#ifdef PDAL_HAVE_LAZPERF
    for (point_count_t i = 0; i < numPts; i++)
    {
        m_compressor->compress(pos);
        pos += pointLen;
    }
#endif
}


bool LasWriter::fillPointBuf(PointRef& point, LeInserter& ostream)
{
    bool has14PointFormat = m_lasHeader.has14PointFormat();
    static const size_t maxReturnCount = m_lasHeader.maxReturnCount();

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
        if (m_discardHighReturnNumbers)
        {
            // If this return number is too high, pitch the point.
            if (returnNumber > maxReturnCount)
                return false;
            numberOfReturns = maxReturnCount;
        }
    }

    auto converter = [this](double d, Dimension::Id dim) -> int32_t
    {
        int32_t i(0);

        if (!Utils::numericCast(d, i))
            throwError("Unable to convert scaled value (" +
                Utils::toString(d) + ") to "
                "int32 for dimension '" + Dimension::name(dim) +
                "' when writing LAS/LAZ file " + m_curFilename + ".");
        return i;
    };

    double xOrig = point.getFieldAs<double>(Id::X);
    double yOrig = point.getFieldAs<double>(Id::Y);
    double zOrig = point.getFieldAs<double>(Id::Z);
    double x = m_scaling.m_xXform.toScaled(xOrig);
    double y = m_scaling.m_yXform.toScaled(yOrig);
    double z = m_scaling.m_zXform.toScaled(zOrig);

    ostream << converter(x, Id::X);
    ostream << converter(y, Id::Y);
    ostream << converter(z, Id::Z);

    ostream << point.getFieldAs<uint16_t>(Id::Intensity);

    uint8_t scanChannel = point.getFieldAs<uint8_t>(Id::ScanChannel);
    uint8_t scanDirectionFlag = point.getFieldAs<uint8_t>(Id::ScanDirectionFlag);
    uint8_t edgeOfFlightLine = point.getFieldAs<uint8_t>(Id::EdgeOfFlightLine);
    uint8_t classification = point.getFieldAs<uint8_t>(Id::Classification);

    if (has14PointFormat)
    {
        uint8_t bits = returnNumber | (numberOfReturns << 4);
        ostream << bits;

        uint8_t classFlags;
        if (point.hasDim(Id::ClassFlags))
            classFlags = point.getFieldAs<uint8_t>(Id::ClassFlags);
        else
            classFlags = classification >> 5;
        bits = (classFlags & 0x0F) |
            ((scanChannel & 0x03) << 4) |
            ((scanDirectionFlag & 0x01) << 6) |
            ((edgeOfFlightLine & 0x01) << 7);
        ostream << bits;
    }
    else
    {
        uint8_t bits = returnNumber | (numberOfReturns << 3) |
            (scanDirectionFlag << 6) | (edgeOfFlightLine << 7);
        ostream << bits;
    }

    ostream << classification;

    uint8_t userData = point.getFieldAs<uint8_t>(Id::UserData);
    if (has14PointFormat)
    {
         // Guaranteed to fit if scan angle rank isn't wonky.
        int16_t scanAngleRank =
            static_cast<int16_t>(std::round(
                point.getFieldAs<float>(Id::ScanAngleRank) / .006f));
        ostream << userData << scanAngleRank;
    }
    else
    {
        int8_t scanAngleRank = point.getFieldAs<int8_t>(Id::ScanAngleRank);
        ostream << scanAngleRank << userData;
    }

    ostream << point.getFieldAs<uint16_t>(Id::PointSourceId);

    if (m_lasHeader.hasTime())
        ostream << point.getFieldAs<double>(Id::GpsTime);

    if (m_lasHeader.hasColor())
    {
        ostream << point.getFieldAs<uint16_t>(Id::Red);
        ostream << point.getFieldAs<uint16_t>(Id::Green);
        ostream << point.getFieldAs<uint16_t>(Id::Blue);
    }

    if (m_lasHeader.hasInfrared())
        ostream << point.getFieldAs<uint16_t>(Id::Infrared);

    Everything e;
    for (auto& dim : m_extraDims)
    {
        point.getField((char *)&e, dim.m_dimType.m_id, dim.m_dimType.m_type);
        Utils::insertDim(ostream, dim.m_dimType.m_type, e);
    }

    m_summaryData->addPoint(xOrig, yOrig, zOrig, returnNumber);
    return true;
}


point_count_t LasWriter::fillWriteBuf(const PointView& view,
    PointId startId, std::vector<char>& buf)
{
    point_count_t blocksize = buf.size() / m_lasHeader.pointLen();
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
    Utils::writeProgress(m_progressFd, "DONEFILE", m_curFilename);
    getMetadata().addList("filename", m_curFilename);
    delete m_ostream;
    m_ostream = NULL;
}


void LasWriter::finishOutput()
{
    if (d->opts.compression == las::Compression::LasZip)
        finishLasZipOutput();
    else if (d->opts.compression == las::Compression::LazPerf)
        finishLazPerfOutput();

    log()->get(LogLevel::Debug) << "Wrote " <<
        m_summaryData->getTotalNumPoints() <<
        " points to the LAS file" << std::endl;

    OLeStream out(m_ostream);

    // addVlr prevents any eVlrs from being added before version 1.4.
    m_lasHeader.setEVlrOffset(m_eVlrs.size() ? (uint32_t)m_ostream->tellp() : 0);
    for (auto vi = m_eVlrs.begin(); vi != m_eVlrs.end(); ++vi)
    {
        ExtLasVLR evlr = *vi;
        out << evlr;
    }

    // Reset the offset/scale since it may have been auto-computed
    try
    {
        m_lasHeader.setScaling(m_scaling);
    }
    catch (const LasHeader::error& err)
    {
        throwError(err.what());
    }

    // The summary is calculated as points are written.
    try
    {
        m_lasHeader.setSummary(*m_summaryData);
    }
    catch (const LasHeader::error& err)
    {
        throwError(err.what());
    }

    out.seek(0);
    out << m_lasHeader;
    out.seek(m_lasHeader.pointOffset());

    m_ostream->flush();
}


void LasWriter::finishLasZipOutput()
{
#ifdef PDAL_HAVE_LASZIP
    handleLaszip(laszip_close_writer(m_laszip));
    handleLaszip(laszip_destroy(m_laszip));
#endif
}


void LasWriter::finishLazPerfOutput()
{
#ifdef PDAL_HAVE_LAZPERF
    m_compressor->done();
#endif
}

} // namespace pdal
