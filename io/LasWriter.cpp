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
#ifdef PDAL_HAVE_LAZPERF
#pragma push_macro("min")
#pragma push_macro("max")
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#include <laz-perf/factory.hpp>
#include <laz-perf/io.hpp>
#pragma pop_macro("max")
#pragma pop_macro("min")
#endif

#include "LasWriter.hpp"

#include <climits>
#include <iostream>
#include <vector>

#include <nlohmann/json.hpp>

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
        "available if enabled at compile-time. Note that LAZ \n" \
        "does not provide LAS 1.4 support at this time.",
    "http://pdal.io/stages/writers.las.html",
    { "las", "laz" }
};

CREATE_STATIC_STAGE(LasWriter, s_info)

std::string LasWriter::getName() const { return s_info.name; }

LasWriter::LasWriter() : m_compressor(nullptr), m_ostream(NULL),
    m_compression(LasCompression::None), m_srsCnt(0),
    m_userVLRs(new NL::json)
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
    args.add("a_srs", "Spatial reference to use to write output", m_aSrs);
    args.add("compression", "Compression to use for output ('LASZIP' or "
        "'LAZPERF')", m_compression, LasCompression::None);
    args.add("discard_high_return_numbers", "Discard points with out-of-spec "
        "return numbers.", m_discardHighReturnNumbers);
    args.add("extra_dims", "Dimensions to write above those in point format",
        m_extraDimSpec);
    args.add("forward", "Dimensions to forward from LAS reader", m_forwardSpec);

    args.add("major_version", "LAS major version", m_majorVersion,
        decltype(m_majorVersion)(1));
    args.add("minor_version", "LAS minor version", m_minorVersion,
        decltype(m_minorVersion)(2));
    args.add("dataformat_id", "Point format", m_dataformatId,
        decltype(m_dataformatId)(3));
    args.add("format", "Point format", m_dataformatId,
        decltype(m_dataformatId)(3));
    args.add("global_encoding", "Global encoding byte", m_globalEncoding,
        decltype(m_globalEncoding)(0));
    args.add("project_id", "Project ID", m_projectId);
    args.add("system_id", "System ID", m_systemId,
        decltype(m_systemId)(m_lasHeader.getSystemIdentifier()));
    args.add("software_id", "Software ID", m_softwareId,
        decltype(m_softwareId)(GetDefaultSoftwareId()));
    args.add("creation_doy", "Creation day of year", m_creationDoy,
        decltype(m_creationDoy)(doy));
    args.add("creation_year", "Creation year", m_creationYear,
        decltype(m_creationYear)(year));
    args.add("pdal_metadata", "Write PDAL metadata as VLR?",
        m_writePDALMetadata, decltype(m_writePDALMetadata)(false));
    args.add("scale_x", "X scale factor", m_scaleX, decltype(m_scaleX)(".01"));
    args.add("scale_y", "Y scale factor", m_scaleY, decltype(m_scaleY)(".01"));
    args.add("scale_z", "Z scale factor", m_scaleZ, decltype(m_scaleZ)(".01"));
    args.add("offset_x", "X offset", m_offsetX);
    args.add("offset_y", "Y offset", m_offsetY);
    args.add("offset_z", "Z offset", m_offsetZ);
    args.add("vlrs", "List of VLRs to set", *m_userVLRs);
}

void LasWriter::initialize()
{
    std::string ext = FileUtils::extension(m_filename);
    ext = Utils::tolower(ext);
    if ((ext == ".laz") && (m_compression == LasCompression::None))
#if defined(PDAL_HAVE_LASZIP)
        m_compression = LasCompression::LasZip;
#elif defined(PDAL_HAVE_LAZPERF)
        m_compression = LasCompression::LazPerf;
#endif

    if (!m_aSrs.empty())
        setSpatialReference(m_aSrs);
    if (m_compression != LasCompression::None)
        m_lasHeader.setCompressed(true);
#if !defined(PDAL_HAVE_LASZIP) && !defined(PDAL_HAVE_LAZPERF)
    if (m_compression != LasCompression::None)
        throwError("Can't write LAZ output.  PDAL not built with "
            "LASzip or LAZperf.");
#endif
    try
    {
        m_extraDims = LasUtils::parse(m_extraDimSpec, true);
    }
    catch (const LasUtils::error& err)
    {
        throwError(err.what());
    }
    fillForwardList();
}


void LasWriter::spatialReferenceChanged(const SpatialReference&)
{
    if (++m_srsCnt > 1 && m_aSrs.empty())
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
        Dimension::IdList ids = m_lasHeader.usedDims();
        DimTypeList dimTypes = layout->dimTypes();
        for (auto& dt : dimTypes)
        {
            if (!Utils::contains(ids, dt.m_id))
                m_extraDims.push_back(
                    ExtraDim(layout->dimName(dt.m_id), dt.m_type));
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
    for (const auto& v : *m_userVLRs)
    {
        uint16_t recordId(1);
        std::string userId("");
        std::string description("");
        std::string b64data("");
        std::string user("");
        if (!v.contains("user_id"))
            throw pdal_error("VLR must contain a 'user_id'!");
        userId = v["user_id"].get<std::string>();

        if (!v.contains("data"))
            throw pdal_error("VLR must contain a base64-encoded 'data' member");
        b64data = v["data"].get<std::string>();

        // Record ID should always be no more than 2 bytes.
        if (v.contains("record_id"))
            recordId = v["record_id"].get<uint16_t>();

        if (v.contains("description"))
            description = v["description"].get<std::string>();

        std::vector<uint8_t> data = Utils::base64_decode(b64data);
        addVlr(userId, recordId, description, data);
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
    for (auto& name : m_forwardSpec)
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
    if(m_writePDALMetadata)
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
    if (m_compression == LasCompression::LasZip)
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
    std::string wkt = m_srs.getWKT();
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
        ExtraBytesIf eb(dim.m_name, dim.m_dimType.m_type,
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
    if (data.size() > LasVLR::MAX_DATA_SIZE)
    {
        if (m_lasHeader.versionAtLeast(1, 4))
        {
            ExtLasVLR evlr(userId, recordId, description, data);
            m_eVlrs.push_back(std::move(evlr));
        }
        else
            throwError("Can't write VLR with user ID/record ID = " +
                userId + "/" + std::to_string(recordId) +
                ".  The data size exceeds the maximum supported.");
    }
    else
    {
        LasVLR vlr(userId, recordId, description, data);
        m_vlrs.push_back(std::move(vlr));
    }
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
    handleHeaderForward("major_version", m_majorVersion, forward);
    handleHeaderForward("minor_version", m_minorVersion, forward);
    handleHeaderForward("dataformat_id", m_dataformatId, forward);
    handleHeaderForward("filesource_id", m_filesourceId, forward);
    handleHeaderForward("global_encoding", m_globalEncoding, forward);
    handleHeaderForward("project_id", m_projectId, forward);
    handleHeaderForward("system_id", m_systemId, forward);
    handleHeaderForward("software_id", m_softwareId, forward);
    handleHeaderForward("creation_doy", m_creationDoy, forward);
    handleHeaderForward("creation_year", m_creationYear, forward);

    handleHeaderForward("scale_x", m_scaleX, forward);
    handleHeaderForward("scale_y", m_scaleY, forward);
    handleHeaderForward("scale_z", m_scaleZ, forward);
    handleHeaderForward("offset_x", m_offsetX, forward);
    handleHeaderForward("offset_y", m_offsetY, forward);
    handleHeaderForward("offset_z", m_offsetZ, forward);

    m_scaling.m_xXform.m_scale.set(m_scaleX.val());
    m_scaling.m_yXform.m_scale.set(m_scaleY.val());
    m_scaling.m_zXform.m_scale.set(m_scaleZ.val());
    m_scaling.m_xXform.m_offset.set(m_offsetX.val());
    m_scaling.m_yXform.m_offset.set(m_offsetY.val());
    m_scaling.m_zXform.m_offset.set(m_offsetZ.val());
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

    m_lasHeader.setPointFormat(m_dataformatId.val());
    m_lasHeader.setPointLen(m_lasHeader.basePointLen() + m_extraByteLen);
    m_lasHeader.setVersionMinor(m_minorVersion.val());
    m_lasHeader.setCreationYear(m_creationYear.val());
    m_lasHeader.setCreationDOY(m_creationDoy.val());
    m_lasHeader.setSoftwareId(m_softwareId.val());
    m_lasHeader.setSystemId(m_systemId.val());
    m_lasHeader.setProjectId(m_projectId.val());
    m_lasHeader.setFileSourceId(m_filesourceId.val());

    // We always write a WKT VLR for version 1.4 and later.
    uint16_t globalEncoding = m_globalEncoding.val();
    if (m_lasHeader.versionAtLeast(1, 4))
        globalEncoding |= WKT_MASK;
    m_lasHeader.setGlobalEncoding(globalEncoding);

    if (!m_lasHeader.pointFormatSupported())
        throwError("Unsupported LAS output point format: " +
            Utils::toString((int)m_lasHeader.pointFormat()) + ".");
}


void LasWriter::readyCompression()
{
    deleteVlr(LASZIP_USER_ID, LASZIP_RECORD_ID);
    if (m_compression == LasCompression::LasZip)
        readyLasZipCompression();
    else if (m_compression == LasCompression::LazPerf)
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
    if (m_lasHeader.versionAtLeast(1, 4))
        throwError("Can't write version 1.4 output with LAZperf.");

    laszip::factory::record_schema schema;
    schema.push(laszip::factory::record_item::POINT10);
    if (m_lasHeader.hasTime())
        schema.push(laszip::factory::record_item::GPSTIME);
    if (m_lasHeader.hasColor())
        schema.push(laszip::factory::record_item::RGB12);
    laszip::io::laz_vlr zipvlr = laszip::io::laz_vlr::from_schema(schema);
    std::vector<uint8_t> data(zipvlr.size());
    zipvlr.extract((char *)data.data());
    addVlr(LASZIP_USER_ID, LASZIP_RECORD_ID, "http://laszip.org", data);

    delete m_compressor;
    m_compressor = new LazPerfVlrCompressor(*m_ostream, schema,
        zipvlr.chunk_size);
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
                "requested in stream mode.  Using value of 1.0." << std::endl;
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
                    "requested in stream mode.  Using value of " <<
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
    if (m_compression == LasCompression::LasZip)
    {
        if (!writeLasZipBuf(point))
            return false;
    }
    else if (m_compression == LasCompression::LazPerf)
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
    if (m_compression == LasCompression::LasZip)
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

            if (m_compression == LasCompression::LazPerf)
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
    const bool has14Format = m_lasHeader.has14Format();
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

    if (has14Format)
    {
        p.classification = (classification & 0x1F) | (classFlags << 5);
        p.scan_angle_rank = point.getFieldAs<int8_t>(Id::ScanAngleRank);
        p.number_of_returns = (std::min)((uint8_t)7, numberOfReturns);
        p.return_number = (std::min)((uint8_t)7, returnNumber);

        // This should always work if ScanAngleRank isn't wonky.
        p.extended_scan_angle = static_cast<laszip_I16>(
            std::round(point.getFieldAs<float>(Id::ScanAngleRank) / .006f));
        p.extended_point_type = 1;
        p.extended_scanner_channel = scanChannel;
        p.extended_classification_flags = classFlags;
        p.extended_classification = classification;
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
    bool has14Format = m_lasHeader.has14Format();
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
    uint8_t scanDirectionFlag =
        point.getFieldAs<uint8_t>(Id::ScanDirectionFlag);
    uint8_t edgeOfFlightLine =
        point.getFieldAs<uint8_t>(Id::EdgeOfFlightLine);

    if (has14Format)
    {
        uint8_t bits = returnNumber | (numberOfReturns << 4);
        ostream << bits;

        uint8_t classFlags = point.getFieldAs<uint8_t>(Id::ClassFlags);
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

    ostream << point.getFieldAs<uint8_t>(Id::Classification);

    uint8_t userData = point.getFieldAs<uint8_t>(Id::UserData);
    if (has14Format)
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
    if (m_compression == LasCompression::LasZip)
        finishLasZipOutput();
    else if (m_compression == LasCompression::LazPerf)
        finishLazPerfOutput();

    log()->get(LogLevel::Debug) << "Wrote " <<
        m_summaryData->getTotalNumPoints() <<
        " points to the LAS file" << std::endl;

    OLeStream out(m_ostream);

    // addVlr prevents any eVlrs from being added before version 1.4.
    m_lasHeader.setEVlrOffset((uint32_t)m_ostream->tellp());
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
