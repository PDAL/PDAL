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

#include "LasWriter.hpp"

#include <type_traits>
#include <boost/uuid/uuid_generators.hpp>
#include <iostream>

#include <pdal/PDALUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/Inserter.hpp>
#include <pdal/util/OStream.hpp>
#include <pdal/util/Utils.hpp>

#include "GeotiffSupport.hpp"
#include "ZipPoint.hpp"

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.las",
    "ASPRS LAS 1.0 - 1.4 writer. LASzip support is also \n" \
        "available if enabled at compile-time. Note that LAZ \n" \
        "does not provide LAS 1.4 support at this time.",
    "http://pdal.io/stages/writers.las.html" );

CREATE_STATIC_PLUGIN(1, 0, LasWriter, Writer, s_info)

std::string LasWriter::getName() const { return s_info.name; }

LasWriter::LasWriter() : m_ostream(NULL)
{
    m_majorVersion.setDefault(1);
    m_minorVersion.setDefault(2);
    m_dataformatId.setDefault(3);
    m_filesourceId.setDefault(0);
    m_globalEncoding.setDefault(0);
    m_systemId.setDefault(m_lasHeader.getSystemIdentifier());
    m_softwareId.setDefault(GetDefaultSoftwareId());

    std::time_t now;
    std::time(&now);
    std::tm* ptm = std::gmtime(&now);
    uint16_t year = ptm->tm_year + 1900;
    uint16_t doy = ptm->tm_yday;

    m_creationDoy.setDefault(doy);
    m_creationYear.setDefault(year);
    m_scaleX.setDefault(".01");
    m_scaleY.setDefault(".01");
    m_scaleZ.setDefault(".01");
    m_offsetX.setDefault("0");
    m_offsetY.setDefault("0");
    m_offsetZ.setDefault("0");
}


Options LasWriter::getDefaultOptions()
{
    Options options;
    LasHeader header;

    options.add("filename", "", "Name of the file for LAS/LAZ output.");
    options.add("compression", false, "Do we LASzip-compress the data?");
    options.add("major_version", 1, "LAS Major version");
    options.add("minor_version", 2, "LAS Minor version");
    options.add("dataformat_id", 3, "Point format to write");
    options.add("filesource_id", 0, "File Source ID for this file");
    options.add("global_encoding", 0, "Global encoding bits");
    options.add("system_id", header.getSystemIdentifier(),
        "System ID for this file");
    options.add("software_id", GetDefaultSoftwareId(),
        "Software ID for this file");

    std::time_t now;
    std::time(&now);
    std::tm* ptm = std::gmtime(&now);
    uint16_t year = ptm->tm_year + 1900;
    uint16_t doy = ptm->tm_yday;

    options.add("creation_doy", doy, "Day of Year for file");
    options.add("creation_year", year, "4-digit year value for file");
    options.add("extra_dims", "", "Extra dimensions not part of the LAS "
        "point format to be added to each point.");

    return options;
}


void LasWriter::processOptions(const Options& options)
{
    if (options.hasOption("a_srs"))
        setSpatialReference(options.getValueOrDefault("a_srs", std::string()));
    m_lasHeader.setCompressed(options.getValueOrDefault("compression", false));
    m_discardHighReturnNumbers = options.getValueOrDefault(
        "discard_high_return_numbers", false);
    StringList extraDims = options.getValueOrDefault<StringList>("extra_dims");
    m_extraDims = LasUtils::parse(extraDims);

#ifndef PDAL_HAVE_LASZIP
    if (m_lasHeader.compressed())
        throw pdal_error("Can't write LAZ output.  "
            "PDAL not built with LASzip.");
#endif // PDAL_HAVE_LASZIP
    fillForwardList(options);
    getHeaderOptions(options);
    getVlrOptions(options);
}


void LasWriter::prepared(PointTableRef table)
{
    m_extraByteLen = 0;
    for (auto& dim : m_extraDims)
    {
        dim.m_dimType.m_id = table.layout()->findDim(dim.m_name);
        if (dim.m_dimType.m_id == Dimension::Id::Unknown)
        {
            std::ostringstream oss;
            oss << "Dimension '" << dim.m_name << "' specified in "
                "'extra_dim' option not found.";
            throw pdal_error(oss.str());
        }
        m_extraByteLen += Dimension::size(dim.m_dimType.m_type);
    }
}


// Get header info from options and store in map for processing with
// metadata.
void LasWriter::fillForwardList(const Options &options)
{
    StringList forwards = options.getValues("forward");

    StringList header;
    header.push_back("dataformat_id");
    header.push_back("major_version");
    header.push_back("minor_version");
    header.push_back("filesource_id");
    header.push_back("global_encoding");
    header.push_back("project_id");
    header.push_back("system_id");
    header.push_back("software_id");
    header.push_back("creation_doy");
    header.push_back("creation_year");

    StringList scale;
    scale.push_back("scale_x");
    scale.push_back("scale_y");
    scale.push_back("scale_z");

    StringList offset;
    offset.push_back("offset_x");
    offset.push_back("offset_y");
    offset.push_back("offset_z");

    StringList all;
    all.insert(all.begin(), header.begin(), header.end());
    all.insert(all.begin(), scale.begin(), scale.end());
    all.insert(all.begin(), offset.begin(), offset.end());

    // Build the forward list, replacing special keywords with the proper
    // field names.
    for (auto& name : forwards)
    {
        if (name == "all")
        {
            m_forwards.insert(all.begin(), all.end());
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
        else if (Utils::contains(all, name))
            m_forwards.insert(name);
        else
        {
            std::ostringstream oss;

            oss << "Error in 'forward' option.  Unknown field for "
                "forwarding: '" << name << "'.";
            throw pdal_error(oss.str());
        }
    }
}


template<typename T>
void setHeaderOption(const std::string& name, T& headerVal, const Options& ops)
{
    if (ops.hasOption(name))
        headerVal.setVal(ops.getValueOrDefault<typename T::type>(name));
}


// Get header info from options and store in map for processing with
// metadata.
void LasWriter::getHeaderOptions(const Options &options)
{
    setHeaderOption("major_version", m_majorVersion, options);
    setHeaderOption("minor_version", m_minorVersion, options);
    setHeaderOption("dataformat_id", m_dataformatId, options);
    setHeaderOption("format", m_dataformatId, options);
    setHeaderOption("global_encoding", m_globalEncoding, options);
    setHeaderOption("project_id", m_projectId, options);
    setHeaderOption("system_id", m_systemId, options);
    setHeaderOption("software_id", m_softwareId, options);
    setHeaderOption("creation_doy", m_creationDoy, options);
    setHeaderOption("creation_year", m_creationYear, options);
    setHeaderOption("scale_x", m_scaleX, options);
    setHeaderOption("scale_y", m_scaleY, options);
    setHeaderOption("scale_z", m_scaleZ, options);
    setHeaderOption("offset_x", m_offsetX, options);
    setHeaderOption("offset_y", m_offsetY, options);
    setHeaderOption("offset_z", m_offsetZ, options);
}

/// Get VLR-specific options and store for processing with metadata.
/// \param opts  Options to check for VLR info.
void LasWriter::getVlrOptions(const Options& opts)
{
    std::vector<pdal::Option> options = opts.getOptions("vlr");
    for (auto o = options.begin(); o != options.end(); ++o)
    {
        if (!boost::algorithm::istarts_with(o->getName(), "vlr"))
            continue;

        boost::optional<pdal::Options const&> vo = o->getOptions();
        if (!vo)
            continue;

        VlrOptionInfo info;
        info.m_name = o->getName().substr(strlen("vlr"));
        info.m_value = o->getValue<std::string>();
        try
        {
            info.m_recordId = vo->getOption("record_id").getValue<int16_t>();
            info.m_userId = vo->getOption("user_id").getValue<std::string>();
        }
        catch (Option::not_found)
        {
            continue;
        }
        info.m_description = vo->getValueOrDefault<std::string>
            ("description", "");
        m_optionInfos.push_back(info);
    }
}


void LasWriter::readyTable(PointTableRef table)
{
    m_srs = getSpatialReference().empty() ?
        table.spatialRef() : getSpatialReference();

    setVlrsFromSpatialRef();
    setExtraBytesVlr();
    MetadataNode forward = table.privateMetadata("lasforward");
    fillHeader(forward);
    setVlrsFromMetadata(forward);
}


void LasWriter::readyFile(const std::string& filename)
{
    m_error.setFilename(filename);
    std::ostream *out = FileUtils::createFile(filename, true);
    if (!out)
    {
        std::stringstream out;

        out << "writers.las couldn't open file '" << filename <<
            "' for output.";
        throw pdal_error(out.str());
    }
    m_curFilename = filename;
    Utils::writeProgress(m_progressFd, "READYFILE", filename);
    prepOutput(out);
}


void LasWriter::prepOutput(std::ostream *outStream)
{
    m_summaryData.reset(new SummaryData());
    m_ostream = outStream;
    if (m_lasHeader.compressed())
        readyCompression();

    // Write the header.
    OLeStream out(m_ostream);
    out << m_lasHeader;

    m_lasHeader.setVlrOffset((uint32_t)m_ostream->tellp());

    for (auto vi = m_vlrs.begin(); vi != m_vlrs.end(); ++vi)
    {
        VariableLengthRecord& vlr = *vi;
        vlr.write(out, m_lasHeader.versionEquals(1, 0) ? 0xAABB : 0);
    }

    // Write the point data start signature for version 1.0.
    if (m_lasHeader.versionEquals(1, 0))
        out << (uint16_t)0xCCDD;
    m_lasHeader.setPointOffset((uint32_t)m_ostream->tellp());
    if (m_lasHeader.compressed())
        openCompression();

    m_error.setLog(log());
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
        return (boost::algorithm::istarts_with(n.name(), "vlr") &&
            !n.findChild(recPred).empty() &&
            !n.findChild(userPred).empty());
    };
    return node.find(pred);
}


/// Set VLRs from metadata for forwarded info, or from option-provided data
/// otherwise.
void LasWriter::setVlrsFromMetadata(MetadataNode& forward)
{
    std::vector<uint8_t> data;

    if (!m_forwardVlrs)
        return;

    auto pred = [](MetadataNode n)
        { return Utils::startsWith(n.name(), "vlr_"); };

    MetadataNodeList nodes = forward.findChildren(pred);
    for (auto& n : nodes)
    {
        const MetadataNode& userIdNode = n.findChild("user_id");
        const MetadataNode& recordIdNode = n.findChild("record_id");
        if (recordIdNode.valid() && userIdNode.valid())
        {
            data = Utils::base64_decode(n.value());
            uint16_t recordId = (uint16_t)std::stoi(recordIdNode.value());
            addVlr(userIdNode.value(), recordId, n.description(), data);
        }
    }
}


/// Set VLRs from the active spatial reference.
/// \param  srs - Active spatial reference.
void LasWriter::setVlrsFromSpatialRef()
{
    VlrList vlrs;

#ifdef PDAL_HAVE_LIBGEOTIFF
    GeotiffSupport geotiff;
    geotiff.resetTags();

    std::string wkt = m_srs.getWKT(SpatialReference::eCompoundOK, false);
    geotiff.setWkt(wkt);

    addGeotiffVlr(geotiff, GEOTIFF_DIRECTORY_RECORD_ID,
        "GeoTiff GeoKeyDirectoryTag");
    addGeotiffVlr(geotiff, GEOTIFF_DOUBLES_RECORD_ID,
        "GeoTiff GeoDoubleParamsTag");
    addGeotiffVlr(geotiff, GEOTIFF_ASCII_RECORD_ID,
        "GeoTiff GeoAsciiParamsTag");
    addWktVlr();
#endif // PDAL_HAVE_LIBGEOTIFF
}


/// Add a geotiff VLR from the information associated with the record ID.
/// \param  geotiff - Geotiff support structure reference.
/// \param  recordId - Record ID associated with the VLR/Geotiff ref.
/// \param  description - Description to use with the VLR
/// \return  Whether the VLR was added.
bool LasWriter::addGeotiffVlr(GeotiffSupport& geotiff, uint16_t recordId,
    const std::string& description)
{
#ifdef PDAL_HAVE_LIBGEOTIFF
    void *data;
    int count;

    size_t size = geotiff.getKey(recordId, &count, &data);
    if (size == 0)
        return false;

    std::vector<uint8_t> buf(size);
    memcpy(buf.data(), data, size);
    addVlr(TRANSFORM_USER_ID, recordId, description, buf);
    return true;
#else
    return false;
#endif // PDAL_HAVE_LIBGEOTIFF
}


/// Add a Well-known Text VLR associated with the spatial reference.
/// \return  Whether the VLR was added.
bool LasWriter::addWktVlr()
{
    std::string wkt = m_srs.getWKT(SpatialReference::eCompoundOK);
    if (wkt.empty())
        return false;

    std::vector<uint8_t> wktBytes(wkt.begin(), wkt.end());
    // This tacks a NULL to the end of the data, which is required by the spec.
    wktBytes.resize(wktBytes.size() + 1, 0);
    addVlr(TRANSFORM_USER_ID, WKT_RECORD_ID, "OGC Tranformation Record",
        wktBytes);

    // The data in the vector gets moved to the VLR, so we have to recreate it.
    std::vector<uint8_t> wktBytes2(wkt.begin(), wkt.end());
    wktBytes2.resize(wktBytes2.size() + 1, 0);
    addVlr(LIBLAS_USER_ID, WKT_RECORD_ID,
        "OGR variant of OpenGIS WKT SRS", wktBytes2);
    return true;
}


void LasWriter::setExtraBytesVlr()
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
    if (data.size() > VariableLengthRecord::MAX_DATA_SIZE)
    {
        ExtVariableLengthRecord evlr(userId, recordId, description, data);
        m_eVlrs.push_back(std::move(evlr));
    }
    else
    {
        VariableLengthRecord vlr(userId, recordId, description, data);
        m_vlrs.push_back(std::move(vlr));
    }
}

template <typename T>
void LasWriter::handleForward(const std::string& s, T& headerVal,
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

void LasWriter::handleForwards(MetadataNode& forward)
{
    handleForward("major_version", m_majorVersion, forward);
    handleForward("minor_version", m_minorVersion, forward);
    handleForward("dataformat_id", m_dataformatId, forward);
    handleForward("filesource_id", m_filesourceId, forward);
    handleForward("global_encoding", m_globalEncoding, forward);
    handleForward("project_id", m_projectId, forward);
    handleForward("system_id", m_systemId, forward);
    handleForward("software_id", m_softwareId, forward);
    handleForward("creation_doy", m_creationDoy, forward);
    handleForward("creation_year", m_creationYear, forward);

    handleForward("scale_x", m_scaleX, forward);
    handleForward("scale_y", m_scaleY, forward);
    handleForward("scale_z", m_scaleZ, forward);
    handleForward("offset_x", m_offsetX, forward);
    handleForward("offset_y", m_offsetY, forward);
    handleForward("offset_z", m_offsetZ, forward);

    m_xXform.setScale(m_scaleX.val());
    m_yXform.setScale(m_scaleY.val());
    m_zXform.setScale(m_scaleZ.val());
    m_xXform.setOffset(m_offsetX.val());
    m_yXform.setOffset(m_offsetY.val());
    m_zXform.setOffset(m_offsetZ.val());
}

/// Fill the LAS header with values as provided in options or forwarded
/// metadata.
void LasWriter::fillHeader(MetadataNode& forward)
{
    handleForwards(forward);

    const uint16_t WKT_MASK = (1 << 4);

    m_lasHeader.setScale(m_xXform.m_scale, m_yXform.m_scale, m_zXform.m_scale);
    m_lasHeader.setOffset(m_xXform.m_offset, m_yXform.m_offset,
        m_zXform.m_offset);
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
    // We always write a WKT VLR, but we need to be sure to set the WKT
    // bit when the version is at least 1.4.
    uint16_t globalEncoding = m_globalEncoding.val();
    if (m_lasHeader.versionAtLeast(1, 4))
        globalEncoding |= WKT_MASK;
    m_lasHeader.setGlobalEncoding(globalEncoding);

    if (!m_lasHeader.pointFormatSupported())
    {
        std::ostringstream oss;
        oss << "Unsupported LAS output point format: " <<
            (int)m_lasHeader.pointFormat() << ".";
        throw pdal_error(oss.str());
    }
}


void LasWriter::readyCompression()
{
#ifdef PDAL_HAVE_LASZIP
    m_zipPoint.reset(new ZipPoint(m_lasHeader.pointFormat(),
        m_lasHeader.pointLen()));
    m_zipper.reset(new LASzipper());
    // Note: this will make the VLR count in the header incorrect, but we
    // rewrite that bit in finishOutput() to fix it up.
    std::vector<uint8_t> data = m_zipPoint->vlrData();
    addVlr(LASZIP_USER_ID, LASZIP_RECORD_ID, "http://laszip.org", data);
#endif
}


/// Prepare the compressor to write points.
/// \param  pointFormat - Formt of points we're writing.
void LasWriter::openCompression()
{
#ifdef PDAL_HAVE_LASZIP
    if (!m_zipper->open(*m_ostream, m_zipPoint->GetZipper()))
    {
        std::ostringstream oss;
        const char* err = m_zipper->get_error();
        if (err == NULL)
            err = "(unknown error)";
        oss << "Error opening LASzipper: " << std::string(err);
        throw pdal_error(oss.str());
    }
#endif
}


void LasWriter::writeView(const PointViewPtr view)
{
    Utils::writeProgress(m_progressFd, "READYVIEW",
        std::to_string(view->size()));
    setAutoXForm(view);

    size_t pointLen = m_lasHeader.pointLen();

    // Make a buffer of at most a meg.
    std::vector<char> buf(std::min((size_t)1000000, pointLen * view->size()));

    const PointView& viewRef(*view.get());

    //ABELL - Removed callback handling for now.
    point_count_t remaining = view->size();
    PointId idx = 0;
    while (remaining)
    {
        point_count_t filled = fillWriteBuf(viewRef, idx, buf);
        idx += filled;
        remaining -= filled;

#ifdef PDAL_HAVE_LASZIP
        if (m_lasHeader.compressed())
        {
            char *pos = buf.data();
            for (point_count_t i = 0; i < filled; i++)
            {
                memcpy(m_zipPoint->m_lz_point_data.data(), pos, pointLen);
                if (!m_zipper->write(m_zipPoint->m_lz_point))
                {
                    std::ostringstream oss;
                    const char* err = m_zipper->get_error();
                    if (err == NULL)
                        err = "(unknown error)";
                    oss << "Error writing point: " << std::string(err);
                    throw pdal_error(oss.str());
                }
                pos += pointLen;
            }
        }
        else
            m_ostream->write(buf.data(), filled * pointLen);
#else
        m_ostream->write(buf.data(), filled * pointLen);
#endif
    }
    Utils::writeProgress(m_progressFd, "DONEVIEW",
        std::to_string(view->size()));
}


point_count_t LasWriter::fillWriteBuf(const PointView& view,
    PointId startId, std::vector<char>& buf)
{
    point_count_t blocksize = buf.size() / m_lasHeader.pointLen();
    blocksize = std::min(blocksize, view.size() - startId);

    bool has14Format = m_lasHeader.has14Format();
    bool hasColor = m_lasHeader.hasColor();
    bool hasTime = m_lasHeader.hasTime();
    bool hasInfrared = m_lasHeader.hasInfrared();

    PointId lastId = startId + blocksize;
    static const size_t maxReturnCount = m_lasHeader.maxReturnCount();
    LeInserter ostream(buf.data(), buf.size());
    for (PointId idx = startId; idx < lastId; idx++)
    {
        // we always write the base fields
        using namespace Dimension;

        uint8_t returnNumber(1);
        uint8_t numberOfReturns(1);
        if (view.hasDim(Id::ReturnNumber))
        {
            returnNumber = view.getFieldAs<uint8_t>(Id::ReturnNumber,
                idx);
            if (returnNumber < 1 || returnNumber > maxReturnCount)
                m_error.returnNumWarning(returnNumber);
        }
        if (view.hasDim(Id::NumberOfReturns))
            numberOfReturns = view.getFieldAs<uint8_t>(
                Id::NumberOfReturns, idx);
        if (numberOfReturns == 0)
            m_error.numReturnsWarning(0);
        if (numberOfReturns > maxReturnCount)
        {
            if (m_discardHighReturnNumbers)
            {
                // If this return number is too high, pitch the point.
                if (returnNumber > maxReturnCount)
                    continue;
                numberOfReturns = maxReturnCount;
            }
            else
                m_error.numReturnsWarning(numberOfReturns);
        }

        double xOrig = view.getFieldAs<double>(Id::X, idx);
        double yOrig = view.getFieldAs<double>(Id::Y, idx);
        double zOrig = view.getFieldAs<double>(Id::Z, idx);

        double x = (xOrig - m_xXform.m_offset) / m_xXform.m_scale;
        double y = (yOrig - m_yXform.m_offset) / m_yXform.m_scale;
        double z = (zOrig - m_zXform.m_offset) / m_zXform.m_scale;

        auto converter = [this](double d, Dimension::Id::Enum dim) -> int32_t
        {
            int32_t i;

            if (!Utils::numericCast(d, i))
            {
                std::ostringstream oss;
                oss << "Unable to convert scaled value (" << d << ") to "
                    "int32 for dimension '" << Dimension::name(dim) <<
                    "' when writing LAS/LAZ file " << m_curFilename << ".";
                throw pdal_error(oss.str());
            }
            return i;
        };

        ostream << converter(x, Id::X);
        ostream << converter(y, Id::Y);
        ostream << converter(z, Id::Z);

        uint16_t intensity = 0;
        if (view.hasDim(Id::Intensity))
            intensity = view.getFieldAs<uint16_t>(Id::Intensity, idx);
        ostream << intensity;

        uint8_t scanChannel(0);
        if (view.hasDim(Id::ScanChannel))
            scanChannel = view.getFieldAs<uint8_t>(Id::ScanChannel, idx);

        uint8_t scanDirectionFlag(0);
        if (view.hasDim(Id::ScanDirectionFlag))
            scanDirectionFlag = view.getFieldAs<uint8_t>(
                Id::ScanDirectionFlag, idx);

        uint8_t edgeOfFlightLine(0);
        if (view.hasDim(Id::EdgeOfFlightLine))
            edgeOfFlightLine = view.getFieldAs<uint8_t>(
                Id::EdgeOfFlightLine, idx);

        if (has14Format)
        {
            uint8_t bits = returnNumber | (numberOfReturns << 4);
            ostream << bits;
            bits = (scanChannel << 4) | (scanDirectionFlag << 6) |
                (edgeOfFlightLine << 7);
            ostream << bits;
        }
        else
        {
            uint8_t bits = returnNumber | (numberOfReturns << 3) |
                (scanDirectionFlag << 6) | (edgeOfFlightLine << 7);
            ostream << bits;
        }

        uint8_t classification = 0;
        if (view.hasDim(Id::Classification))
        {
            classification = view.getFieldAs<uint8_t>(Id::Classification,
                idx);
        }
        ostream << classification;

        uint8_t userData = 0;
        if (view.hasDim(Id::UserData))
            userData = view.getFieldAs<uint8_t>(Id::UserData, idx);
        if (has14Format)
        {
            int16_t scanAngleRank = 0;
            if (view.hasDim(Id::ScanAngleRank))
                scanAngleRank =
                    view.getFieldAs<float>(Id::ScanAngleRank, idx) / .006;
            ostream << userData << scanAngleRank;
        }
        else
        {
            int8_t scanAngleRank = 0;
            if (view.hasDim(Id::ScanAngleRank))
                scanAngleRank = view.getFieldAs<int8_t>(Id::ScanAngleRank, idx);

            ostream << scanAngleRank << userData;
        }

        uint16_t pointSourceId = 0;
        if (view.hasDim(Id::PointSourceId))
            pointSourceId = view.getFieldAs<uint16_t>(Id::PointSourceId,
                idx);
        ostream << pointSourceId;

        if (hasTime)
        {
            double t = 0.0;
            if (view.hasDim(Id::GpsTime))
                t = view.getFieldAs<double>(Id::GpsTime, idx);
            ostream << t;
        }

        if (hasColor)
        {
            uint16_t red = 0;
            uint16_t green = 0;
            uint16_t blue = 0;
            if (view.hasDim(Id::Red))
                red = view.getFieldAs<uint16_t>(Id::Red, idx);
            if (view.hasDim(Id::Green))
                green = view.getFieldAs<uint16_t>(Id::Green, idx);
            if (view.hasDim(Id::Blue))
                blue = view.getFieldAs<uint16_t>(Id::Blue, idx);

            ostream << red << green << blue;
        }

        if (hasInfrared)
        {
            uint16_t nearInfraRed = 0;

            if (view.hasDim(Id::Infrared))
                nearInfraRed = view.getFieldAs<uint16_t>(Id::Infrared, idx);
            ostream << nearInfraRed;
        }

        Everything e;
        for (auto& dim : m_extraDims)
        {
            view.getField((char *)&e, dim.m_dimType.m_id,
                dim.m_dimType.m_type, idx);
            ostream.put(dim.m_dimType.m_type, e);
        }

        using namespace Dimension;
        m_summaryData->addPoint(xOrig, yOrig, zOrig, returnNumber);
    }
    return blocksize;
}


void LasWriter::doneFile()
{
    finishOutput();
    Utils::writeProgress(m_progressFd, "DONEFILE", m_curFilename);
    m_curFilename.clear();
    delete m_ostream;
    m_ostream = NULL;
}


void LasWriter::finishOutput()
{
    //ABELL - The zipper has to be closed right after all the points
    // are written or bad things happen since this call expects the
    // stream to be positioned at a particular position.
#ifdef PDAL_HAVE_LASZIP
    if (m_lasHeader.compressed())
        m_zipper->close();
#endif

    log()->get(LogLevel::Debug) << "Wrote " <<
        m_summaryData->getTotalNumPoints() <<
        " points to the LAS file" << std::endl;

    OLeStream out(m_ostream);

    for (auto vi = m_eVlrs.begin(); vi != m_eVlrs.end(); ++vi)
    {
        ExtVariableLengthRecord evlr = *vi;
        out << evlr;
    }

    // Reset the offset/scale since it may have been auto-computed
    m_lasHeader.setOffset(m_xXform.m_offset, m_yXform.m_offset,
        m_zXform.m_offset);
    m_lasHeader.setScale(m_xXform.m_scale, m_yXform.m_scale,
        m_zXform.m_scale);

    // The summary is calculated as points are written.
    m_lasHeader.setSummary(*m_summaryData);
    // VLR count may change as LAS records are written.
    m_lasHeader.setVlrCount(m_vlrs.size());

    out.seek(0);
    out << m_lasHeader;
    out.seek(m_lasHeader.pointOffset());

#ifdef PDAL_HAVE_LASZIP
    if (m_lasHeader.compressed())
    {
        m_zipper.reset();
        m_zipPoint.reset();
    }
#endif
    m_ostream->flush();
}

} // namespace pdal
