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

#include <boost/uuid/uuid_generators.hpp>
#include <iostream>

#include <pdal/Charbuf.hpp>
#include <pdal/OStream.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Utils.hpp>

#include "GeotiffSupport.hpp"
#include "ZipPoint.hpp"

namespace pdal
{

void LasWriter::construct()
{
    m_xXform.m_scale = .01;
    m_yXform.m_scale = .01;
    m_zXform.m_scale = .01;
    m_numPointsWritten = 0;
    m_streamOffset = 0;
}


void LasWriter::flush()
{
#ifdef PDAL_HAVE_LASZIP
    if (m_lasHeader.compressed())
    {
        m_zipper.reset();
        m_zipPoint.reset();
    }
#endif
    m_ostream->flush();
}


Options LasWriter::getDefaultOptions()
{
    Options options;

    options.add("filename", "", "Name of the file for LAS/LAZ output.");
    options.add("compression", false, "Do we LASzip-compress the data?");
    options.add("format", 3, "Point format to write");
    options.add("major_version", 1, "LAS Major version");
    options.add("minor_version", 2, "LAS Minor version");
    options.add("creation_doy", 0, "Day of Year for file");
    options.add("creation_year", 2011, "4-digit year value for file");

    LasHeader header;
    options.add("system_id", header.getSystemIdentifier(),
        "System ID for this file");
    options.add("software_id", GetDefaultSoftwareId(),
        "Software ID for this file");
    options.add("filesource_id", 0, "File Source ID for this file");
    options.add("forward_metadata", false, "forward metadata into "
        "the file as necessary");

    return options;
}


void LasWriter::processOptions(const Options& options)
{
    if (options.hasOption("a_srs"))
        setSpatialReference(options.getValueOrDefault("a_srs", std::string()));
    m_lasHeader.setCompressed(options.getValueOrDefault("compression", false));
    m_discardHighReturnNumbers = options.getValueOrDefault(
        "discard_high_return_numbers", false);

    getHeaderOptions(options);
    getVlrOptions(options);
    m_error.setFilename(m_filename);
}


// Get header info from options and store in map for processing with
// metadata.
void LasWriter::getHeaderOptions(const Options &options)
{
    typedef boost::optional<std::string> OpString;
    auto metaOptionValue = [this, options](const std::string& name,
        const::std::string& defVal)
    {
        std::string value;
        OpString opValue = options.getMetadataOption<std::string>(name);
        if (opValue)
        {
            value = *opValue;
            // The reassignment makes sure the case is correct.
            if (boost::iequals(value, "FORWARD"))
            {
                value = "FORWARD";
                value += defVal;
            }
        }
        else
            value = options.getValueOrDefault(name, defVal);
        m_headerVals[name] = value;
    };

    std::time_t now;
    std::time(&now);
    std::tm* ptm = std::gmtime(&now);
    uint16_t year = ptm->tm_year + 1900;
    uint16_t doy = ptm->tm_yday;

    metaOptionValue("format", "3");
    metaOptionValue("minor_version", "2");
    metaOptionValue("creation_year", std::to_string(year));
    metaOptionValue("creation_doy", std::to_string(doy));
    metaOptionValue("software_id", GetDefaultSoftwareId());
    LasHeader header;
    metaOptionValue("system_id", header.getSystemIdentifier());
    metaOptionValue("project_id",
        boost::lexical_cast<std::string>(boost::uuids::uuid()));
    metaOptionValue("global_encoding", "0");
    metaOptionValue("filesource_id", "0");
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
        catch (option_not_found err)
        {
            continue;
        }
        info.m_description = vo->getValueOrDefault<std::string>
            ("description", "");
        m_optionInfos.push_back(info);
    }
}


void LasWriter::ready(PointContextRef ctx)
{
    const SpatialReference& srs = getSpatialReference().empty() ?
        ctx.spatialRef() : getSpatialReference();

    if (!m_ostream)
        m_ostream = FileUtils::createFile(m_filename, true);
    setVlrsFromMetadata();
    setVlrsFromSpatialRef(srs);
    fillHeader(ctx);

    if (m_lasHeader.compressed())
        readyCompression();

    // Write the header.
    m_ostream->seekp(m_streamOffset);
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
void LasWriter::setVlrsFromMetadata()
{
    std::vector<uint8_t> data;

    for (auto oi = m_optionInfos.begin(); oi != m_optionInfos.end(); ++oi)
    {
        VlrOptionInfo& vlrInfo = *oi;

        if (vlrInfo.m_name == "FORWARD")
        {
            MetadataNode m = findVlrMetadata(m_metadata, vlrInfo.m_recordId,
                vlrInfo.m_userId);
            if (m.empty())
                continue;
            data = Utils::base64_decode(m.value());
        }
        else
            data = Utils::base64_decode(vlrInfo.m_value);
        addVlr(vlrInfo.m_userId, vlrInfo.m_recordId, vlrInfo.m_description,
            data);
    }
}


/// Set VLRs from the active spatial reference.
/// \param  srs - Active spatial reference.
void LasWriter::setVlrsFromSpatialRef(const SpatialReference& srs)
{
    VlrList vlrs;

#ifdef PDAL_HAVE_LIBGEOTIFF
    GeotiffSupport geotiff;
    geotiff.resetTags();

    std::string wkt = srs.getWKT(SpatialReference::eCompoundOK, false);
    geotiff.setWkt(wkt);

    addGeotiffVlr(geotiff, GEOTIFF_DIRECTORY_RECORD_ID,
        "GeoTiff GeoKeyDirectoryTag");
    addGeotiffVlr(geotiff, GEOTIFF_DOUBLES_RECORD_ID,
        "GeoTiff GeoDoubleParamsTag");
    addGeotiffVlr(geotiff, GEOTIFF_ASCII_RECORD_ID,
        "GeoTiff GeoAsciiParamsTag");
    addWktVlr(srs);
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
/// \param  srs - Associated spatial reference.
/// \return  Whether the VLR was added.
bool LasWriter::addWktVlr(const SpatialReference& srs)
{
    std::string wkt = srs.getWKT(SpatialReference::eCompoundOK);
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


/// Fill the LAS header with values as provided in options or forwarded
/// metadata.
void LasWriter::fillHeader(PointContextRef ctx)
{
    m_lasHeader.setScale(m_xXform.m_scale, m_yXform.m_scale,
        m_zXform.m_scale);
    m_lasHeader.setOffset(m_xXform.m_offset, m_yXform.m_offset,
        m_zXform.m_offset);
    m_lasHeader.setVlrCount(m_vlrs.size());
    m_lasHeader.setEVlrCount(m_eVlrs.size());

    m_lasHeader.setPointFormat((uint8_t)headerVal<unsigned>("format"));
    m_lasHeader.setPointLen(m_lasHeader.basePointLen());
    m_lasHeader.setVersionMinor((uint8_t)headerVal<unsigned>("minor_version"));
    m_lasHeader.setCreationYear(headerVal<uint16_t>("creation_year"));
    m_lasHeader.setCreationDOY(headerVal<uint16_t>("creation_doy"));
    m_lasHeader.setSoftwareId(headerVal<std::string>("software_id"));
    m_lasHeader.setSystemId(headerVal<std::string>("system_id"));
    m_lasHeader.setProjectId(headerVal<boost::uuids::uuid>("project_id"));
    m_lasHeader.setGlobalEncoding(headerVal<uint16_t>("global_encoding"));
    m_lasHeader.setFileSourceId(headerVal<uint16_t>("filesource_id"));

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
    // rewrite that bit in done() to fix it up.
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
#else
    throw pdal_error("LASzip compression is not enabled for "
        "this compressed file!");
#endif
}


void LasWriter::write(const PointBuffer& pointBuffer)
{
    uint32_t numValidPoints = 0;

    setAutoOffset(pointBuffer);

    std::vector<char> buf(m_lasHeader.pointLen());

    bool hasColor = m_lasHeader.hasColor();
    bool hasTime = m_lasHeader.hasTime();
    uint16_t record_length = m_lasHeader.pointLen();

    m_callback->setTotal(pointBuffer.size());
    m_callback->invoke(0);

    static const size_t maxReturnCount = m_lasHeader.maxReturnCount();
    for (PointId idx = 0; idx < pointBuffer.size(); idx++)
    {
        Charbuf charstreambuf(buf.data(), buf.size());

        std::ostream stream(&charstreambuf);

        // Wrap the output stream with byte ordering
        OLeStream ostream(&stream);

        // we always write the base fields
        using namespace Dimension;

        uint8_t returnNumber(1);
        uint8_t numberOfReturns(1);
        if (pointBuffer.hasDim(Id::ReturnNumber))
        {
            returnNumber = pointBuffer.getFieldAs<uint8_t>(Id::ReturnNumber,
                idx);
            if (returnNumber < 1 || returnNumber > maxReturnCount)
                m_error.returnNumWarning(returnNumber);
        }
        if (pointBuffer.hasDim(Id::NumberOfReturns))
            numberOfReturns = pointBuffer.getFieldAs<uint8_t>(
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

        double x = pointBuffer.getFieldAs<double>(Id::X, idx);
        double y = pointBuffer.getFieldAs<double>(Id::Y, idx);
        double z = pointBuffer.getFieldAs<double>(Id::Z, idx);

        x = (x - m_xXform.m_offset) / m_xXform.m_scale;
        y = (y - m_yXform.m_offset) / m_yXform.m_scale;
        z = (z - m_zXform.m_offset) / m_zXform.m_scale;

        ostream << boost::numeric_cast<int32_t>(lround(x));
        ostream << boost::numeric_cast<int32_t>(lround(y));
        ostream << boost::numeric_cast<int32_t>(lround(z));

        uint16_t intensity = 0;
        if (pointBuffer.hasDim(Id::Intensity))
            intensity = pointBuffer.getFieldAs<uint16_t>(Id::Intensity, idx);
        ostream << intensity;

        uint8_t scanDirectionFlag(0);
        if (pointBuffer.hasDim(Id::ScanDirectionFlag))
            scanDirectionFlag = pointBuffer.getFieldAs<uint8_t>(
                Id::ScanDirectionFlag, idx);

        uint8_t edgeOfFlightLine(0);
        if (pointBuffer.hasDim(Id::EdgeOfFlightLine))
            edgeOfFlightLine = pointBuffer.getFieldAs<uint8_t>(
                Id::EdgeOfFlightLine, idx);

        uint8_t bits = returnNumber | (numberOfReturns<<3) |
            (scanDirectionFlag << 6) | (edgeOfFlightLine << 7);
        ostream << bits;

        uint8_t classification = 0;
        if (pointBuffer.hasDim(Id::Classification))
            classification = pointBuffer.getFieldAs<uint8_t>(Id::Classification,
                idx);
        ostream << classification;

        int8_t scanAngleRank = 0;
        if (pointBuffer.hasDim(Id::ScanAngleRank))
            scanAngleRank = pointBuffer.getFieldAs<int8_t>(Id::ScanAngleRank,
                idx);
        ostream << scanAngleRank;

        uint8_t userData = 0;
        if (pointBuffer.hasDim(Id::UserData))
            userData = pointBuffer.getFieldAs<uint8_t>(Id::UserData, idx);
        ostream << userData;

        uint16_t pointSourceId = 0;
        if (pointBuffer.hasDim(Id::PointSourceId))
            pointSourceId = pointBuffer.getFieldAs<uint16_t>(Id::PointSourceId,
                idx);
        ostream << pointSourceId;

        if (hasTime)
        {
            double t = 0.0;
            if (pointBuffer.hasDim(Id::GpsTime))
                t = pointBuffer.getFieldAs<double>(Id::GpsTime, idx);
            ostream << t;
        }

        if (hasColor)
        {
            uint16_t red = 0;
            uint16_t green = 0;
            uint16_t blue = 0;
            if (pointBuffer.hasDim(Id::Red))
                red = pointBuffer.getFieldAs<uint16_t>(Id::Red, idx);
            if (pointBuffer.hasDim(Id::Green))
                green = pointBuffer.getFieldAs<uint16_t>(Id::Green, idx);
            if (pointBuffer.hasDim(Id::Blue))
                blue = pointBuffer.getFieldAs<uint16_t>(Id::Blue, idx);

            ostream << red << green << blue;
        }

        //Buffer is complete.

#ifdef PDAL_HAVE_LASZIP
        if (m_lasHeader.compressed())
        {
            // If we're going to compress, do it.
            for (unsigned i = 0; i < m_zipPoint->m_lz_point_size; i++)
                m_zipPoint->m_lz_point_data[i] = buf[i];
            if (!m_zipper->write(m_zipPoint->m_lz_point))
            {
                std::ostringstream oss;
                const char* err = m_zipper->get_error();
                if (err == NULL)
                    err = "(unknown error)";
                oss << "Error writing point: " << std::string(err);
                throw pdal_error(oss.str());
            }
        }
        else
           m_ostream->write(buf.data(), buf.size());
#else
        m_ostream->write(buf.data(), buf.size());
#endif
        ++numValidPoints;

        double xValue = pointBuffer.getFieldAs<double>(Id::X, idx);
        double yValue = pointBuffer.getFieldAs<double>(Id::Y, idx);
        double zValue = pointBuffer.getFieldAs<double>(Id::Z, idx);
        m_summaryData.addPoint(xValue, yValue, zValue, returnNumber);

        // Perhaps the interval should come out of the callback itself.
        if (idx % 1000 == 0)
            m_callback->invoke(idx + 1);
    }
    m_callback->invoke(pointBuffer.size());

    m_numPointsWritten = m_numPointsWritten + numValidPoints;
}


void LasWriter::done(PointContextRef ctx)
{
    //ABELL - The zipper has to be closed right after all the points
    // are written or bad things happen since this call expects the
    // stream to be positioned at a particular position.
#ifdef PDAL_HAVE_LASZIP
    if (m_lasHeader.compressed())
        m_zipper->close();
#endif

    log()->get(LogLevel::Debug) << "Wrote " << m_numPointsWritten <<
        " points to the LAS file" << std::endl;

    OLeStream out(m_ostream);

    for (auto vi = m_eVlrs.begin(); vi != m_eVlrs.end(); ++vi)
    {
        ExtVariableLengthRecord evlr = *vi;
        out << evlr;
    }

    // Reset the offset since it may have been auto-computed
    m_lasHeader.setOffset(m_xXform.m_offset, m_yXform.m_offset,
        m_zXform.m_offset);
    // We didn't know the point count until we go through the points.
    m_lasHeader.setPointCount(m_numPointsWritten);
    // The summary is calculated as points are written.
    m_lasHeader.setSummary(m_summaryData);
    // VLR count may change as LAS records are written.
    m_lasHeader.setVlrCount(m_vlrs.size());

    out.seek(m_streamOffset);
    out << m_lasHeader;
    out.seek(m_lasHeader.pointOffset());
}

} // namespace pdal
