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

#include <boost/property_tree/xml_parser.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <iostream>

#include <pdal/drivers/las/Writer.hpp>

#include "LasHeaderWriter.hpp"

#include "ZipPoint.hpp"

#ifdef PDAL_HAVE_LASZIP
#include <laszip/laszipper.hpp>
#endif

#include <pdal/Stage.hpp>
#include <pdal/PointBuffer.hpp>

namespace pdal
{
namespace drivers
{
namespace las
{

namespace
{
    const uint8_t MAX_RETURN_NUMBER = 5;
}


Writer::Writer(const Options& options)
    : pdal::Writer(options)
    , m_streamManager(options.getOption("filename").getValue<std::string>())
{
    Construct();
}


Writer::Writer(std::ostream *ostream) :
    m_streamManager(ostream)
{
    Construct();
}


Writer::Writer(const Options& options, std::ostream *ostream) :
    pdal::Writer(options), m_streamManager(ostream)
{
    Construct();
}


void Writer::Construct()
{
    m_numPointsWritten = 0;
    m_headerInitialized = false;
    m_streamOffset = 0;
    setOptions();
}


void Writer::processOptions(const Options& options)
{
    setGeneratingSoftware(options.getValueOrDefault("software_id",
        GetDefaultSoftwareId()));

    m_lasHeader.SetCreationDOY((uint16_t)options.getValueOrDefault(
        "creation_doy", 0U));
    m_lasHeader.SetCreationYear((uint16_t)options.getValueOrDefault(
        "creation_year", 0U));
    m_lasHeader.setPointFormat(static_cast<PointFormat>(
        options.getValueOrDefault("format", 3U)));
    m_lasHeader.SetSystemId(options.getValueOrDefault<std::string>("system_id",
        LasHeader::SystemIdentifier));

    m_lasHeader.SetHeaderPadding(options.getValueOrDefault(
        "header_padding", 0U));
    if (options.hasOption("a_srs"))
        setSpatialReference(options.getValueOrDefault("a_srs", std::string()));
    m_lasHeader.SetCompressed(options.getValueOrDefault("compression", false));
    m_lasHeader.SetFileSourceId(options.getValueOrDefault<uint16_t>(
        "filesource_id", 0));
    try
    {
        m_lasHeader.SetDataRecordLength(options.getValueOrThrow<uint16_t>(
            "datarecordlength"));
    }
    catch (pdal::option_not_found&) {};
    
    m_discardHighReturnNumbers = options.getValueOrDefault(
            "discard_high_return_numbers", false);
}


void Writer::setOptions()
{

}


Writer::~Writer()
{
#ifdef PDAL_HAVE_LASZIP
    m_zipper.reset();
    m_zipPoint.reset();
#endif
    m_streamManager.close();
}

void Writer::initialize()
{
    m_streamManager.open();
}

Options Writer::getDefaultOptions()
{
    Options options;

    Option filename("filename", "", "file to read from");
    Option compression("compression", false, "Do we LASzip-compress the data?");
    Option format("format", PointFormat3, "Point format to write");
    Option major_version("major_version", 1, "LAS Major version");
    Option minor_version("minor_version", 2, "LAS Minor version");
    Option day_of_year("creation_doy", 0, "Day of Year for file");
    Option year("creation_year", 2011, "4-digit year value for file");
    Option system_id("system_id", LasHeader::SystemIdentifier,
        "System ID for this file");
    Option software_id("software_id", GetDefaultSoftwareId(),
        "Software ID for this file");
    Option filesourceid("filesource_id", 0, "File Source ID for this file");
    Option header_padding("header_padding", 0, "Header padding (space between "
        "end of VLRs and beginning of point data)");
    Option set_metadata("forward_metadata", false, "forward metadata into "
        "the file as necessary");

    options.add(major_version);
    options.add(minor_version);
    options.add(day_of_year);
    options.add(year);
    options.add(system_id);
    options.add(software_id);
    options.add(header_padding);
    options.add(format);
    options.add(filename);
    options.add(compression);
    return options;
}

void Writer::setCompressed(bool v)
{
    m_lasHeader.SetCompressed(v);
}

void Writer::setFormatVersion(boost::uint8_t majorVersion,
    boost::uint8_t minorVersion)
{
    m_lasHeader.SetVersionMajor(majorVersion);
    m_lasHeader.SetVersionMinor(minorVersion);
}

void Writer::setPointFormat(PointFormat pointFormat)
{
    m_lasHeader.setPointFormat(pointFormat);
}

void Writer::setDate(boost::uint16_t dayOfYear, boost::uint16_t year)
{
    m_lasHeader.SetCreationDOY(dayOfYear);
    m_lasHeader.SetCreationYear(year);
}

void Writer::setProjectId(const boost::uuids::uuid& id)
{
    m_lasHeader.SetProjectId(id);
}

void Writer::setSystemIdentifier(const std::string& systemId)
{
    m_lasHeader.SetSystemId(systemId);
}

void Writer::setGeneratingSoftware(const std::string& softwareId)
{
    m_lasHeader.SetSoftwareId(softwareId);
}

void Writer::setHeaderPadding(boost::uint32_t const& v)
{
    m_lasHeader.SetHeaderPadding(v);
}


MetadataNode Writer::findVlr(MetadataNode node, const std::string& recordId,
    const std::string& userId)
{
    // Find a node whose name starts with vlr and that has child nodes
    // with the name and recordId we're looking for.
    auto pred = [recordId,userId](MetadataNode n)
    {
        auto recPred = [recordId](MetadataNode n)
        {
            return n.name() == "record_id" &&
                n.value() == recordId;
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

void Writer::setVLRsFromMetadata(LasHeader& header, MetadataNode metaNode,
    Options const& opts)
{
    using namespace boost::property_tree;

    std::vector<pdal::Option> options = opts.getOptions("vlr");
    std::vector<pdal::Option>::const_iterator o;

    for (o = options.begin(); o != options.end(); ++o)
    {
        if (!boost::algorithm::istarts_with(o->getName(), "vlr"))
            continue;

        boost::optional<pdal::Options const&> vo = o->getOptions();
        if (!vo)
            throw pdal_error("VLR option given, but no sub options "
                "available to specify which VLRs to copy!");

        if (boost::algorithm::iequals(o->getValue<std::string>(), "FORWARD"))
        {
            uint16_t recordId =
                vo->getOption("record_id").getValue<uint16_t>();
            std::string userId =
                vo->getOption("user_id").getValue<std::string>();

            MetadataNode node = findVlr(metaNode,
                boost::lexical_cast<std::string>(recordId), userId);
            if (node.empty())
                continue;

            std::vector<uint8_t> data = Utils::base64_decode(node.value());
            VariableLengthRecord vlr(0xAABB, userId, recordId,
                node.description(), data.data(), (uint16_t)data.size());
            header.getVLRs().add(vlr);
        }
        else
        {
            boost::uint16_t record_id =
                vo->getValueOrDefault<uint16_t>("record_id", 4321);
            std::string user_id = vo->getValueOrDefault<std::string>("user_id",
                "PDALUSERID");
            std::string description = vo->getValueOrDefault<std::string>(
                "description", "PDAL default VLR description");
            std::vector<uint8_t> data = Utils::base64_decode(
                o->getValue<std::string>());
            pdal::drivers::las::VariableLengthRecord vlr(0xAABB,
                    user_id,
                    record_id,
                    description,
                    &data[0],
                    static_cast<boost::uint16_t>(data.size()));
            header.getVLRs().add(vlr);
            log()->get(LogLevel::Debug) << "Setting VLR from metadata with "
                "user_id='" << user_id << "' and record_id='" << record_id <<
                "'"<< " with size: " << data.size() << std::endl;
        }
    }
}


void Writer::ready(PointContext ctx)
{
    if (m_headerInitialized)
        return;

    m_streamOffset = m_streamManager.ostream().tellp();

    m_lasHeader.SetScale(m_xXform.m_scale, m_yXform.m_scale, m_zXform.m_scale);
    m_lasHeader.SetOffset(m_xXform.m_offset, m_yXform.m_offset,
        m_zXform.m_offset);

    m_lasHeader.setSpatialReference(getSpatialReference().empty() ?
        ctx.spatialRef() : getSpatialReference());

    bool useMetadata = false;
    try
    {
        getOptions().getOption("metadata");
        useMetadata = true;
    }
    catch (pdal::option_not_found&)
    {}

    if (useMetadata)
    {
        // Default to PointFormat 3 if not forwarded from a previous metadata
        // or given in a metadata option
        uint32_t v = getMetadataOption<uint32_t>(getOptions(), m_metadata,
            "dataformat_id", 3);
        uint32_t v2 = getMetadataOption<uint32_t>(getOptions(), m_metadata,
            "format", 3);
            
        // Use the 'format' option specified by the options instead of the
        // metadata one that was set passively
        v = v2; 
        setPointFormat(static_cast<PointFormat>(v));
        log()->get(LogLevel::Debug) << "Setting point format to " << v <<
            " from metadata " << std::endl;
        
        uint32_t minor = getMetadataOption<uint32_t>(getOptions(),
            m_metadata, "minor_version", 2);

        setFormatVersion(1, static_cast<boost::uint8_t>(minor));
        log()->get(LogLevel::Debug) << "Setting version to "
                             << "1." << minor
                             << " from metadata " << std::endl;

        std::time_t now;
        std::time(&now);
        std::tm* ptm = std::gmtime(&now);
        boost::uint32_t day(0);
        boost::uint32_t year(0);
        if (0 != ptm)
        {
            day = static_cast<uint16_t>(ptm->tm_yday);
            year = static_cast<uint16_t>(ptm->tm_year + 1900);
        }

        year = getMetadataOption<uint32_t>(getOptions(), m_metadata,
            "creation_year", year);
        day = getMetadataOption<uint32_t>(getOptions(), m_metadata,
            "creation_doy", day);

        setDate(static_cast<uint16_t>(day), static_cast<uint16_t>(year));
        log()->get(LogLevel::Debug) << "Setting date to format " << day <<
            "/" << year << " from metadata " << std::endl;

        std::string software_id = getMetadataOption<std::string>(getOptions(),
            m_metadata, "software_id", GetDefaultSoftwareId());
        setGeneratingSoftware(software_id);
        log()->get(LogLevel::Debug) << "Setting generating software to '" <<
            software_id << "' from metadata " << std::endl;

        std::string system_id = getMetadataOption<std::string>(getOptions(),
            m_metadata, "system_id", LasHeader::SystemIdentifier);
        setSystemIdentifier(system_id);
        log()->get(LogLevel::Debug) << "Setting system identifier to " <<
            system_id << " from metadata " << std::endl;

        boost::uuids::uuid project_id =
            getMetadataOption<boost::uuids::uuid>(getOptions(),
            m_metadata, "project_id", boost::uuids::nil_uuid());
        m_lasHeader.SetProjectId(project_id);
        log()->get(LogLevel::Debug) << "Setting project_id to " << project_id <<
            " from metadata " << std::endl;
        
        std::string global_encoding_data = getMetadataOption<std::string>(
            getOptions(), m_metadata, "global_encoding", "");
        std::vector<uint8_t> data = Utils::base64_decode(global_encoding_data);
        
        uint16_t reserved = 0;
        if (global_encoding_data.size())
        {
            if (data.size() == 0)
                ;
            else if (data.size() == 1 )
                reserved = data[0];
            else if (data.size() == 2 )
                memcpy(&reserved, data.data(), data.size());
            else if (data.size() == 4 )
            {
                uint32_t temp;
                memcpy(&temp, data.data(), data.size());
                reserved = static_cast<uint16_t>(temp);
            }
            else if (data.size() == 8 )
            {
                uint64_t temp;
                memcpy(&temp, data.data(), data.size());
                reserved = static_cast<uint16_t>(temp);
            }
            else 
            {
                std::ostringstream oss;
                oss << "size of global_encoding bytes should == 2, not " <<
                    data.size();
                throw pdal_error(oss.str());                
            }
        }
        m_lasHeader.SetReserved(reserved);
        log()->get(LogLevel::Debug) << "Setting reserved to " << reserved <<
            " from metadata " << std::endl;

        uint16_t filesource_id = getMetadataOption<uint16_t>(getOptions(),
            m_metadata, "filesource_id", 0);
        m_lasHeader.SetFileSourceId(filesource_id);
        log()->get(LogLevel::Debug) << "Setting file source id to " <<
            filesource_id << " from metadata " << std::endl;

        try
        {
            boost::optional<pdal::Options const&> opts =
                getOptions().getOption("metadata").getOptions();
            if (opts)
                setVLRsFromMetadata(m_lasHeader, m_metadata, *opts);
        }
        catch (pdal::option_not_found&)
        {}
    } // useMetadata

    LasHeaderWriter lasHeaderWriter(m_lasHeader, m_streamManager.ostream(),
        m_streamOffset);
    lasHeaderWriter.write();
    m_summaryData.reset();

    if (m_lasHeader.Compressed())
    {
#ifdef PDAL_HAVE_LASZIP
        if (!m_zipPoint)
        {
            PointFormat format = m_lasHeader.getPointFormat();
            boost::scoped_ptr<ZipPoint> z(new ZipPoint(format, m_lasHeader,
                false));
            m_zipPoint.swap(z);
        }

        if (!m_zipper)
        {
            boost::scoped_ptr<LASzipper> z(new LASzipper());
            m_zipper.swap(z);

            bool stat(false);
            stat = m_zipper->open(m_streamManager.ostream(),
                m_zipPoint->GetZipper());
            if (!stat)
            {
                std::ostringstream oss;
                const char* err = m_zipper->get_error();
                if (err==NULL) err="(unknown error)";
                oss << "Error opening LASzipper: " << std::string(err);
                throw pdal_error(oss.str());
            }
        }
#else
        throw pdal_error("LASzip compression is not enabled for "
            "this compressed file!");
#endif
    }
    m_headerInitialized = true;
}

void Writer::write(const PointBuffer& pointBuffer)
{
    boost::uint32_t numValidPoints = 0;

    boost::uint8_t buf[64]; // BUG: fixed size

    bool hasColor = m_lasHeader.hasColor();
    bool hasTime = m_lasHeader.hasTime();
    boost::uint16_t record_length = m_lasHeader.GetDataRecordLength();

    m_callback->setTotal(pointBuffer.size());
    m_callback->invoke(0);

    for (PointId idx = 0; idx < pointBuffer.size(); idx++)
    {
        boost::uint8_t* p = buf;

        // we always write the base fields
        using namespace Dimension;

        uint8_t returnNumber(0);
        uint8_t numberOfReturns(0);
        if (pointBuffer.hasDim(Id::ReturnNumber))
            returnNumber = pointBuffer.getFieldAs<uint8_t>(Id::ReturnNumber,
                idx);
        if (pointBuffer.hasDim(Id::NumberOfReturns))
            numberOfReturns = pointBuffer.getFieldAs<uint8_t>(
                Id::NumberOfReturns, idx);
        if (m_discardHighReturnNumbers && numberOfReturns > MAX_RETURN_NUMBER)
        {
            if (returnNumber > MAX_RETURN_NUMBER)
            {
                if (idx % 100 == 0)
                    m_callback->invoke(idx + 1);
                continue;
            }
            numberOfReturns = MAX_RETURN_NUMBER;
        }

        double x = pointBuffer.getFieldAs<double>(Id::X, idx);
        double y = pointBuffer.getFieldAs<double>(Id::Y, idx);
        double z = pointBuffer.getFieldAs<double>(Id::Z, idx);

        x = (x - m_xXform.m_offset) / m_xXform.m_scale;
        y = (y - m_yXform.m_offset) / m_yXform.m_scale;
        z = (z - m_zXform.m_offset) / m_zXform.m_scale;

        Utils::write_field(p, boost::numeric_cast<int32_t>(lround(x)));
        Utils::write_field(p, boost::numeric_cast<int32_t>(lround(y)));
        Utils::write_field(p, boost::numeric_cast<int32_t>(lround(z)));

        uint16_t intensity = 0;
        if (pointBuffer.hasDim(Id::Intensity))
            intensity = pointBuffer.getFieldAs<uint16_t>(Id::Intensity, idx);
        Utils::write_field(p, intensity);

        uint8_t scanDirectionFlag(0);
        if (pointBuffer.hasDim(Id::ScanDirectionFlag))
            scanDirectionFlag = pointBuffer.getFieldAs<uint8_t>(
                Id::ScanDirectionFlag, idx);

        uint8_t edgeOfFlightLine(0);
        if (pointBuffer.hasDim(Id::EdgeOfFlightLine))
            edgeOfFlightLine = pointBuffer.getFieldAs<uint8_t>(
                Id::EdgeOfFlightLine, idx);

        boost::uint8_t bits = returnNumber | (numberOfReturns<<3) |
            (scanDirectionFlag << 6) | (edgeOfFlightLine << 7);
        Utils::write_field(p, bits);

        uint8_t classification = 0;
        if (pointBuffer.hasDim(Id::Classification))
            classification = pointBuffer.getFieldAs<uint8_t>(Id::Classification,
                idx);
        Utils::write_field(p, classification);

        int8_t scanAngleRank = 0;
        if (pointBuffer.hasDim(Id::ScanAngleRank))
            scanAngleRank = pointBuffer.getFieldAs<int8_t>(Id::ScanAngleRank,
                idx);
        Utils::write_field(p, scanAngleRank);

        uint8_t userData = 0;
        if (pointBuffer.hasDim(Id::UserData))
            userData = pointBuffer.getFieldAs<uint8_t>(Id::UserData, idx);
        Utils::write_field(p, userData);

        uint16_t pointSourceId = 0;
        if (pointBuffer.hasDim(Id::PointSourceId))
            pointSourceId = pointBuffer.getFieldAs<uint16_t>(Id::PointSourceId,
                idx);
        Utils::write_field(p, pointSourceId);

        if (hasTime)
        {
            double t = 0.0;
            if (pointBuffer.hasDim(Id::GpsTime))
                t = pointBuffer.getFieldAs<double>(Id::GpsTime, idx);
            Utils::write_field(p, t);
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

            Utils::write_field(p, red);
            Utils::write_field(p, green);
            Utils::write_field(p, blue);
        }

        //Buffer is complete.

        // If we're going to compress, do it.
#ifdef PDAL_HAVE_LASZIP
        if (m_zipPoint)
        {
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
        {
            Utils::write_n(m_streamManager.ostream(), buf, record_length);
        }
#else
        Utils::write_n(m_streamManager.ostream(), buf, record_length);
#endif
        ++numValidPoints;

        double xValue = pointBuffer.getFieldAs<double>(Id::X, idx);
        double yValue = pointBuffer.getFieldAs<double>(Id::Y, idx);
        double zValue = pointBuffer.getFieldAs<double>(Id::Z, idx);
        m_summaryData.addPoint(xValue, yValue, zValue, returnNumber);

        // Perhaps the interval should come out of the callback itself.
        if (idx % 100 == 0)
            m_callback->invoke(idx + 1);
    }
    m_callback->invoke(pointBuffer.size());

    m_numPointsWritten = m_numPointsWritten + numValidPoints;
}

void Writer::done(PointContext ctx)
{
    m_lasHeader.SetPointRecordsCount(m_numPointsWritten);

    log()->get(LogLevel::Debug) << "Wrote " << m_numPointsWritten <<
        " points to the LAS file" << std::endl;

    m_streamManager.ostream().seekp(m_streamOffset);
    Support::rewriteHeader(m_streamManager.ostream(), m_summaryData);
}

} // namespace las
} // namespace drivers
} // namespace pdal

