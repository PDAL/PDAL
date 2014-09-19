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

#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/las/Support.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>

#include <pdal/Charbuf.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/drivers/las/Header.hpp>
#include <pdal/drivers/las/VariableLengthRecord.hpp>
#include <pdal/drivers/las/ZipPoint.hpp>
#include <pdal/IStream.hpp>
#include "LasHeaderReader.hpp"
#include <pdal/PointBuffer.hpp>
#include <pdal/Metadata.hpp>
#include <stdexcept>

#ifdef PDAL_HAVE_GDAL
#include "gdal.h"
#include "cpl_vsi.h"
#include "cpl_conv.h"
#include "cpl_string.h"
#endif

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace pdal
{
namespace drivers
{
namespace las
{


Reader::~Reader()
{
    if (m_istream)
        m_streamFactory->deallocate(*m_istream);
}


void Reader::processOptions(const Options& options)
{
    m_filename = options.getValueOrDefault<std::string>("filename", "");
}


void Reader::initialize()
{
    initialize(m_metadata);
}


void Reader::initialize(MetadataNode& m)
{
    m_streamFactory = createFactory();
    m_istream = &(m_streamFactory->allocate());
    LasHeaderReader lasHeaderReader(m_lasHeader, *m_istream);
    try
    {
        lasHeaderReader.read(*this);
    }
    catch (const std::invalid_argument& e)
    {
        // Improve the error message. #277
        std::stringstream msg;
        if (m_filename.empty())
        {
            throw e;
        }
        msg << "Unable to read file " << m_filename
            << ". It does not have a las file signature.";
        throw std::invalid_argument(msg.str());
    }
    extractMetadata(m);
}


Options Reader::getDefaultOptions()
{
    Option option1("filename", "", "file to read from");
    Options options(option1);
    return options;
}


void Reader::extractMetadata(MetadataNode& m)
{
    // If the user is already overriding this by setting it on the stage, we'll
    // take their overridden value
    const SpatialReference& srs = getSpatialReference();
    if (srs.getWKT(pdal::SpatialReference::eCompoundOK).empty())
    {
        SpatialReference new_srs;
        m_lasHeader.getVLRs().constructSRS(new_srs);
        setSpatialReference(m, new_srs);
    }
    m.add<bool>("compressed", m_lasHeader.Compressed(),
        "true if this LAS file is compressed");
    m.add<uint32_t>("dataformat_id",
        static_cast<uint32_t>(m_lasHeader.getPointFormat()),
        "The Point Format ID as specified in the LAS specification");
    m.add<uint32_t>("major_version",
        static_cast<uint32_t>(m_lasHeader.GetVersionMajor()),
        "The major LAS version for the file, always 1 for now");
    m.add<uint32_t>("minor_version",
        static_cast<uint32_t>(m_lasHeader.GetVersionMinor()),
        "The minor LAS version for the file");
    m.add<uint32_t>("filesource_id",
        static_cast<uint32_t>(m_lasHeader.GetFileSourceId()),
        "File Source ID (Flight Line Number if this file was derived from "
        "an original flight line): This field should be set to a value "
        "between 1 and 65,535, inclusive. A value of zero (0) is interpreted "
        "to mean that an ID has not been assigned. In this case, processing "
        "software is free to assign any valid number. Note that this scheme "
        "allows a LIDAR project to contain up to 65,535 unique sources. A "
        "source can be considered an original flight line or it can be the "
        "result of merge and/or extract operations.");

    //Reserved is always 0, making this seem a bit silly.
    boost::uint16_t reserved = m_lasHeader.GetReserved();
    uint8_t *start = (uint8_t*)&reserved;
    std::vector<uint8_t> raw_bytes;
    for (std::size_t i = 0 ; i < sizeof(uint16_t); ++i)
        raw_bytes.push_back(start[i]);

    m.addEncoded("global_encoding", raw_bytes.data(), raw_bytes.size(),
        "Global Encoding: "
        "This is a bit field used to "
        "indicate certain global properties about the file. In LAS 1.2 "
        "(the version in which this field was introduced), only the low bit "
        "is defined (this is the bit, that if set, would have the unsigned "
        "integer yield a value of 1).");
    m.add<boost::uuids::uuid>("project_id",
         m_lasHeader.GetProjectId(), "Project ID (GUID data): The four fields "
         "that comprise a complete Globally Unique Identifier (GUID) are now "
         "reserved for use as a Project Identifier (Project ID). The field "
         "remains optional. The time of assignment of the Project ID is at "
         "the discretion of processing software. The Project ID should be "
         "the same for all files that are associated with a unique project. "
         "By assigning a Project ID and using a File Source ID (defined above) "         "every file within a project and every point within a file can be "
         "uniquely identified, globally.");
    m.add<std::string>("system_id", m_lasHeader.GetSystemId(false));
    m.add<std::string>("software_id",
        m_lasHeader.GetSoftwareId(false), "This information is ASCII data "
        "describing the generating software itself. This field provides a "
        "mechanism for specifying which generating software package and "
        "version was used during LAS file creation (e.g. \"TerraScan V-10.8\","
        " \"REALM V-4.2\" and etc.).");
    m.add<uint32_t>("creation_doy",
        static_cast<uint32_t>(m_lasHeader.GetCreationDOY()),
        "Day, expressed as an unsigned short, on which this file was created. "
        "Day is computed as the Greenwich Mean Time (GMT) day. January 1 is "
        "considered day 1.");
    m.add<uint32_t>("creation_year",
        static_cast<boost::uint32_t>(m_lasHeader.GetCreationYear()),
        "The year, expressed as a four digit number, in which the file was "
        "created.");
    m.add<uint32_t>("header_size",
        static_cast<uint32_t>(m_lasHeader.GetHeaderSize()),
        "The size, in bytes, of the Public Header Block itself. In the event "
        "that the header is extended by a software application through the "
        "addition of data at the end of the header, the Header Size field "
        "must be updated with the new header size. Extension of the Public "
        "Header Block is discouraged; the Variable Length Records should be "
        "used whenever possible to add custom header data. In the event a "
        "generating software package adds data to the Public Header Block, "
        "this data must be placed at the end of the structure and the Header "
        "Size must be updated to reflect the new size.");
    m.add<uint32_t>("dataoffset",
        static_cast<boost::uint32_t>(m_lasHeader.GetDataOffset()),
        "The actual number of bytes from the beginning of the file to the "
        "first field of the first point record data field. This data offset "
        "must be updated if any software adds data from the Public Header "
        "Block or adds/removes data to/from the Variable Length Records.");
    m.add<double>("scale_x", m_lasHeader.GetScaleX(),
        "The scale factor fields contain a double floating point value that "
        "is used to scale the corresponding X, Y, and Z long values within "
        "the point records. The corresponding X, Y, and Z scale factor must "
        "be multiplied by the X, Y, or Z point record value to get the actual "
        "X, Y, or Z coordinate. For example, if the X, Y, and Z coordinates "
        "are intended to have two decimal point values, then each scale factor "
        "will contain the number 0.01.");
    m.add<double>("scale_y", m_lasHeader.GetScaleY(),
        "The scale factor fields contain a double floating point value that "
        "is used to scale the corresponding X, Y, and Z long values within "
        "the point records. The corresponding X, Y, and Z scale factor must "
        "be multiplied by the X, Y, or Z point record value to get the "
        "actual X, Y, or Z coordinate. For example, if the X, Y, and Z "
        "coordinates are intended to have two decimal point values, then each "
        "scale factor will contain the number 0.01.");
    m.add<double>("scale_z", m_lasHeader.GetScaleZ(),
        "The scale factor fields contain a double floating point value that "
        "is used to scale the corresponding X, Y, and Z long values within "
        "the point records. The corresponding X, Y, and Z scale factor must "
        "be multiplied by the X, Y, or Z point record value to get the actual "
        "X, Y, or Z coordinate. For example, if the X, Y, and Z coordinates "
        "are intended to have two decimal point values, then each scale factor "
        "will contain the number 0.01.");
    m.add<double>("offset_x", m_lasHeader.GetOffsetX(),
        "The offset fields should be used to set the overall offset for the "
        "point records. In general these numbers will be zero, but for "
        "certain cases the resolution of the point data may not be large "
        "enough for a given projection system. However, it should always be "
        "assumed that these numbers are used.");
    m.add<double>("offset_y", m_lasHeader.GetOffsetY(),
        "The offset fields should be used to set the overall offset for the "
        "point records. In general these numbers will be zero, but for "
        "certain cases the resolution of the point data may not be large "
        "enough for a given projection system. However, it should always be "
        "assumed that these numbers are used.");
    m.add<double>("offset_z", m_lasHeader.GetOffsetZ(),
        "The offset fields should be used to set the overall offset for the "
        "point records. In general these numbers will be zero, but for certain "
        "cases the resolution of the point data may not be large enough for "
        "a given projection system. However, it should always be assumed that "
        "these numbers are used.");
    m.add<double>("minx", m_lasHeader.GetMinX(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("miny", m_lasHeader.GetMinY(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("minz", m_lasHeader.GetMinZ(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxx", m_lasHeader.GetMaxX(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxy", m_lasHeader.GetMaxY(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxz", m_lasHeader.GetMaxZ(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<uint32_t>("count",
        m_lasHeader.GetPointRecordsCount(), "This field contains the total "
        "number of point records within the file.");

    std::vector<VariableLengthRecord> const& vlrs =
        m_lasHeader.getVLRs().getAll();
    for (std::vector<VariableLengthRecord>::size_type t = 0;
        t < vlrs.size(); ++t)
    {
        VariableLengthRecord const& v = vlrs[t];

        std::vector<boost::uint8_t> raw_bytes;
        for (std::size_t i = 0 ; i < v.getLength(); ++i)
            raw_bytes.push_back(v.getBytes()[i]);

        std::ostringstream name;
        name << "vlr_" << t;
        MetadataNode vlrNode = m.addEncoded(name.str(), raw_bytes.data(),
            raw_bytes.size(), v.getDescription());

        vlrNode.add<uint32_t>("reserved", v.getReserved(),
            "Two bytes of padded, unused space. Some softwares expect the "
            "values of these bytes to be 0xAABB as specified in the 1.0 "
            "version of the LAS specification");
        vlrNode.add<std::string>("user_id", v.getUserId(),
            "The User ID field is ASCII character data that identifies the "
            "user which created the variable length record. It is possible to "
            "have many Variable Length Records from different sources with "
            "different User IDs. If the character data is less than 16 "
            "characters, the remaining data must be null. The User ID must be "
            "registered with the LAS specification managing body. The "
            "management of these User IDs ensures that no two individuals "
            "accidentally use the same User ID. The specification will "
            "initially use two IDs: one for globally specified records "
            "(LASF_Spec), and another for projection types (LASF_Projection). "
            "Keys may be requested at "
            "http://www.asprs.org/lasform/keyform.html.");
        vlrNode.add<uint32_t>("record_id", v.getRecordId(),
            "The Record ID is dependent upon the User ID. There can be "
            "0 to 65535 Record IDs for every User ID. The LAS specification "
            "manages its own Record IDs (User IDs owned by the specification), "
            "otherwise Record IDs will be managed by the owner of the given "
            "User ID. Thus each User ID is allowed to assign 0 to 65535 Record "
            "IDs in any manner they desire. Publicizing the meaning of a given "
            "Record ID is left to the owner of the given User ID. Unknown User "
            "ID/Record ID combinations should be ignored.");
        vlrNode.add<std::string>("description", v.getDescription());
    }
}


void Reader::addDimensions(PointContextRef ctx)
{
    using namespace Dimension;
    Id::Enum ids[] = { Id::X, Id::Y, Id::Z, Id::Intensity, Id::ReturnNumber,
        Id::NumberOfReturns, Id::ScanDirectionFlag, Id::EdgeOfFlightLine,
        Id::Classification, Id::ScanAngleRank, Id::UserData, Id::PointSourceId,
        Id::Unknown };
    ctx.registerDims(ids);

    if (m_lasHeader.hasTime())
        ctx.registerDim(Id::GpsTime);
    if (m_lasHeader.hasColor())
    {
        Id::Enum ids[] = { Id::Red, Id::Green, Id::Blue, Id::Unknown };
        ctx.registerDims(ids);
    }
}


void Reader::ready(PointContextRef ctx)
{
    m_index = 0;
    m_istream->seekg(m_lasHeader.GetDataOffset());

    if (m_lasHeader.Compressed())
    {
#ifdef PDAL_HAVE_LASZIP
        if (!m_zipPoint)
        {
            PointFormat format = m_lasHeader.getPointFormat();
            std::unique_ptr<ZipPoint> z(new ZipPoint(format,
                m_lasHeader, true));
            m_zipPoint.swap(z);
        }

        if (!m_unzipper)
        {
            std::unique_ptr<LASunzipper> z(new LASunzipper());
            m_unzipper.swap(z);

            m_istream->seekg(static_cast<std::streampos>(
                        m_lasHeader.GetDataOffset()), std::ios::beg);
            if (!m_unzipper->open(*m_istream, m_zipPoint->GetZipper()))
            {
                std::ostringstream oss;
                const char* err = m_unzipper->get_error();
                if (err == NULL)
                    err = "(unknown error)";
                oss << "Failed to open LASzip stream: " << std::string(err);
                throw pdal_error(oss.str());
            }
        }
#else
        throw pdal_error("LASzip is not enabled.  Can't read LAZ data.");
#endif
    }
}


point_count_t Reader::read(PointBuffer& data, point_count_t count)
{
    size_t pointByteCount = m_lasHeader.getPointDataSize();
    count = std::min(count, getNumPoints() - m_index);

    PointId i = 0;
    if (m_zipPoint)
    {
#ifdef PDAL_HAVE_LASZIP
        for (i = 0; i < count; i++)
        {
            if (!m_unzipper->read(m_zipPoint->m_lz_point))
            {
                std::string error = "Error reading compressed point data: ";
                const char* err = m_unzipper->get_error();
                if (!err)
                    err = "(unknown error)";
                error += err;
                throw pdal_error(error);
            }
            loadPoint(data, (char *)m_zipPoint->m_lz_point_data.get(),
                pointByteCount);
        }
#else
        boost::ignore_unused_variable_warning(m_unzipper);
        throw pdal_error("LASzip is not enabled for this "
            "pdal::drivers::las::Reader::processBuffer");
#endif
    }
    else
    {
        std::vector<char> buf(pointByteCount);
        try
        {
            for (; i < count; ++i)
            {
                m_istream->read(buf.data(), pointByteCount);
                loadPoint(data, buf.data(), pointByteCount);
            }
        }
        catch (std::out_of_range&)
        {}
        catch (pdal::invalid_stream&)
        {}
    }
    m_index += i;
    return (point_count_t)i;
}


void Reader::loadPoint(PointBuffer& data, char *buf, size_t bufsize)
{
    // Turn a raw buffer (array of bytes) into a stream buf.
    Charbuf charstreambuf(buf, bufsize, 0);

    // Make an input stream based on the stream buf.
    std::istream stream(&charstreambuf);

    // Wrap the input stream with byte ordering.
    ILeStream istream(&stream);

    PointId nextId = data.size();

    int32_t xi, yi, zi;
    istream >> xi >> yi >> zi;

    const LasHeader& h = m_lasHeader;
            
    double x = xi * h.GetScaleX() + h.GetOffsetX();
    double y = yi * h.GetScaleY() + h.GetOffsetY();
    double z = zi * h.GetScaleZ() + h.GetOffsetZ();

    uint16_t intensity;
    uint8_t flags;
    uint8_t classification;
    int8_t scanAngleRank;
    uint8_t user;
    uint16_t pointSourceId;

    istream >> intensity >> flags >> classification >> scanAngleRank >> 
        user >> pointSourceId;

    uint8_t returnNum = flags & 0x07;
    uint8_t numReturns = (flags >> 3) & 0x07;
    uint8_t scanDirFlag = (flags >> 6) & 0x01;
    uint8_t flight = (flags >> 7) & 0x01;
            
    data.setField(Dimension::Id::X, nextId, x);
    data.setField(Dimension::Id::Y, nextId, y);
    data.setField(Dimension::Id::Z, nextId, z);
    data.setField(Dimension::Id::Intensity, nextId, intensity);
    data.setField(Dimension::Id::ReturnNumber, nextId, returnNum);
    data.setField(Dimension::Id::NumberOfReturns, nextId, numReturns);
    data.setField(Dimension::Id::ScanDirectionFlag, nextId, scanDirFlag);
    data.setField(Dimension::Id::EdgeOfFlightLine, nextId, flight);
    data.setField(Dimension::Id::Classification, nextId, classification);
    data.setField(Dimension::Id::ScanAngleRank, nextId, scanAngleRank);
    data.setField(Dimension::Id::UserData, nextId, user);
    data.setField(Dimension::Id::PointSourceId, nextId, pointSourceId);

    if (h.hasTime())
    {
        double time;
        istream >> time;
        data.setField(Dimension::Id::GpsTime, nextId, time);
    }

    if (h.hasColor())
    {
        uint16_t red, green, blue;
        istream >> red >> green >> blue;
        data.setField(Dimension::Id::Red, nextId, red);
        data.setField(Dimension::Id::Green, nextId, green);
        data.setField(Dimension::Id::Blue, nextId, blue);
    }
}


void Reader::done(PointContextRef ctx)
{
#ifdef PDAL_HAVE_LASZIP
    m_zipPoint.reset();
    m_unzipper.reset();
#endif
}

} // namespace las
} // namespace drivers
} // namespace pdal

