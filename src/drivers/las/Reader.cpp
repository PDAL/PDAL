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

#ifdef PDAL_HAVE_LASZIP
#include <laszip/lasunzipper.hpp>
#endif

#include <pdal/Charbuf.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/drivers/las/Header.hpp>
#include <pdal/drivers/las/VariableLengthRecord.hpp>
#include <pdal/IStream.hpp>
#include "LasHeaderReader.hpp"
#include <pdal/PointBuffer.hpp>
#include <pdal/Metadata.hpp>
#include "ZipPoint.hpp"
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


Reader::Reader(const Options& options)
    : pdal::Reader(options)
{}


Reader::Reader(const std::string& filename)
    : pdal::Reader(Option("filename", filename))
{}


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

    std::istream& stream = m_streamFactory->allocate();

    LasHeaderReader lasHeaderReader(m_lasHeader, stream);
    try
    {
        lasHeaderReader.read(*this, m_schema);
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

    setBounds(m_lasHeader.getBounds());
    setNumPoints(m_lasHeader.GetPointRecordsCount());
    extractMetadata(m);
    m_streamFactory->deallocate(stream);
}


Options Reader::getDefaultOptions()
{
    Option option1("filename", "", "file to read from");
    Options options(option1);
    return options;
}


StreamFactory& Reader::getStreamFactory() const
{
    return *m_streamFactory;
}


pdal::StageSequentialIterator*
Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::las::iterators::sequential::Reader(*this,
        getNumPoints());
}

pdal::StageSequentialIterator* Reader::createSequentialIterator() const
{
    return new pdal::drivers::las::iterators::sequential::Reader(*this,
        getNumPoints());
}


pdal::StageRandomIterator*
Reader::createRandomIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::las::iterators::random::Reader(*this, buffer,
        getNumPoints());
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
    LasHeader const& header = getLasHeader();

    m.add<bool>("compressed", header.Compressed(),
        "true if this LAS file is compressed");
    m.add<uint32_t>("dataformat_id",
        static_cast<uint32_t>(header.getPointFormat()),
        "The Point Format ID as specified in the LAS specification");
    m.add<uint32_t>("major_version",
        static_cast<uint32_t>(header.GetVersionMajor()),
        "The major LAS version for the file, always 1 for now");
    m.add<uint32_t>("minor_version",
        static_cast<uint32_t>(header.GetVersionMinor()),
        "The minor LAS version for the file");
    m.add<uint32_t>("filesource_id",
        static_cast<uint32_t>(header.GetFileSourceId()),
        "File Source ID (Flight Line Number if this file was derived from "
        "an original flight line): This field should be set to a value "
        "between 1 and 65,535, inclusive. A value of zero (0) is interpreted "
        "to mean that an ID has not been assigned. In this case, processing "
        "software is free to assign any valid number. Note that this scheme "
        "allows a LIDAR project to contain up to 65,535 unique sources. A "
        "source can be considered an original flight line or it can be the "
        "result of merge and/or extract operations.");

    boost::uint16_t reserved = header.GetReserved();
    //ABELL  Byte order assumptions?
    boost::uint8_t* start = (uint8_t*)&reserved;
    std::vector<uint8_t> raw_bytes;
    for (std::size_t i = 0 ; i < sizeof(uint16_t); ++i)
        raw_bytes.push_back(start[i]);
    pdal::ByteArray bytearray(raw_bytes);

    m.add("global_encoding", bytearray, "Global Encoding: "
        "This is a bit field used to "
        "indicate certain global properties about the file. In LAS 1.2 "
        "(the version in which this field was introduced), only the low bit "
        "is defined (this is the bit, that if set, would have the unsigned "
        "integer yield a value of 1).");
    m.add<boost::uuids::uuid>("project_id",
         header.GetProjectId(), "Project ID (GUID data): The four fields "
         "that comprise a complete Globally Unique Identifier (GUID) are now "
         "reserved for use as a Project Identifier (Project ID). The field "
         "remains optional. The time of assignment of the Project ID is at "
         "the discretion of processing software. The Project ID should be "
         "the same for all files that are associated with a unique project. "
         "By assigning a Project ID and using a File Source ID (defined above) "         "every file within a project and every point within a file can be "
         "uniquely identified, globally.");
    m.add<std::string>("system_id", header.GetSystemId(false));
    m.add<std::string>("software_id",
        header.GetSoftwareId(false), "This information is ASCII data "
        "describing the generating software itself. This field provides a "
        "mechanism for specifying which generating software package and "
        "version was used during LAS file creation (e.g. \"TerraScan V-10.8\","
        " \"REALM V-4.2\" and etc.).");
    m.add<uint32_t>("creation_doy",
        static_cast<uint32_t>(header.GetCreationDOY()),
        "Day, expressed as an unsigned short, on which this file was created. "
        "Day is computed as the Greenwich Mean Time (GMT) day. January 1 is "
        "considered day 1.");
    m.add<uint32_t>("creation_year",
        static_cast<boost::uint32_t>(header.GetCreationYear()),
        "The year, expressed as a four digit number, in which the file was "
        "created.");
    m.add<uint32_t>("header_size",
        static_cast<uint32_t>(header.GetHeaderSize()),
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
        static_cast<boost::uint32_t>(header.GetDataOffset()),
        "The actual number of bytes from the beginning of the file to the "
        "first field of the first point record data field. This data offset "
        "must be updated if any software adds data from the Public Header "
        "Block or adds/removes data to/from the Variable Length Records.");
    m.add<double>("scale_x", header.GetScaleX(),
        "The scale factor fields contain a double floating point value that "
        "is used to scale the corresponding X, Y, and Z long values within "
        "the point records. The corresponding X, Y, and Z scale factor must "
        "be multiplied by the X, Y, or Z point record value to get the actual "
        "X, Y, or Z coordinate. For example, if the X, Y, and Z coordinates "
        "are intended to have two decimal point values, then each scale factor "
        "will contain the number 0.01.");
    m.add<double>("scale_y", header.GetScaleY(),
        "The scale factor fields contain a double floating point value that "
        "is used to scale the corresponding X, Y, and Z long values within "
        "the point records. The corresponding X, Y, and Z scale factor must "
        "be multiplied by the X, Y, or Z point record value to get the "
        "actual X, Y, or Z coordinate. For example, if the X, Y, and Z "
        "coordinates are intended to have two decimal point values, then each "
        "scale factor will contain the number 0.01.");
    m.add<double>("scale_z", header.GetScaleZ(),
        "The scale factor fields contain a double floating point value that "
        "is used to scale the corresponding X, Y, and Z long values within "
        "the point records. The corresponding X, Y, and Z scale factor must "
        "be multiplied by the X, Y, or Z point record value to get the actual "
        "X, Y, or Z coordinate. For example, if the X, Y, and Z coordinates "
        "are intended to have two decimal point values, then each scale factor "
        "will contain the number 0.01.");
    m.add<double>("offset_x", header.GetOffsetX(),
        "The offset fields should be used to set the overall offset for the "
        "point records. In general these numbers will be zero, but for "
        "certain cases the resolution of the point data may not be large "
        "enough for a given projection system. However, it should always be "
        "assumed that these numbers are used.");
    m.add<double>("offset_y", header.GetOffsetY(),
        "The offset fields should be used to set the overall offset for the "
        "point records. In general these numbers will be zero, but for "
        "certain cases the resolution of the point data may not be large "
        "enough for a given projection system. However, it should always be "
        "assumed that these numbers are used.");
    m.add<double>("offset_z", header.GetOffsetZ(),
        "The offset fields should be used to set the overall offset for the "
        "point records. In general these numbers will be zero, but for certain "
        "cases the resolution of the point data may not be large enough for "
        "a given projection system. However, it should always be assumed that "
        "these numbers are used.");
    m.add<double>("minx", header.GetMinX(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("miny", header.GetMinY(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("minz", header.GetMinZ(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxx", header.GetMaxX(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxy", header.GetMaxY(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxz", header.GetMaxZ(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<uint32_t>("count",
        header.GetPointRecordsCount(), "This field contains the total number "
        "of point records within the file.");

    std::vector<VariableLengthRecord> const& vlrs = header.getVLRs().getAll();
    for (std::vector<VariableLengthRecord>::size_type t = 0;
        t < vlrs.size(); ++t)
    {
        VariableLengthRecord const& v = vlrs[t];

        std::vector<boost::uint8_t> raw_bytes;
        for (std::size_t i = 0 ; i < v.getLength(); ++i)
            raw_bytes.push_back(v.getBytes()[i]);
        pdal::ByteArray bytearray(raw_bytes);

        std::ostringstream name;
        name << "vlr_" << t;
        MetadataNode vlrNode = m.add(name.str(), bytearray,
            v.getDescription());

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


void Reader::buildSchema(Schema *s)
{
    const LasHeader& h = getLasHeader();

    std::vector<Dimension> output;
    Dimension x("X", dimension::SignedInteger, 4, "X coordinate as a long "
        "integer. You must use the scale and offset information of the "
        "header to determine the double value.");
    x.setUUID("2ee118d1-119e-4906-99c3-42934203f872");
    x.setNamespace(getName());
    x.setNumericOffset(h.GetOffsetX());
    x.setNumericScale(h.GetScaleX());
    s->appendDimension(x);

    Dimension y("Y", dimension::SignedInteger, 4, "Y coordinate as a long "
        "integer. You must use the scale and offset information of the "
        "header to determine the double value.");
    y.setUUID("87707eee-2f30-4979-9987-8ef747e30275");
    y.setNamespace(getName());
    y.setNumericOffset(h.GetOffsetY());
    y.setNumericScale(h.GetScaleY());
    s->appendDimension(y);

    Dimension z("Z", dimension::SignedInteger, 4, "Z coordinate as a long "
        "integer. You must use the scale and offset information of the "
        "header to determine the double value.");
    z.setUUID("e74b5e41-95e6-4cf2-86ad-e3f5a996da5d");
    z.setNamespace(getName());
    z.setNumericOffset(h.GetOffsetZ());
    z.setNumericScale(h.GetScaleZ());
    s->appendDimension(z);

    Dimension intensity("Intensity", dimension::UnsignedInteger, 2,
        "The intensity value is the integer representation of the pulse "
        "return magnitude. This value is optional and system specific. "
        "However, it should always be included if available.");
    intensity.setUUID("61e90c9a-42fc-46c7-acd3-20d67bd5626f");
    intensity.setNamespace(getName());
    s->appendDimension(intensity);

    Dimension return_number("ReturnNumber", dimension::UnsignedInteger, 1,
        "Return Number: The Return Number is the pulse return number for "
        "a given output pulse. A given output laser pulse can have many "
        "returns, and they must be marked in sequence of return. The first "
        "return will have a Return Number of one, the second a Return "
        "Number of two, and so on up to five returns.");
    return_number.setUUID("ffe5e5f8-4cec-4560-abf0-448008f7b89e");
    return_number.setNamespace(getName());
    s->appendDimension(return_number);

    Dimension number_of_returns("NumberOfReturns", dimension::UnsignedInteger,
        1, "Number of Returns (for this emitted pulse): The Number of Returns "
        "is the total number of returns for a given pulse. For example, "
        "a laser data point may be return two (Return Number) within a "
        "total number of five returns.");
    number_of_returns.setUUID("7c28bfd4-a9ed-4fb2-b07f-931c076fbaf0");
    number_of_returns.setNamespace(getName());
    s->appendDimension(number_of_returns);

    Dimension scan_direction("ScanDirectionFlag", dimension::UnsignedInteger, 1,
        "The Scan Direction Flag denotes the direction at which the "
        "scanner mirror was traveling at the time of the output pulse. "
        "A bit value of 1 is a positive scan direction, and a bit value "
        "of 0 is a negative scan direction (where positive scan direction "
        "is a scan moving from the left side of the in-track direction to "
        "the right side and negative the opposite).");
    scan_direction.setUUID("13019a2c-cf88-480d-a995-0162055fe5f9");
    scan_direction.setNamespace(getName());
    s->appendDimension(scan_direction);

    Dimension edge("EdgeOfFlightLine", dimension::UnsignedInteger, 1,
        "The Edge of Flight Line data bit has a value of 1 only when "
        "the point is at the end of a scan. It is the last point on "
        "a given scan line before it changes direction.");
    edge.setUUID("108c18f2-5cc0-4669-ae9a-f41eb4006ea5");
    edge.setNamespace(getName());
    s->appendDimension(edge);

    Dimension classification("Classification", dimension::UnsignedInteger, 1,
        "Classification in LAS 1.0 was essentially user defined and optional. "
        "LAS 1.1 defines a standard set of ASPRS classifications. In addition, "
        "the field is now mandatory. If a point has never been classified, "
        "this byte must be set to zero. There are no user defined classes "
        "since both point format 0 and point format 1 supply 8 bits per point "
        "for user defined operations. Note that the format for classification "
        "is a bit encoded field with the lower five bits used for class and "
        "the three high bits used for flags.");
    classification.setUUID("b4c67de9-cef1-432c-8909-7c751b2a4e0b");
    classification.setNamespace(getName());
    s->appendDimension(classification);

    Dimension scan_angle("ScanAngleRank", dimension::SignedInteger, 1,
        "The Scan Angle Rank is a signed one-byte number with a "
        "valid range from -90 to +90. The Scan Angle Rank is the "
        "angle (rounded to the nearest integer in the absolute "
        "value sense) at which the laser point was output from the "
        "laser system including the roll of the aircraft. The scan "
        "angle is within 1 degree of accuracy from +90 to 90 degrees. "
        "The scan angle is an angle based on 0 degrees being nadir, "
        "and 90 degrees to the left side of the aircraft in the "
        "direction of flight.");
    scan_angle.setUUID("aaadaf77-e0c9-4df0-81a7-27060794cd69");
    scan_angle.setNamespace(getName());
    s->appendDimension(scan_angle);

    Dimension user_data("UserData", dimension::UnsignedInteger, 1,
        "This field may be used at the users discretion");
    user_data.setUUID("70eb558e-63d4-4804-b1db-fc2fd716927c");
    user_data.setNamespace(getName());
    s->appendDimension(user_data);

    Dimension point_source("PointSourceId", dimension::UnsignedInteger, 2,
        "This value indicates the file from which this point originated. "
        "Valid values for this field are 1 to 65,535 inclusive with zero "
        "being used for a special case discussed below. The numerical value "
        "corresponds to the File Source ID from which this point originated. "
        "Zero is reserved as a convenience to system implementers. A Point "
        "Source ID of zero implies that this point originated in this file. "
        "This implies that processing software should set the Point Source "
        "ID equal to the File Source ID of the file containing this point "
        "at some time during processing. ");
    point_source.setUUID("4e42e96a-6af0-4fdd-81cb-6216ff47bf6b");
    point_source.setNamespace(getName());
    s->appendDimension(point_source);

    if (h.hasTime())
    {
        Dimension time("Time", dimension::Float, 8, "The GPS Time is the "
            "double floating point time tag value at which the point was "
            "acquired. It is GPS Week Time if the Global Encoding low bit "
            "is clear and Adjusted Standard GPS Time if the Global Encoding "
            "low bit is set (see Global Encoding in the Public Header Block "
            "description).");
        time.setUUID("aec43586-2711-4e59-9df0-65aca78a4ffc");
        time.setNamespace(getName());
        s->appendDimension(time);
    }

    if (h.hasColor())
    {
        Dimension red("Red", dimension::UnsignedInteger, 2,
                "The red image channel value associated with this point");
        red.setUUID("a42ce297-6aa2-4a62-bd29-2db19ba862d5");
        red.setNamespace(getName());
        s->appendDimension(red);

        Dimension green("Green", dimension::UnsignedInteger, 2,
                "The green image channel value associated with this point");
        green.setUUID("7752759d-5713-48cd-9842-51db350cc979");
        green.setNamespace(getName());
        s->appendDimension(green);

        Dimension blue("Blue", dimension::UnsignedInteger, 2,
                "The blue image channel value associated with this point");
        blue.setUUID("5c1a99c8-1829-4d5b-8735-4f6f393a7970");
        blue.setNamespace(getName());
        s->appendDimension(blue);
    }

//ABELL - These don't seem to be used.
/**
    Dimension packet_descriptor("WavePacketDescriptorIndex",
        dimension::UnsignedInteger, 1);
    packet_descriptor.setUUID("1d095eb0-099f-4800-abb6-2272be486f81");
    packet_descriptor.setNamespace(getName());
    output.push_back(packet_descriptor);

    Dimension packet_offset("WaveformDataOffset",
        dimension::UnsignedInteger, 8);
    packet_offset.setUUID("6dee8edf-0c2a-4554-b999-20c9d5f0e7b9");
    packet_offset.setNamespace(getName());
    output.push_back(packet_offset);

    Dimension return_point("ReturnPointWaveformLocation",
        dimension::UnsignedInteger, 4);
    return_point.setUUID("f0f37962-2563-4c3e-858d-28ec15a1103f");
    return_point.setNamespace(getName());
    output.push_back(return_point);

    Dimension wave_x("WaveformXt", dimension::Float, 4);
    wave_x.setUUID("c0ec76eb-9121-4127-b3d7-af92ef871a2d");
    wave_x.setNamespace(getName());
    output.push_back(wave_x);

    Dimension wave_y("WaveformYt", dimension::Float, 4);
    wave_y.setUUID("b3f5bb56-3c25-42eb-9476-186bb6b78e3c");
    wave_y.setNamespace(getName());
    output.push_back(wave_y);

    Dimension wave_z("WaveformZt", dimension::Float, 4);
    wave_z.setUUID("7499ae66-462f-4d0b-a449-6e5c721fb087");
    wave_z.setNamespace(getName());
    output.push_back(wave_z);
**/
}

namespace iterators
{

Base::Base(pdal::drivers::las::Reader const& reader)
    : m_bounds(3), m_reader(reader),
    m_istream(m_reader.getStreamFactory().allocate()) , m_zipPoint(NULL),
    m_unzipper(NULL)
{
    m_istream.seekg(m_reader.getLasHeader().GetDataOffset());

    if (m_reader.getLasHeader().Compressed())
    {
#ifdef PDAL_HAVE_LASZIP
        initialize();
#else
        throw pdal_error("LASzip is not enabled for this "
            "pdal::drivers::las::IteratorBase!");
#endif
    }
}


Base::~Base()
{
#ifdef PDAL_HAVE_LASZIP
    m_zipPoint.reset();
    m_unzipper.reset();
#endif
    m_reader.getStreamFactory().deallocate(m_istream);
}


void Base::initialize()
{
#ifdef PDAL_HAVE_LASZIP
    if (!m_zipPoint)
    {
        PointFormat format = m_reader.getLasHeader().getPointFormat();
        boost::scoped_ptr<ZipPoint> z(new ZipPoint(format,
            m_reader.getLasHeader(), true));
        m_zipPoint.swap(z);
    }

    if (!m_unzipper)
    {
        boost::scoped_ptr<LASunzipper> z(new LASunzipper());
        m_unzipper.swap(z);

        m_istream.seekg(static_cast<std::streampos>(
            m_reader.getLasHeader().GetDataOffset()), std::ios::beg);
        if (!m_unzipper->open(m_istream, m_zipPoint->GetZipper()))
        {
            std::ostringstream oss;
            const char* err = m_unzipper->get_error();
            if (err==NULL)
                err="(unknown error)";
            oss << "Failed to open LASzip stream: " << std::string(err);
            throw pdal_error(oss.str());
        }
    }
#endif
}

point_count_t Base::processBuffer(PointBuffer& data, std::istream& stream,
    point_count_t count, LASunzipper* unzipper, ZipPoint* zipPoint,
    PointDimensions* dimensions)
{
    if (!dimensions)
        throw pdal_error("No dimension positions are available!");
 
    const LasHeader& h = m_reader.getLasHeader();
    size_t pointByteCount = h.getPointDataSize();

    PointId i = 0;
    if (zipPoint)
    {
#ifdef PDAL_HAVE_LASZIP
        for (i = 0; i < count; i++)
        {
            if (!unzipper->read(zipPoint->m_lz_point))
            {
                std::string error = "Error reading compressed point data: ";
                const char* err = unzipper->get_error();
                if (!err)
                    err = "(unknown error)";
                error += err;
                throw pdal_error(error);
            }
            loadPoint(data, dimensions, (char *)zipPoint->m_lz_point_data.get(),
                pointByteCount);
        }
#else
        boost::ignore_unused_variable_warning(unzipper);
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
                stream.read(buf.data(), pointByteCount);
                loadPoint(data, dimensions, buf.data(), pointByteCount);
            }
        }
        catch (std::out_of_range&)
        {}
        catch (pdal::invalid_stream&)
        {}
    }
//ABELL
//    data.setSpatialBounds(m_bounds);
    return (point_count_t)i;
}


void Base::loadPoint(PointBuffer& data, PointDimensions *dimensions,
    char *buf, size_t bufsize)
{
    Charbuf charstreambuf(buf, bufsize, 0);
    std::istream stream(&charstreambuf);
    ILeStream istream(&stream);
    PointId nextId = data.size();

    int32_t x, y, z;
    istream >> x >> y >> z;
            
    if (dimensions->X && dimensions->Y && dimensions->Z)
        m_bounds.grow(dimensions->X->applyScaling(x),
                      dimensions->X->applyScaling(y),
                      dimensions->X->applyScaling(z));

    uint16_t intensity;
    uint8_t flags;
    uint8_t classification;
    int8_t scanAngleRank;
    uint8_t user;
    uint16_t pointSourceId;

    istream >> intensity >> flags >> classification >> scanAngleRank >> 
        user >> pointSourceId;
    classification &= 31;

    uint8_t returnNum = flags & 0x07;
    uint8_t numReturns = (flags >> 3) & 0x07;
    uint8_t scanDirFlag = (flags >> 6) & 0x01;
    uint8_t flight = (flags >> 7) & 0x01;
            
    if (dimensions->X)
        data.setField(*dimensions->X, nextId, x);
    if (dimensions->Y)
        data.setField(*dimensions->Y, nextId, y);
    if (dimensions->Z)
        data.setField(*dimensions->Z, nextId, z);
    if (dimensions->Intensity)
        data.setField(*dimensions->Intensity, nextId, intensity);
    if (dimensions->ReturnNumber)
        data.setField(*dimensions->ReturnNumber, nextId, returnNum);
    if (dimensions->NumberOfReturns)
        data.setField(*dimensions->NumberOfReturns, nextId, numReturns);
    if (dimensions->ScanDirectionFlag)
        data.setField(*dimensions->ScanDirectionFlag, nextId, scanDirFlag);
    if (dimensions->EdgeOfFlightLine)
        data.setField(*dimensions->EdgeOfFlightLine, nextId, flight);
    if (dimensions->Classification)
        data.setField(*dimensions->Classification, nextId, classification);
    if (dimensions->ScanAngleRank)
        data.setField(*dimensions->ScanAngleRank, nextId, scanAngleRank);
    if (dimensions->UserData)
        data.setField(*dimensions->UserData, nextId, user);
    if (dimensions->PointSourceId)
        data.setField(*dimensions->PointSourceId, nextId, pointSourceId);

    const LasHeader& h = m_reader.getLasHeader();
    if (h.hasTime())
    {
        double time;
        istream >> time;
        if (dimensions->Time)
            data.setField(*dimensions->Time, nextId, time);
    }

    if (h.hasColor())
    {
        uint16_t red, green, blue;
        istream >> red >> green >> blue;
        if (dimensions->Red)
            data.setField(*dimensions->Red, nextId, red);
        if (dimensions->Green)
            data.setField(*dimensions->Green, nextId, green);
        if (dimensions->Blue)
            data.setField(*dimensions->Blue, nextId, blue);
    }
}


namespace sequential
{

Reader::Reader(pdal::drivers::las::Reader const& reader,
        boost::uint32_t numPoints)
    : Base(reader), m_numPoints(numPoints)
{}


boost::uint64_t Reader::skipImpl(boost::uint64_t count)
{
    const LasHeader& h = m_reader.getLasHeader();

#ifdef PDAL_HAVE_LASZIP
    if (m_unzipper)
    {
        const boost::uint32_t pos32 =
            Utils::safeconvert64to32(getIndex() + count);
        m_unzipper->seek(pos32);
    }
    else
    {
        std::streamoff delta = h.getPointDataSize();
        m_istream.seekg(delta * count, std::ios::cur);
    }
#else
    std::streamoff delta = h.getPointDataSize();
    m_istream.seekg(delta * count, std::ios::cur);
#endif
    return count;
}


bool Reader::atEndImpl() const
{
    return getIndex() >= m_numPoints;
}


point_count_t Reader::readImpl(PointBuffer& data, point_count_t count)
{
    PointDimensions cachedDimensions(data.getSchema(), m_reader.getName());

    boost::uint32_t numToRead = m_numPoints - getIndex();
#ifdef PDAL_HAVE_LASZIP
    return processBuffer(data, m_istream, count, m_unzipper.get(),
        m_zipPoint.get(), &cachedDimensions);
#else
    return processBuffer(data, m_istream, count, NULL, NULL, &cachedDimensions);
#endif
}

boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    PointDimensions cachedDimensions(data.getSchema(), m_reader.getName());

    boost::uint32_t numToRead = m_numPoints - getIndex();
#ifdef PDAL_HAVE_LASZIP
    return processBuffer(data, m_istream, numToRead, m_unzipper.get(),
        m_zipPoint.get(), &cachedDimensions);
#else
    return processBuffer(data, m_istream, numToRead, NULL, NULL,
        &cachedDimensions);
#endif
}

} // sequential

namespace random
{

Reader::Reader(const pdal::drivers::las::Reader& reader, PointBuffer& buffer,
        boost::uint32_t numPoints)
    : Base(reader), pdal::ReaderRandomIterator(buffer), m_numPoints(numPoints)
{}


boost::uint64_t Reader::seekImpl(boost::uint64_t count)
{
    const LasHeader& h = m_reader.getLasHeader();

#ifdef PDAL_HAVE_LASZIP
    if (m_unzipper)
    {
        const boost::uint32_t pos32 = Utils::safeconvert64to32(count);
        m_unzipper->seek(pos32);
    }
    else
    {
        std::streamoff delta = h.getPointDataSize();
        m_istream.seekg(h.GetDataOffset() + delta * count);
    }
#else
    std::streamoff delta = h.getPointDataSize();
    m_istream.seekg(h.GetDataOffset() + delta * count);
#endif
    return count;
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    PointDimensions cachedDimensions(data.getSchema(), m_reader.getName());

    boost::uint32_t numToRead = m_numPoints - getIndex();
#ifdef PDAL_HAVE_LASZIP
    return processBuffer(data, m_istream, numToRead, m_unzipper.get(),
        m_zipPoint.get(), &cachedDimensions);
#else
    return processBuffer(data, m_istream, numToRead, NULL, NULL,
         &cachedDimensions);
#endif
}

} // random
} // iterators

}
}
} // namespaces
