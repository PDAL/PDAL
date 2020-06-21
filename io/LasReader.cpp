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

#include <pdal/compression/LazPerfVlrCompression.hpp>

#include "LasReader.hpp"

#include <sstream>
#include <string.h>

#include <pdal/pdal_features.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/PointView.hpp>
#include <pdal/QuickInfo.hpp>
#include <pdal/util/Extractor.hpp>
#include <pdal/util/IStream.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "GeotiffSupport.hpp"
#include "LasHeader.hpp"
#include "LasVLR.hpp"

namespace pdal
{

namespace
{

struct invalid_stream : public std::runtime_error
{
    invalid_stream(const std::string& msg) : std::runtime_error(msg)
        {}
};

} // unnamed namespace

LasReader::LasReader() : m_decompressor(nullptr), m_index(0)
{}


LasReader::~LasReader()
{
#ifdef PDAL_HAVE_LAZPERF
    delete m_decompressor;
#endif
}


void LasReader::addArgs(ProgramArgs& args)
{
    args.add("extra_dims", "Dimensions to assign to extra byte data",
        m_extraDimSpec);
    args.add("compression", "Decompressor to use", m_compression, "EITHER");
    args.add("use_eb_vlr", "Use extra bytes VLR for 1.0 - 1.3 files",
        m_useEbVlr);
    args.add("ignore_vlr", "VLR userid/recordid to ignore", m_ignoreVLROption);
}


static StaticPluginInfo const s_info {
    "readers.las",
    "ASPRS LAS 1.0 - 1.4 read support. LASzip support is also \n" \
        "enabled through this driver if LASzip was found during \n" \
        "compilation.",
    "http://pdal.io/stages/readers.las.html",
    { "las", "laz" }
};

CREATE_STATIC_STAGE(LasReader, s_info)

std::string LasReader::getName() const { return s_info.name; }

QuickInfo LasReader::inspect()
{
    QuickInfo qi;
    std::unique_ptr<PointLayout> layout(new PointLayout());

    RowPointTable table;
    initialize(table);
    addDimensions(layout.get());

    Dimension::IdList dims = layout->dims();
    for (auto di = dims.begin(); di != dims.end(); ++di)
        qi.m_dimNames.push_back(layout->dimName(*di));
    if (!Utils::numericCast(m_header.pointCount(), qi.m_pointCount))
        qi.m_pointCount = (std::numeric_limits<point_count_t>::max)();
    qi.m_bounds = m_header.getBounds();
    qi.m_srs = getSpatialReference();
    qi.m_valid = true;

    done(table);

    return qi;
}


void LasReader::handleCompressionOption()
{
    std::string compression = Utils::toupper(m_compression);
#if defined(PDAL_HAVE_LAZPERF) && defined(PDAL_HAVE_LASZIP)
    if (compression == "EITHER")
        compression = "LASZIP";
#endif
#if !defined(PDAL_HAVE_LAZPERF) && defined(PDAL_HAVE_LASZIP)
    if (compression == "EITHER")
        compression = "LASZIP";
    if (compression == "LAZPERF")
        throwError("Can't decompress with LAZperf.  PDAL not built "
            "with LAZperf.");
#endif
#if defined(PDAL_HAVE_LAZPERF) && !defined(PDAL_HAVE_LASZIP)
    if (compression == "EITHER")
        compression = "LAZPERF";
    if (compression == "LASZIP")
        throwError("Can't decompress with LASzip.  PDAL not built "
            "with LASzip.");
#endif

#if defined(PDAL_HAVE_LAZPERF) || defined(PDAL_HAVE_LASZIP)
    if (compression != "LAZPERF" && compression != "LASZIP")
        throwError("Invalid value for option for compression: '" +
            m_compression + "'.  Value values are 'lazperf' and 'laszip'.");
#endif

    // Set case-corrected value.
    m_compression = compression;
}


void LasReader::initializeLocal(PointTableRef table, MetadataNode& m)
{
    try
    {
        m_extraDims = LasUtils::parse(m_extraDimSpec, false);
    }
    catch (const LasUtils::error& err)
    {
        throwError(err.what());
    }

    try
    {
        m_ignoreVLRs = LasUtils::parseIgnoreVLRs(m_ignoreVLROption);
    }
    catch (const LasUtils::error& err)
    {
        throwError(err.what());
    }

    m_header.setLog(log());

    createStream();
    std::istream *stream(m_streamIf->m_istream);

    stream->seekg(0);
    ILeStream in(stream);
    try
    {
        // This also reads the extended VLRs at the end of the data.
        in >> m_header;
    }
    catch (const LasHeader::error& e)
    {
        throwError(e.what());
    }

    for (auto i: m_ignoreVLRs)
    {
        if (i.m_recordId)
            m_header.removeVLR(i.m_userId, i.m_recordId);
        else
            m_header.removeVLR(i.m_userId);
    }

    if (m_header.compressed())
        handleCompressionOption();
#ifdef PDAL_HAVE_LASZIP
    m_laszip = nullptr;
#endif

    if (!m_header.pointFormatSupported())
        throwError("Unsupported LAS input point format: " +
            Utils::toString((int)m_header.pointFormat()) + ".");

    if (m_header.versionAtLeast(1, 4) || m_useEbVlr)
        readExtraBytesVlr();
    setSrs(m);
    MetadataNode forward = table.privateMetadata("lasforward");
    extractHeaderMetadata(forward, m);
    extractVlrMetadata(forward, m);

    m_streamIf.reset();
}


void LasReader::handleLaszip(int result)
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


void LasReader::ready(PointTableRef table)
{
    createStream();
    std::istream *stream(m_streamIf->m_istream);

    m_index = 0;
    if (m_header.compressed())
    {
#ifdef PDAL_HAVE_LASZIP
        if (m_compression == "LASZIP")
        {
            laszip_BOOL compressed;

            handleLaszip(laszip_create(&m_laszip));
            handleLaszip(laszip_open_reader_stream(m_laszip, *stream,
                &compressed));
            handleLaszip(laszip_get_point_pointer(m_laszip, &m_laszipPoint));
        }
#endif

#ifdef PDAL_HAVE_LAZPERF
        if (m_compression == "LAZPERF")
        {
            delete m_decompressor;

            const LasVLR *vlr = m_header.findVlr(LASZIP_USER_ID,
                LASZIP_RECORD_ID);
            if (!vlr)
                throwError("LAZ file missing required laszip VLR.");
            m_decompressor = new LazPerfVlrDecompressor(*stream,
                vlr->data(), m_header.pointOffset());
            m_decompressorBuf.resize(m_decompressor->pointSize());
        }
#endif

#if !defined(PDAL_HAVE_LAZPERF) && !defined(PDAL_HAVE_LASZIP)
        throwError("Can't read compressed file without LASzip or "
            "LAZperf decompression library.");
#endif
    }
    else
        stream->seekg(m_header.pointOffset());
}


namespace
{

void addForwardMetadata(MetadataNode& forward, MetadataNode& m,
    const std::string& name, double val, const std::string description,
    size_t precision)
{
    MetadataNode n = m.add(name, val, description, precision);

    // If the entry doesn't already exist, just add it.
    MetadataNode f = forward.findChild(name);
    if (!f.valid())
    {
        forward.add(n);
        return;
    }

    // If the old value and new values aren't the same, set an invalid flag.
    MetadataNode temp = f.addOrUpdate("temp", val, description, precision);
    if (f.value<std::string>() != temp.value<std::string>())
        forward.addOrUpdate(name + "INVALID", "");
}

}

// Store data in the normal metadata place.  Also store it in the private
// lasforward metadata node.
template <typename T>
void addForwardMetadata(MetadataNode& forward, MetadataNode& m,
    const std::string& name, T val, const std::string description)
{
    MetadataNode n = m.add(name, val, description);

    // If the entry doesn't already exist, just add it.
    MetadataNode f = forward.findChild(name);
    if (!f.valid())
    {
        forward.add(n);
        return;
    }

    // If the old value and new values aren't the same, set an invalid flag.
    MetadataNode temp = f.addOrUpdate("temp", val);
    if (f.value<std::string>() != temp.value<std::string>())
        forward.addOrUpdate(name + "INVALID", "");
}


void LasReader::extractHeaderMetadata(MetadataNode& forward, MetadataNode& m)
{
    m.add<bool>("compressed", m_header.compressed(),
        "true if this LAS file is compressed");

    addForwardMetadata(forward, m, "major_version", m_header.versionMajor(),
        "The major LAS version for the file, always 1 for now");
    addForwardMetadata(forward, m, "minor_version", m_header.versionMinor(),
        "The minor LAS version for the file");
    addForwardMetadata(forward, m, "dataformat_id", m_header.pointFormat(),
        "LAS Point Data Format");
    if (m_header.versionAtLeast(1, 1))
        addForwardMetadata(forward, m, "filesource_id",
            m_header.fileSourceId(), "File Source ID (Flight Line Number "
            "if this file was derived from an original flight line).");
    if (m_header.versionAtLeast(1, 2))
    {
        // For some reason we've written global encoding as a base 64
        // encoded value in the past.  In an effort to standardize things,
        // I'm writing this as a special value, and will also write
        // global_encoding like we write all other header metadata.
        uint16_t globalEncoding = m_header.globalEncoding();
        m.addEncoded("global_encoding_base64", (uint8_t *)&globalEncoding,
            sizeof(globalEncoding),
            "Global Encoding: general property bit field.");

        addForwardMetadata(forward, m, "global_encoding",
            m_header.globalEncoding(),
            "Global Encoding: general property bit field.");
    }

    addForwardMetadata(forward, m, "project_id", m_header.projectId(),
        "Project ID.");
    addForwardMetadata(forward, m, "system_id", m_header.systemId(),
        "Generating system ID.");
    addForwardMetadata(forward, m, "software_id", m_header.softwareId(),
        "Generating software description.");
    addForwardMetadata(forward, m, "creation_doy", m_header.creationDOY(),
        "Day, expressed as an unsigned short, on which this file was created. "
        "Day is computed as the Greenwich Mean Time (GMT) day. January 1 is "
        "considered day 1.");
    addForwardMetadata(forward, m, "creation_year", m_header.creationYear(),
        "The year, expressed as a four digit number, in which the file was "
        "created.");
    addForwardMetadata(forward, m, "scale_x", m_header.scaleX(),
        "The scale factor for X values.", 15);
    addForwardMetadata(forward, m, "scale_y", m_header.scaleY(),
        "The scale factor for Y values.", 15);
    addForwardMetadata(forward, m, "scale_z", m_header.scaleZ(),
        "The scale factor for Z values.", 15);
    addForwardMetadata(forward, m, "offset_x", m_header.offsetX(),
        "The offset for X values.", 15);
    addForwardMetadata(forward, m, "offset_y", m_header.offsetY(),
        "The offset for Y values.", 15);
    addForwardMetadata(forward, m, "offset_z", m_header.offsetZ(),
        "The offset for Z values.", 15);

    m.add("point_length", m_header.pointLen(),
        "The size, in bytes, of each point records.");
    m.add("header_size", m_header.vlrOffset(),
        "The size, in bytes, of the header block, including any extension "
        "by specific software.");
    m.add("dataoffset", m_header.pointOffset(),
        "The actual number of bytes from the beginning of the file to the "
        "first field of the first point record data field. This data offset "
        "must be updated if any software adds data from the Public Header "
        "Block or adds/removes data to/from the Variable Length Records.");
    m.add<double>("minx", m_header.minX(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("miny", m_header.minY(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("minz", m_header.minZ(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxx", m_header.maxX(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxy", m_header.maxY(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxz", m_header.maxZ(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<point_count_t>("count",
        m_header.pointCount(), "This field contains the total "
        "number of point records within the file.");

    // PDAL metadata VLR
    const LasVLR *vlr = m_header.findVlr("PDAL", 12);
    if (vlr)
    {
        const char *pos = vlr->data();
        size_t size = vlr->dataLen();
        m.addWithType("pdal_metadata", std::string(pos, size), "json",
            "PDAL Processing Metadata");
    }
    //
    // PDAL pipeline VLR
    vlr = m_header.findVlr("PDAL", 13);
    if (vlr)
    {
        const char *pos = vlr->data();
        size_t size = vlr->dataLen();
        m.addWithType("pdal_pipeline", std::string(pos, size), "json",
            "PDAL Processing Pipeline");
    }
}


void LasReader::readExtraBytesVlr()
{
    const LasVLR *vlr = m_header.findVlr(SPEC_USER_ID,
        EXTRA_BYTES_RECORD_ID);
    if (!vlr)
        return;
    const char *pos = vlr->data();
    size_t size = vlr->dataLen();
    if (size % sizeof(ExtraBytesSpec) != 0)
    {
        log()->get(LogLevel::Warning) << "Bad size for extra bytes VLR.  "
            "Ignoring.";
        return;
    }
    size /= sizeof(ExtraBytesSpec);
    std::vector<ExtraBytesIf> ebList;
    while (size--)
    {
        ExtraBytesIf eb;
        eb.readFrom(pos);
        ebList.push_back(eb);
        pos += sizeof(ExtraBytesSpec);
    }

    std::vector<ExtraDim> extraDims;
    for (ExtraBytesIf& eb : ebList)
    {
       std::vector<ExtraDim> eds = eb.toExtraDims();
       for (auto& ed : eds)
           extraDims.push_back(std::move(ed));
    }
    if (m_extraDims.size() && m_extraDims != extraDims)
        log()->get(LogLevel::Warning) << "Extra byte dimensions specified "
            "in pipeline and VLR don't match.  Ignoring pipeline-specified "
            "dimensions";
    m_extraDims = extraDims;
}


void LasReader::setSrs(MetadataNode& m)
{
    setSpatialReference(m, m_header.srs());
}


void LasReader::extractVlrMetadata(MetadataNode& forward, MetadataNode& m)
{
    static const size_t DATA_LEN_MAX = 1000000;

    int i = 0;
    for (auto vlr : m_header.vlrs())
    {
        if (vlr.dataLen() > DATA_LEN_MAX)
            continue;

        std::ostringstream name;
        name << "vlr_" << i++;
        MetadataNode vlrNode(name.str());

        vlrNode.addEncoded("data",
            (const uint8_t *)vlr.data(), vlr.dataLen(), vlr.description());
        vlrNode.add("user_id", vlr.userId(),
            "User ID of the record or pre-defined value from the "
            "specification.");
        vlrNode.add("record_id", vlr.recordId(),
            "Record ID specified by the user.");
        vlrNode.add("description", vlr.description());
        m.add(vlrNode);

        if (vlr.userId() == TRANSFORM_USER_ID||
            vlr.userId() == LASZIP_USER_ID ||
            vlr.userId() == LIBLAS_USER_ID)
            continue;
        if (vlr.userId() == SPEC_USER_ID &&
            vlr.recordId() != 0 && vlr.recordId() != 3)
            continue;
        forward.add(vlrNode);
    }
}


void LasReader::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;

    layout->registerDim(Id::X, Type::Double);
    layout->registerDim(Id::Y, Type::Double);
    layout->registerDim(Id::Z, Type::Double);
    layout->registerDim(Id::Intensity, Type::Unsigned16);
    layout->registerDim(Id::ReturnNumber, Type::Unsigned8);
    layout->registerDim(Id::NumberOfReturns, Type::Unsigned8);
    layout->registerDim(Id::ScanDirectionFlag, Type::Unsigned8);
    layout->registerDim(Id::EdgeOfFlightLine, Type::Unsigned8);
    layout->registerDim(Id::Classification, Type::Unsigned8);
    layout->registerDim(Id::ScanAngleRank, Type::Float);
    layout->registerDim(Id::UserData, Type::Unsigned8);
    layout->registerDim(Id::PointSourceId, Type::Unsigned16);

    if (m_header.hasTime())
        layout->registerDim(Id::GpsTime, Type::Double);
    if (m_header.hasColor())
    {
        layout->registerDim(Id::Red, Type::Unsigned16);
        layout->registerDim(Id::Green, Type::Unsigned16);
        layout->registerDim(Id::Blue, Type::Unsigned16);
    }
    if (m_header.hasInfrared())
        layout->registerDim(Id::Infrared);
    if (m_header.has14Format())
    {
        layout->registerDim(Id::ScanChannel);
        layout->registerDim(Id::ClassFlags);
    }

    for (auto& dim : m_extraDims)
    {
        Dimension::Type type = dim.m_dimType.m_type;
        if (type == Dimension::Type::None)
            continue;
        if (dim.m_dimType.m_xform.nonstandard())
            type = Dimension::Type::Double;
        dim.m_dimType.m_id = layout->registerOrAssignDim(dim.m_name, type);
    }
}


bool LasReader::processOne(PointRef& point)
{
    if (m_index >= getNumPoints())
        return false;

    size_t pointLen = m_header.pointLen();

    if (m_header.compressed())
    {
#ifdef PDAL_HAVE_LASZIP
        if (m_compression == "LASZIP")
        {
            handleLaszip(laszip_read_point(m_laszip));
            loadPoint(point, *m_laszipPoint);
        }
#endif

#ifdef PDAL_HAVE_LAZPERF
        if (m_compression == "LAZPERF")
        {
            m_decompressor->decompress(m_decompressorBuf.data());
            loadPoint(point, m_decompressorBuf.data(), pointLen);
        }
#endif
#if !defined(PDAL_HAVE_LAZPERF) && !defined(PDAL_HAVE_LASZIP)
        throwError("Can't read compressed file without LASzip or "
            "LAZperf decompression library.");
#endif
    } // compression
    else
    {
        std::vector<char> buf(m_header.pointLen());

        m_streamIf->m_istream->read(buf.data(), pointLen);
        loadPoint(point, buf.data(), pointLen);
    }
    m_index++;
    return true;
}


point_count_t LasReader::read(PointViewPtr view, point_count_t count)
{
    size_t pointLen = m_header.pointLen();
    count = (std::min)(count, getNumPoints() - m_index);

    PointId i = 0;
    if (m_header.compressed())
    {
#if defined(PDAL_HAVE_LAZPERF) || defined(PDAL_HAVE_LASZIP)
        if (m_compression == "LASZIP" || m_compression == "LAZPERF")
        {
            for (i = 0; i < count; i++)
            {
                PointRef point = view->point(i);
                PointId id = view->size();
                processOne(point);
                if (m_cb)
                    m_cb(*view, id);
            }
        }
#else
        throwError("Can't read compressed file without LASzip or "
            "LAZperf decompression library.");
#endif
    }
    else
    {
        point_count_t remaining = count;

        // Make a buffer at most a meg.
        size_t bufsize = (std::min)((point_count_t)1000000, count * pointLen);
        std::vector<char> buf(bufsize);
        try
        {
            do
            {
                point_count_t blockPoints = readFileBlock(buf, remaining);
                remaining -= blockPoints;
                char *pos = buf.data();
                while (blockPoints--)
                {
                    PointId id = view->size();
                    PointRef point = view->point(id);
                    loadPoint(point, pos, pointLen);
                    if (m_cb)
                        m_cb(*view, id);
                    pos += pointLen;
                    i++;
                }
            } while (remaining);
        }
        catch (std::out_of_range&)
        {}
        catch (invalid_stream&)
        {}
    }
    m_index += i;
    return (point_count_t)i;
}


point_count_t LasReader::readFileBlock(std::vector<char>& buf,
    point_count_t maxpoints)
{
    std::istream *stream(m_streamIf->m_istream);

    size_t ptLen = m_header.pointLen();
    point_count_t blockpoints = buf.size() / ptLen;

    blockpoints = (std::min)(maxpoints, blockpoints);
    if (stream->eof())
        throw invalid_stream("stream is done");

    stream->read(buf.data(), blockpoints * ptLen);
    if (stream->gcount() != (std::streamsize)(blockpoints * ptLen))
    {
        // we read fewer bytes than we asked for
        // because the file was either truncated
        // or the header is bunk.
        blockpoints = stream->gcount() / ptLen;
    }
    return blockpoints;
}


#ifdef PDAL_HAVE_LASZIP
void LasReader::loadPoint(PointRef& point, laszip_point& p)
{
    if (m_header.has14Format())
        loadPointV14(point, p);
    else
        loadPointV10(point, p);
}
#endif // PDAL_HAVE_LASZIP


void LasReader::loadPoint(PointRef& point, char *buf, size_t bufsize)
{
    if (m_header.has14Format())
        loadPointV14(point, buf, bufsize);
    else
        loadPointV10(point, buf, bufsize);
}


#ifdef PDAL_HAVE_LASZIP
void LasReader::loadPointV10(PointRef& point, laszip_point& p)
{
    const LasHeader& h = m_header;

    double x = p.X * h.scaleX() + h.offsetX();
    double y = p.Y * h.scaleY() + h.offsetY();
    double z = p.Z * h.scaleZ() + h.offsetZ();

    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
    point.setField(Dimension::Id::Intensity, p.intensity);
    point.setField(Dimension::Id::ReturnNumber, p.return_number);
    point.setField(Dimension::Id::NumberOfReturns, p.number_of_returns);
    point.setField(Dimension::Id::ScanDirectionFlag, p.scan_direction_flag);
    point.setField(Dimension::Id::EdgeOfFlightLine, p.edge_of_flight_line);
    uint8_t classification = p.classification | (p.synthetic_flag << 5) |
        (p.keypoint_flag << 6) | (p.withheld_flag << 7);
    point.setField(Dimension::Id::Classification, classification);
    point.setField(Dimension::Id::ScanAngleRank, p.scan_angle_rank);
    point.setField(Dimension::Id::UserData, p.user_data);
    point.setField(Dimension::Id::PointSourceId, p.point_source_ID);

    if (h.hasTime())
        point.setField(Dimension::Id::GpsTime, p.gps_time);

    if (h.hasColor())
    {
        point.setField(Dimension::Id::Red, p.rgb[0]);
        point.setField(Dimension::Id::Green, p.rgb[1]);
        point.setField(Dimension::Id::Blue, p.rgb[2]);
    }

    if (m_extraDims.size())
    {
        LeExtractor extractor((const char *)p.extra_bytes, p.num_extra_bytes);
        loadExtraDims(extractor, point);
    }
}
#endif // PDAL_HAVE_LASZIP

void LasReader::loadPointV10(PointRef& point, char *buf, size_t bufsize)
{
    LeExtractor istream(buf, bufsize);

    int32_t xi, yi, zi;
    istream >> xi >> yi >> zi;

    const LasHeader& h = m_header;

    double x = xi * h.scaleX() + h.offsetX();
    double y = yi * h.scaleY() + h.offsetY();
    double z = zi * h.scaleZ() + h.offsetZ();

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

    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
    point.setField(Dimension::Id::Intensity, intensity);
    point.setField(Dimension::Id::ReturnNumber, returnNum);
    point.setField(Dimension::Id::NumberOfReturns, numReturns);
    point.setField(Dimension::Id::ScanDirectionFlag, scanDirFlag);
    point.setField(Dimension::Id::EdgeOfFlightLine, flight);
    point.setField(Dimension::Id::Classification, classification);
    point.setField(Dimension::Id::ScanAngleRank, scanAngleRank);
    point.setField(Dimension::Id::UserData, user);
    point.setField(Dimension::Id::PointSourceId, pointSourceId);

    if (h.hasTime())
    {
        double time;
        istream >> time;
        point.setField(Dimension::Id::GpsTime, time);
    }

    if (h.hasColor())
    {
        uint16_t red, green, blue;
        istream >> red >> green >> blue;
        point.setField(Dimension::Id::Red, red);
        point.setField(Dimension::Id::Green, green);
        point.setField(Dimension::Id::Blue, blue);
    }

    if (m_extraDims.size())
        loadExtraDims(istream, point);
}


#ifdef PDAL_HAVE_LASZIP
void LasReader::loadPointV14(PointRef& point, laszip_point& p)
{
    const LasHeader& h = m_header;

    double x = p.X * h.scaleX() + h.offsetX();
    double y = p.Y * h.scaleY() + h.offsetY();
    double z = p.Z * h.scaleZ() + h.offsetZ();

    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
    point.setField(Dimension::Id::Intensity, p.intensity);
    point.setField(Dimension::Id::ReturnNumber, p.extended_return_number);
    point.setField(Dimension::Id::NumberOfReturns,
        p.extended_number_of_returns);
    point.setField(Dimension::Id::ClassFlags, p.extended_classification_flags);
    point.setField(Dimension::Id::ScanChannel, p.extended_scanner_channel);
    point.setField(Dimension::Id::ScanDirectionFlag, p.scan_direction_flag);
    point.setField(Dimension::Id::EdgeOfFlightLine, p.edge_of_flight_line);
    point.setField(Dimension::Id::Classification, p.extended_classification);
    point.setField(Dimension::Id::ScanAngleRank, p.extended_scan_angle * .006);
    point.setField(Dimension::Id::UserData, p.user_data);
    point.setField(Dimension::Id::PointSourceId, p.point_source_ID);
    point.setField(Dimension::Id::GpsTime, p.gps_time);

    if (h.hasColor())
    {
        point.setField(Dimension::Id::Red, p.rgb[0]);
        point.setField(Dimension::Id::Green, p.rgb[1]);
        point.setField(Dimension::Id::Blue, p.rgb[2]);
    }

    if (h.hasInfrared())
    {
        point.setField(Dimension::Id::Infrared, p.rgb[3]);
    }

    if (m_extraDims.size())
    {
        LeExtractor extractor((const char *)p.extra_bytes, p.num_extra_bytes);
        loadExtraDims(extractor, point);
    }
}
#endif  // PDAL_HAVE_LASZIP


void LasReader::loadPointV14(PointRef& point, char *buf, size_t bufsize)
{
    LeExtractor istream(buf, bufsize);

    int32_t xi, yi, zi;
    istream >> xi >> yi >> zi;

    const LasHeader& h = m_header;

    double x = xi * h.scaleX() + h.offsetX();
    double y = yi * h.scaleY() + h.offsetY();
    double z = zi * h.scaleZ() + h.offsetZ();

    uint16_t intensity;
    uint8_t returnInfo;
    uint8_t flags;
    uint8_t classification;
    uint8_t user;
    int16_t scanAngle;
    uint16_t pointSourceId;
    double gpsTime;

    istream >> intensity >> returnInfo >> flags >> classification >> user >>
        scanAngle >> pointSourceId >> gpsTime;

    uint8_t returnNum = returnInfo & 0x0F;
    uint8_t numReturns = (returnInfo >> 4) & 0x0F;
    uint8_t classFlags = flags & 0x0F;
    uint8_t scanChannel = (flags >> 4) & 0x03;
    uint8_t scanDirFlag = (flags >> 6) & 0x01;
    uint8_t flight = (flags >> 7) & 0x01;

    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
    point.setField(Dimension::Id::Intensity, intensity);
    point.setField(Dimension::Id::ReturnNumber, returnNum);
    point.setField(Dimension::Id::NumberOfReturns, numReturns);
    point.setField(Dimension::Id::ClassFlags, classFlags);
    point.setField(Dimension::Id::ScanChannel, scanChannel);
    point.setField(Dimension::Id::ScanDirectionFlag, scanDirFlag);
    point.setField(Dimension::Id::EdgeOfFlightLine, flight);
    point.setField(Dimension::Id::Classification, classification);
    point.setField(Dimension::Id::ScanAngleRank, scanAngle * .006);
    point.setField(Dimension::Id::UserData, user);
    point.setField(Dimension::Id::PointSourceId, pointSourceId);
    point.setField(Dimension::Id::GpsTime, gpsTime);

    if (h.hasColor())
    {
        uint16_t red, green, blue;
        istream >> red >> green >> blue;
        point.setField(Dimension::Id::Red, red);
        point.setField(Dimension::Id::Green, green);
        point.setField(Dimension::Id::Blue, blue);
    }

    if (h.hasInfrared())
    {
        uint16_t nearInfraRed;

        istream >> nearInfraRed;
        point.setField(Dimension::Id::Infrared, nearInfraRed);
    }

    if (m_extraDims.size())
        loadExtraDims(istream, point);
}


void LasReader::loadExtraDims(LeExtractor& istream, PointRef& point)
{
    for (auto& dim : m_extraDims)
    {
        // Dimension type of None is undefined and unprocessed
        if (dim.m_dimType.m_type == Dimension::Type::None)
        {
            istream.skip(dim.m_size);
            continue;
        }

        Everything e = Utils::extractDim(istream, dim.m_dimType.m_type);
        if (dim.m_dimType.m_xform.nonstandard())
        {
            double d = Utils::toDouble(e, dim.m_dimType.m_type);
            d = d * dim.m_dimType.m_xform.m_scale.m_val +
                dim.m_dimType.m_xform.m_offset.m_val;
            point.setField(dim.m_dimType.m_id, d);
        }
        else
            point.setField(dim.m_dimType.m_id, dim.m_dimType.m_type, &e);
    }
}


void LasReader::done(PointTableRef)
{
#ifdef PDAL_HAVE_LASZIP
    if (m_laszip)
    {
        handleLaszip(laszip_close_reader(m_laszip));
        handleLaszip(laszip_destroy(m_laszip));
    }
#endif
    m_streamIf.reset();
}

} // namespace pdal
