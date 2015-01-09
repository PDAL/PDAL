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

#include "LasReader.hpp"

#include <sstream>
#include <string.h>

#include <pdal/Charbuf.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/IStream.hpp>
#include <pdal/QuickInfo.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Metadata.hpp>

#include "LasHeader.hpp"
#include "VariableLengthRecord.hpp"
#include "ZipPoint.hpp"
#include "GeotiffSupport.hpp"

namespace pdal
{

QuickInfo LasReader::inspect()
{
    QuickInfo qi;
    PointContext ctx;

    addDimensions(ctx);
    initialize();

    Dimension::IdList dims = ctx.dims();
    for (auto di = dims.begin(); di != dims.end(); ++di)
        qi.m_dimNames.push_back(ctx.dimName(*di));
    qi.m_pointCount =
        Utils::saturation_cast<point_count_t>(m_lasHeader.pointCount());
    qi.m_bounds = m_lasHeader.getBounds();
    qi.m_srs = getSrsFromVlrs();
    return qi;
}


void LasReader::initialize()
{
    m_streamFactory = createFactory();
    m_istream = &(m_streamFactory->allocate());

    m_istream->seekg(0);
    ILeStream in(m_istream);
    in >> m_lasHeader;

    if (!m_lasHeader.pointFormatSupported())
    {
        std::ostringstream oss;
        oss << "Unsupported LAS input point format: " <<
            (int)m_lasHeader.pointFormat() << ".";
       throw pdal_error(oss.str());
    }

    // We need to read the VLRs in initialize() because they may contain an
    // extra-bytes VLR that is needed to determine dimensions.
    m_istream->seekg(m_lasHeader.vlrOffset());
    for (size_t i = 0; i < m_lasHeader.vlrCount(); ++i)
    {
        VariableLengthRecord r;
        in >> r;
        m_vlrs.push_back(std::move(r));
    }

    if (m_lasHeader.versionAtLeast(1, 4))
    {
        m_istream->seekg(m_lasHeader.eVlrOffset());
        for (size_t i = 0; i < m_lasHeader.eVlrCount(); ++i)
        {
            ExtVariableLengthRecord r;
            in >> r;
            m_vlrs.push_back(std::move(r));
        }
    }
    fixupVlrs();
}


void LasReader::ready(PointContext ctx, MetadataNode& m)
{
    m_index = 0;

    setSrsFromVlrs(m);
    extractHeaderMetadata(m);

    if (m_lasHeader.compressed())
    {
#ifdef PDAL_HAVE_LASZIP
        VariableLengthRecord *vlr = findVlr(LASZIP_USER_ID, LASZIP_RECORD_ID);
        m_zipPoint.reset(new ZipPoint(vlr));

        if (!m_unzipper)
        {
            m_unzipper.reset(new LASunzipper());

            m_istream->seekg(m_lasHeader.pointOffset(), std::ios::beg);

            // Once we open the zipper, don't touch the stream until the
            // zipper is closed or bad things happen.
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


Options LasReader::getDefaultOptions()
{
    Option option1("filename", "", "file to read from");
    Options options(option1);
    return options;
}


void LasReader::extractHeaderMetadata(MetadataNode& m)
{
    m.add<bool>("compressed", m_lasHeader.compressed(),
        "true if this LAS file is compressed");
    m.add<uint32_t>("major_version",
        static_cast<uint32_t>(m_lasHeader.versionMajor()),
        "The major LAS version for the file, always 1 for now");
    m.add<uint32_t>("minor_version",
        static_cast<uint32_t>(m_lasHeader.versionMinor()),
        "The minor LAS version for the file");
    if (m_lasHeader.versionAtLeast(1, 1))
        m.add<uint32_t>("filesource_id",
            static_cast<uint32_t>(m_lasHeader.fileSourceId()),
            "File Source ID (Flight Line Number if this file was derived from "
            "an original flight line).");
    if (m_lasHeader.versionAtLeast(1, 2))
    {
        uint16_t globalEncoding = m_lasHeader.globalEncoding();
        m.addEncoded("global_encoding", (uint8_t *)&globalEncoding,
            sizeof(globalEncoding),
            "Global Encoding: general property bit field.");
    }
    m.add<boost::uuids::uuid>("project_id",
         m_lasHeader.projectId(), "Project ID.");
    m.add<std::string>("system_id", m_lasHeader.systemId());
    m.add<std::string>("software_id", m_lasHeader.softwareId(),
        "Generating software description.");
    m.add<uint32_t>("creation_doy",
        static_cast<uint32_t>(m_lasHeader.creationDOY()),
        "Day, expressed as an unsigned short, on which this file was created. "
        "Day is computed as the Greenwich Mean Time (GMT) day. January 1 is "
        "considered day 1.");
    m.add<uint32_t>("creation_year",
        static_cast<uint32_t>(m_lasHeader.creationYear()),
        "The year, expressed as a four digit number, in which the file was "
        "created.");
    m.add<uint32_t>("header_size",
        static_cast<uint32_t>(m_lasHeader.vlrOffset()),
        "The size, in bytes, of the header block, including any extension "
        "by specific software.");
    m.add<uint32_t>("dataoffset",
        static_cast<uint32_t>(m_lasHeader.pointOffset()),
        "The actual number of bytes from the beginning of the file to the "
        "first field of the first point record data field. This data offset "
        "must be updated if any software adds data from the Public Header "
        "Block or adds/removes data to/from the Variable Length Records.");
    m.add<double>("scale_x", m_lasHeader.scaleX(),
        "The scale factor for X values.");
    m.add<double>("scale_y", m_lasHeader.scaleY(),
        "The scale factor for Y values.");
    m.add<double>("scale_z", m_lasHeader.scaleZ(),
        "The scale factor for Z values.");
    m.add<double>("offset_x", m_lasHeader.offsetX(),
        "The offset for X values.");
    m.add<double>("offset_y", m_lasHeader.offsetY(),
        "The offset for Y values.");
    m.add<double>("offset_z", m_lasHeader.offsetZ(),
        "The offset for Z values.");
    m.add<double>("minx", m_lasHeader.minX(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("miny", m_lasHeader.minY(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("minz", m_lasHeader.minZ(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxx", m_lasHeader.maxX(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxy", m_lasHeader.maxY(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxz", m_lasHeader.maxZ(),
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<uint32_t>("count",
        m_lasHeader.pointCount(), "This field contains the total "
        "number of point records within the file.");
}


void LasReader::fixupVlrs()
{
    const size_t KEY_SIZE = 8;
    char zeros[8] = {};

    // There is currently only one fixup - for the geotiff directory VLR.
    VariableLengthRecord *vlr =
        findVlr(TRANSFORM_USER_ID, GEOTIFF_DIRECTORY_RECORD_ID);
    if (vlr)
    {
        while (vlr->dataLen() > 8)
        {
            // If the key at the end has a zero value, remove it
            // by resizing the array and decrementing the size.
            char *testPos = const_cast<char *>(
                vlr->data() + vlr->dataLen() - KEY_SIZE);
            if (memcmp(zeros, testPos, KEY_SIZE))
                break;
            uint16_t size;
            vlr->setDataLen(vlr->dataLen() - 8);

            // Reduce the size of the data by one.  The size field is
            // at offset 6 from the beginning of the data.
            memcpy((void *)&size, vlr->data() + 6, 2);
            size = le16toh(size);
            size--;
            size = htole16(size);
            memcpy((void *)(vlr->data()  + 6), &size, 2);
        }
    }
}


void LasReader::setSrsFromVlrs(MetadataNode& m)
{
    // If the user is already overriding this by setting it on the stage, we'll
    // take their overridden value
    SpatialReference srs = getSpatialReference();

    if (srs.getWKT(pdal::SpatialReference::eCompoundOK).empty())
        setSpatialReference(m, getSrsFromVlrs());
}


SpatialReference LasReader::getSrsFromVlrs()
{
    SpatialReference srs = getSrsFromWktVlr();
    if (srs.empty())
        srs = getSrsFromGeotiffVlr();
    return srs;
}


VariableLengthRecord *LasReader::findVlr(const std::string& userId,
    uint16_t recordId)
{
    for (auto vi = m_vlrs.begin(); vi != m_vlrs.end(); ++vi)
    {
        VariableLengthRecord& vlr = *vi;
        if (vlr.matches(userId, recordId))
            return &vlr;
    }
    return NULL;
}


SpatialReference LasReader::getSrsFromWktVlr()
{
    SpatialReference srs;

    VariableLengthRecord *vlr = findVlr(TRANSFORM_USER_ID, WKT_RECORD_ID);
    if (!vlr)
        vlr = findVlr(LIBLAS_USER_ID, WKT_RECORD_ID);
    if (!vlr || vlr->dataLen() == 0)
        return srs;

    // There is supposed to be a NULL byte at the end of the data,
    // but sometimes there isn't because some people don't follow the
    // rules.  If there is a NULL byte, don't stick it in the
    // wkt string.
    size_t len = vlr->dataLen();
    const char *c = vlr->data() + len - 1;
    if (*c == 0)
        len--;
    std::string wkt(vlr->data(), len);
    srs.setWKT(wkt);
    return srs;
}


SpatialReference LasReader::getSrsFromGeotiffVlr()
{
    SpatialReference srs;

#ifdef PDAL_HAVE_LIBGEOTIFF
    GeotiffSupport geotiff;
    geotiff.resetTags();

    VariableLengthRecord *vlr;

    vlr = findVlr(TRANSFORM_USER_ID, GEOTIFF_DIRECTORY_RECORD_ID);
    // We must have a directory entry.
    if (!vlr)
        return srs;
    geotiff.setKey(vlr->recordId(), (void *)vlr->data(), vlr->dataLen(),
        STT_SHORT);

    vlr = findVlr(TRANSFORM_USER_ID, GEOTIFF_DOUBLES_RECORD_ID);
    if (vlr)
        geotiff.setKey(vlr->recordId(), (void *)vlr->data(), vlr->dataLen(),
            STT_DOUBLE);
    vlr = findVlr(TRANSFORM_USER_ID, GEOTIFF_ASCII_RECORD_ID);
    if (vlr)
        geotiff.setKey(vlr->recordId(), (void *)vlr->data(), vlr->dataLen(),
            STT_ASCII);

    geotiff.setTags();
    srs.setFromUserInput(geotiff.getWkt(false, false));
#endif
    return srs;
}


void LasReader::extractVlrMetadata(MetadataNode& m)
{
    static const size_t DATA_LEN_MAX = 1000000;

    int i = 0;
    for (auto vi = m_vlrs.begin(); vi != m_vlrs.end(); ++vi, ++i)
    {
        const VariableLengthRecord& vlr = *vi;
        if (vlr.dataLen() > DATA_LEN_MAX)
            continue;

        std::ostringstream name;
        name << "vlr_" << i;
        MetadataNode vlrNode = m.addEncoded(name.str(),
            (const uint8_t *)vlr.data(), vlr.dataLen(), vlr.description());

        vlrNode.add("user_id", vlr.userId(),
            "User ID of the record or pre-defined value from the "
            "specification.");
        vlrNode.add("record_id", vlr.recordId(),
            "Record ID specified by the user.");
        vlrNode.add("description", vlr.description());
    }
}


void LasReader::addDimensions(PointContextRef ctx)
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
    if (m_lasHeader.hasInfrared())
        ctx.registerDim(Id::Infrared);
    if (m_lasHeader.versionAtLeast(1, 4))
        ctx.registerDim(Id::ScanChannel);
}


point_count_t LasReader::read(PointBuffer& data, point_count_t count)
{
    size_t pointByteCount = m_lasHeader.pointLen();
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
            loadPoint(data, (char *)m_zipPoint->m_lz_point_data.data(),
                pointByteCount);
        }
#else
        throw pdal_error("LASzip is not enabled for this "
            "LasReader::processBuffer");
#endif
    }
    else
    {
        m_istream->seekg(m_lasHeader.pointOffset());
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


void LasReader::loadPoint(PointBuffer& data, char *buf, size_t bufsize)
{
    if (m_lasHeader.versionAtLeast(1, 4))
        loadPointV14(data, buf, bufsize);
    else
        loadPointV10(data, buf, bufsize);
}


void LasReader::loadPointV10(PointBuffer& data, char *buf, size_t bufsize)
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

    if (returnNum == 0 || returnNum > 5)
        m_error.returnNumWarning(returnNum);

    if (numReturns == 0 || numReturns > 5)
        m_error.numReturnsWarning(numReturns);

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

void LasReader::loadPointV14(PointBuffer& data, char *buf, size_t bufsize)
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

    //ABELL - Need to do something with the classFlags;
    data.setField(Dimension::Id::X, nextId, x);
    data.setField(Dimension::Id::Y, nextId, y);
    data.setField(Dimension::Id::Z, nextId, z);
    data.setField(Dimension::Id::Intensity, nextId, intensity);
    data.setField(Dimension::Id::ReturnNumber, nextId, returnNum);
    data.setField(Dimension::Id::NumberOfReturns, nextId, numReturns);
    data.setField(Dimension::Id::ScanChannel, nextId, scanChannel);
    data.setField(Dimension::Id::ScanDirectionFlag, nextId, scanDirFlag);
    data.setField(Dimension::Id::EdgeOfFlightLine, nextId, flight);
    data.setField(Dimension::Id::Classification, nextId, classification);
    data.setField(Dimension::Id::ScanAngleRank, nextId, scanAngle * .006);
    data.setField(Dimension::Id::UserData, nextId, user);
    data.setField(Dimension::Id::PointSourceId, nextId, pointSourceId);
    data.setField(Dimension::Id::GpsTime, nextId, gpsTime);

    if (h.hasColor())
    {
        uint16_t red, green, blue;
        istream >> red >> green >> blue;
        data.setField(Dimension::Id::Red, nextId, red);
        data.setField(Dimension::Id::Green, nextId, green);
        data.setField(Dimension::Id::Blue, nextId, blue);
    }

    if (h.hasInfrared())
    {
        uint16_t nearInfraRed;

        istream >> nearInfraRed;
        data.setField(Dimension::Id::Infrared, nextId, nearInfraRed);
    }
}


void LasReader::done(PointContextRef ctx)
{
#ifdef PDAL_HAVE_LASZIP
    m_zipPoint.reset();
    m_unzipper.reset();
#endif
    if (m_istream)
        m_streamFactory->deallocate(*m_istream);
}

} // namespace pdal
