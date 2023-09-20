/******************************************************************************
 * Copyright (c) 2015, Hobu Inc.
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include "Header.hpp"
#include "Srs.hpp"
#include "Summary.hpp"
#include "Utils.hpp"
#include "Vlr.hpp"

#include <pdal/PointRef.hpp>

#include <pdal/util/Extractor.hpp>
#include <pdal/util/Inserter.hpp>
#include <pdal/util/Utils.hpp>

namespace pdal
{

namespace
{
    using DT = Dimension::Type;
    const Dimension::Type lastypes[] = {
        DT::None, DT::Unsigned8, DT::Signed8, DT::Unsigned16, DT::Signed16,
        DT::Unsigned32, DT::Signed32, DT::Unsigned64, DT::Signed64,
        DT::Float, DT::Double
    };
}

namespace las
{

void setSummary(las::Header& header, const Summary& summary)
{
    header.setPointCount(summary.getTotalNumPoints());
    for (int i = 0; i < Header::ReturnCount; ++i)
        header.setPointsByReturn(i, summary.getReturnCount(i));
    if (summary.getTotalNumPoints() == 0)
    {
        header.bounds.minx = 0;
        header.bounds.maxx = 0;
        header.bounds.miny = 0;
        header.bounds.maxy = 0;
        header.bounds.minz = 0;
        header.bounds.maxz = 0;
    }
    else
        header.bounds = summary.getBounds();
}


void extractHeaderMetadata(const Header& h, MetadataNode& forward, MetadataNode& m)
{
    addForwardMetadata(forward, m, "major_version", h.versionMajor,
        "The major LAS version for the file, always 1 for now");
    addForwardMetadata(forward, m, "minor_version", h.versionMinor,
        "The minor LAS version for the file");
    addForwardMetadata(forward, m, "dataformat_id", h.pointFormat(),
        "LAS Point Data Format");
    addForwardMetadata(forward, m, "filesource_id", h.fileSourceId,
        "File Source ID (Flight Line Number if this file was derived from an original "
            "flight line).");
    if (h.versionAtLeast(1, 2))
    {
        // For some reason we've written global encoding as a base 64
        // encoded value in the past.  In an effort to standardize things,
        // I'm writing this as a special value, and will also write
        // global_encoding like we write all other header metadata.
        uint16_t globalEncoding = h.globalEncoding;
        m.addEncoded("global_encoding_base64", (uint8_t *)&globalEncoding, sizeof(globalEncoding),
            "Global Encoding: general property bit field.");
        addForwardMetadata(forward, m, "global_encoding", h.globalEncoding,
            "Global Encoding: general property bit field.");
    }

    addForwardMetadata(forward, m, "project_id", h.projectGuid, "Project ID.");
    addForwardMetadata(forward, m, "system_id", h.systemId, "Generating system ID.");
    addForwardMetadata(forward, m, "software_id", h.softwareId, "Generating software description.");
    addForwardMetadata(forward, m, "creation_doy", h.creationDoy,
        "Day, expressed as an unsigned short, on which this file was created. "
        "Day is computed as the Greenwich Mean Time (GMT) day. January 1 is "
        "considered day 1.");
    addForwardMetadata(forward, m, "creation_year", h.creationYear,
        "The year, expressed as a four digit number, in which the file was created.");
    addForwardMetadata(forward, m, "scale_x", h.scale.x, "The scale factor for X values.", 15);
    addForwardMetadata(forward, m, "scale_y", h.scale.y, "The scale factor for Y values.", 15);
    addForwardMetadata(forward, m, "scale_z", h.scale.z, "The scale factor for Z values.", 15);
    addForwardMetadata(forward, m, "offset_x", h.offset.x, "The offset for X values.", 15);
    addForwardMetadata(forward, m, "offset_y", h.offset.y, "The offset for Y values.", 15);
    addForwardMetadata(forward, m, "offset_z", h.offset.z, "The offset for Z values.", 15);

    m.add<bool>("compressed", h.dataCompressed(), "true if this LAS file is compressed");
    m.add("point_length", h.pointSize, "The size, in bytes, of each point records.");
    m.add("header_size", h.vlrOffset,
        "The size, in bytes, of the header block, including any extension by specific software.");
    m.add("dataoffset", h.pointOffset,
        "The actual number of bytes from the beginning of the file to the "
        "first field of the first point record data field. This data offset "
        "must be updated if any software adds data from the Public Header "
        "Block or adds/removes data to/from the Variable Length Records.");
    m.add<double>("minx", h.bounds.minx,
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("miny", h.bounds.miny,
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("minz", h.bounds.minz,
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxx", h.bounds.maxx,
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxy", h.bounds.maxy,
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<double>("maxz", h.bounds.maxz,
        "The max and min data fields are the actual unscaled extents of the "
        "LAS point file data, specified in the coordinate system of the LAS "
        "data.");
    m.add<point_count_t>("count", h.pointCount(),
        "This field contains the total number of point records within the file.");
}

void extractSrsMetadata(const Srs& srs, MetadataNode& m)
{
    const std::string s = srs.geotiffString();
    if (s.size())
        m.add("gtiff", s, "GTifPrint output of GEOTIFF keys");
}

void addVlrMetadata(const Vlr& vlr, std::string name, MetadataNode& forward, MetadataNode& m)
{
    const size_t DataLenMax = 1000000;

    // We don't extract metadata from large VLRs.
    if (vlr.dataSize() > DataLenMax)
        return;

    if (vlr.userId == las::PdalUserId)
    {
        if (vlr.recordId == las::PdalMetadataRecordId)
            name = "pdal_metadata";
        else if (vlr.recordId == las::PdalPipelineRecordId)
            name = "pdal_pipeline";
        m.addWithType(name, std::string(vlr.data(), vlr.dataSize()), "json", vlr.description);
        return;
    }
    MetadataNode vlrNode(name);
    vlrNode.addEncoded("data", (const unsigned char *)vlr.data(), vlr.dataSize(), vlr.description);
    vlrNode.add("user_id", vlr.userId, "User ID of the record or pre-defined value "
        "from the specification.");
    vlrNode.add("record_id", vlr.recordId, "Record ID specified by the user.");
    vlrNode.add("description", vlr.description);
    m.add(vlrNode);

    if (vlr.userId == las::TransformUserId ||
        vlr.userId == las::LaszipUserId ||
        vlr.userId == las::LiblasUserId)
        return;
    if (vlr.userId == las::SpecUserId &&
        vlr.recordId != las::ClassLookupRecordId &&
        vlr.recordId != las::TextDescriptionRecordId)
        return;
    forward.add(vlrNode);
}

uint8_t lasType(Dimension::Type type, int fieldCnt)
{
    if (fieldCnt < 1 || fieldCnt > 3)
        return 0;

    uint8_t lastype = 0;
    for (size_t i = 0; i < sizeof(lastypes) / sizeof(lastypes[0]); ++i)
        if (type == lastypes[i])
        {
            lastype = i;
            break;
        }

    // 0 is the "undocumented" type, which we don't really support.
    if (lastype == 0)
        return 0;
    return 10 * (fieldCnt - 1) + lastype;
}


void ExtraBytesIf::setType(uint8_t lastype)
{
    m_fieldCnt = 1;
    while (lastype > 10)
    {
        m_fieldCnt++;
        lastype -= 10;
    }

    m_type = lastypes[lastype];
    if (m_type == Dimension::Type::None)
        m_fieldCnt = 0;
}


void ExtraBytesIf::appendTo(std::vector<char>& ebBytes)
{
    size_t offset = ebBytes.size();
    ebBytes.resize(ebBytes.size() + ExtraBytesSpecSize);
    LeInserter inserter(ebBytes.data() + offset, ExtraBytesSpecSize);

    uint8_t lastype = lasType(m_type, m_fieldCnt);
    uint8_t options = lastype ? 0 : m_size;

    inserter << (uint16_t)0 << lastype << options;
    inserter.put(m_name, 32);
    inserter << (uint32_t)0;  // Reserved.
    for (size_t i = 0; i < 3; ++i)
        inserter << (uint64_t)0;  // No data field.
    for (size_t i = 0; i < 3; ++i)
        inserter << (double)0.0; // Min.
    for (size_t i = 0; i < 3; ++i)
        inserter << (double)0.0; // Max.
    for (size_t i = 0; i < 3; ++i)
        inserter << m_scale[i];
    for (size_t i = 0; i < 3; ++i)
        inserter << m_offset[i];
    inserter.put(m_description, 32);
}


void ExtraBytesIf::readFrom(const char *buf)
{
    LeExtractor extractor(buf, ExtraBytesSpecSize);
    uint16_t dummy16;
    uint32_t dummy32;
    uint64_t dummy64;
    double dummyd;
    uint8_t options;
    uint8_t type;

    uint8_t SCALE_MASK = 1 << 3;
    uint8_t OFFSET_MASK = 1 << 4;

    extractor >> dummy16 >> type >> options;
    extractor.get(m_name, 32);
    extractor >> dummy32;
    for (size_t i = 0; i < 3; ++i)
        extractor >> dummy64;  // No data field.
    for (size_t i = 0; i < 3; ++i)
        extractor >> dummyd;  // Min.
    for (size_t i = 0; i < 3; ++i)
        extractor >> dummyd;  // Max.
    for (size_t i = 0; i < 3; ++i)
        extractor >> m_scale[i];
    for (size_t i = 0; i < 3; ++i)
        extractor >> m_offset[i];
    extractor.get(m_description, 32);

    setType(type);
    if (m_type == Dimension::Type::None)
        m_size = options;
    if (!(options & SCALE_MASK))
        for (size_t i = 0; i < 3; ++i)
            m_scale[i] = 1.0;
    if (!(options & OFFSET_MASK))
        for (size_t i = 0; i < 3; ++i)
            m_offset[i] = 0.0;
}


// NOTE: You must make sure that bufsize is a multiple of ExtraBytesSpecSize before calling.
std::vector<ExtraDim> ExtraBytesIf::toExtraDims(const char *buf, size_t bufsize, int baseSize)
{
    std::vector<ExtraDim> eds;

    int byteOffset = baseSize;
    while (bufsize)
    {
        ExtraBytesIf spec;
        spec.readFrom(buf);

        if (spec.m_type == Dimension::Type::None)
        {
            ExtraDim ed(spec.m_name, spec.m_size, byteOffset);
            eds.push_back(ed);
            byteOffset += ed.m_size;
        }
        else if (spec.m_fieldCnt == 1)
        {
            ExtraDim ed(spec.m_name, spec.m_type, byteOffset, spec.m_scale[0], spec.m_offset[0]);
            eds.push_back(ed);
            byteOffset += ed.m_size;
        }
        else
        {
            for (size_t i = 0; i < spec.m_fieldCnt; ++i)
            {
                ExtraDim ed(spec.m_name + std::to_string(i), spec.m_type, byteOffset,
                    spec.m_scale[i], spec.m_offset[i]);
                eds.push_back(ed);
                byteOffset += ed.m_size;
            }
        }
        bufsize -= ExtraBytesSpecSize;
        buf += ExtraBytesSpecSize;
    }
    return eds;
}


const Dimension::IdList& pdrfDims(int pdrf)
{
    if (pdrf < 0 || pdrf > 10)
        pdrf = 10;

    using D = Dimension::Id;
    static const Dimension::IdList dims[11]
    {
        // 0
        { D::X, D::Y, D::Z, D::Intensity, D::ReturnNumber, D::NumberOfReturns, D::ScanDirectionFlag,
          D::EdgeOfFlightLine, D::Classification, D::ScanAngleRank, D::UserData, D::PointSourceId },
        // 1
        { D::X, D::Y, D::Z, D::Intensity, D::ReturnNumber, D::NumberOfReturns, D::ScanDirectionFlag,
          D::EdgeOfFlightLine, D::Classification, D::ScanAngleRank, D::UserData, D::PointSourceId,
          D::GpsTime },
        // 2
        { D::X, D::Y, D::Z, D::Intensity, D::ReturnNumber, D::NumberOfReturns, D::ScanDirectionFlag,
          D::EdgeOfFlightLine, D::Classification, D::ScanAngleRank, D::UserData, D::PointSourceId,
          D::Red, D::Green, D::Blue },
        // 3
        { D::X, D::Y, D::Z, D::Intensity, D::ReturnNumber, D::NumberOfReturns, D::ScanDirectionFlag,
          D::EdgeOfFlightLine, D::Classification, D::ScanAngleRank, D::UserData, D::PointSourceId,
          D::GpsTime, D::Red, D::Green, D::Blue },
        {},
        {},
        // 6
        { D::X, D::Y, D::Z, D::Intensity, D::ReturnNumber, D::NumberOfReturns, D::ScanDirectionFlag,
          D::EdgeOfFlightLine, D::Classification, D::ScanAngleRank, D::UserData, D::PointSourceId,
          D::GpsTime, D::ScanChannel, D::ClassFlags },
        // 7
        { D::X, D::Y, D::Z, D::Intensity, D::ReturnNumber, D::NumberOfReturns, D::ScanDirectionFlag,
          D::EdgeOfFlightLine, D::Classification, D::ScanAngleRank, D::UserData, D::PointSourceId,
          D::GpsTime, D::ScanChannel, D::ClassFlags, D::Red, D::Green, D::Blue },
        // 8
        { D::X, D::Y, D::Z, D::Intensity, D::ReturnNumber, D::NumberOfReturns, D::ScanDirectionFlag,
          D::EdgeOfFlightLine, D::Classification, D::ScanAngleRank, D::UserData, D::PointSourceId,
          D::GpsTime, D::ScanChannel, D::ClassFlags, D::Red, D::Green, D::Blue, D::Infrared },
        {},
        {}
    };
    return dims[pdrf];
}

std::string generateSoftwareId()
{
    std::string ver(Config::versionString());
    std::stringstream oss;
    std::ostringstream revs;
    revs << Config::sha1();
    oss << "PDAL " << ver << " (" << revs.str().substr(0, 6) <<")";
    return oss.str();
}

// Throws las::error. Be sure to catch it.
std::vector<ExtraDim> parse(const StringList& dimString, bool allOk)
{
    std::vector<ExtraDim> extraDims;
    bool all = false;

    int byteOffset = 0;
    for (auto& dim : dimString)
    {
        if (dim == "all")
        {
            // We only accept all for LasWriter.
            if (!allOk)
                throw error("Invalid extra dimension specified: '" + dim +
                    "'.  Need <dimension>=<type>.  See documentation "
                    " for details.");
            all = true;
            continue;
        }

        StringList s = Utils::split2(dim, '=');
        if (s.size() != 2)
            throw error("Invalid extra dimension specified: '" + dim +
                "'.  Need <dimension>=<type>.  See documentation "
                " for details.");
        Utils::trim(s[0]);
        Utils::trim(s[1]);
        Dimension::Type type = Dimension::type(s[1]);
        if (type == Dimension::Type::None)
            throw error("Invalid extra dimension type specified: '" + dim +
                "'.  Need <dimension>=<type>.  See documentation "
                " for details.");
        ExtraDim ed(s[0], type, byteOffset);
        extraDims.push_back(ed);
        byteOffset += ed.m_size;
    }

    if (all)
    {
        if (extraDims.size())
            throw error("Can't specify specific extra dimensions with "
                "special 'all' keyword.");
        extraDims.push_back(ExtraDim("all", Dimension::Type::None, 0));
    }

    return extraDims;
}

// LAS loader driver

LoaderDriver::LoaderDriver(int pdrf, const Scaling& scaling, const ExtraDims& dims)
{
    init(pdrf, scaling, dims);
}

void LoaderDriver::init(int pdrf, const Scaling& scaling, const ExtraDims& dims)
{
    switch (pdrf)
    {
    case 0:
        m_loaders.push_back(PointLoaderPtr(new V10BaseLoader(scaling)));
        break;
    case 1:
        m_loaders.push_back(PointLoaderPtr(new V10BaseLoader(scaling)));
        m_loaders.push_back(PointLoaderPtr(new GpstimeLoader(20)));
        break;
    case 2:
        m_loaders.push_back(PointLoaderPtr(new V10BaseLoader(scaling)));
        m_loaders.push_back(PointLoaderPtr(new ColorLoader(20)));
        break;
    case 3:
        m_loaders.push_back(PointLoaderPtr(new V10BaseLoader(scaling)));
        m_loaders.push_back(PointLoaderPtr(new GpstimeLoader(20)));
        m_loaders.push_back(PointLoaderPtr(new ColorLoader(28)));
        break;
    case 6:
        m_loaders.push_back(PointLoaderPtr(new V14BaseLoader(scaling)));
        break;
    case 7:
        m_loaders.push_back(PointLoaderPtr(new V14BaseLoader(scaling)));
        m_loaders.push_back(PointLoaderPtr(new ColorLoader(30)));
        break;
    case 8:
        m_loaders.push_back(PointLoaderPtr(new V14BaseLoader(scaling)));
        m_loaders.push_back(PointLoaderPtr(new ColorLoader(30)));
        m_loaders.push_back(PointLoaderPtr(new NirLoader(36)));
        break;
    }
    if (dims.size())
        m_loaders.push_back(PointLoaderPtr(new ExtraDimLoader(dims)));
}

bool LoaderDriver::load(PointRef& point, const char *buf, int bufsize)
{
    for (PointLoaderPtr& l : m_loaders)
        l->load(point, buf, bufsize);
    return true;
}

bool LoaderDriver::pack(const PointRef& point, char *buf, int bufsize)
{
    for (PointLoaderPtr& l : m_loaders)
        l->pack(point, buf, bufsize);
    return true;
}

V10BaseLoader::V10BaseLoader(const Scaling& scaling) : m_scaling(scaling)
{}

void V10BaseLoader::load(PointRef& point, const char *buf, int bufsize)
{
    LeExtractor istream(buf, bufsize);

    int32_t xi, yi, zi;
    istream >> xi >> yi >> zi;

    double x = m_scaling.m_xXform.fromScaled(xi);
    double y = m_scaling.m_yXform.fromScaled(yi);
    double z = m_scaling.m_zXform.fromScaled(zi);

    uint16_t intensity;
    uint8_t flags;
    uint8_t classification;
    int8_t scanAngleRank;
    uint8_t user;
    uint16_t pointSourceId;

    istream >> intensity >> flags >> classification >> scanAngleRank >> user >> pointSourceId;

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
}

void V10BaseLoader::pack(const PointRef& point, char *buf, int bufsize)
{
    LeInserter ostream(buf, bufsize);


    auto converter = [](double val, Dimension::Id dim) -> int32_t
    {
        int32_t i(0);

        if (!Utils::numericCast(val, i))
            throw std::runtime_error("Unable to convert scaled value (" +
                Utils::toString(val) + ") to "
                "int32 for dimension '" + Dimension::name(dim) );
        return i;
    };

    double xOrig = point.getFieldAs<double>(Dimension::Id::X);
    double yOrig = point.getFieldAs<double>(Dimension::Id::Y);
    double zOrig = point.getFieldAs<double>(Dimension::Id::Z);

    int32_t x = converter(m_scaling.m_xXform.toScaled(xOrig), Dimension::Id::X);
    int32_t y = converter(m_scaling.m_yXform.toScaled(yOrig), Dimension::Id::Y);
    int32_t z = converter(m_scaling.m_zXform.toScaled(zOrig), Dimension::Id::Z);

    ostream << x;
    ostream << y;
    ostream << z;

    int returnNum = point.getFieldAs<int>(Dimension::Id::ReturnNumber);
    int numReturns = point.getFieldAs<int>(Dimension::Id::NumberOfReturns);
    int scanDir = point.getFieldAs<int>(Dimension::Id::ScanDirectionFlag);
    int eofFlag = point.getFieldAs<int>(Dimension::Id::EdgeOfFlightLine);

    uint8_t flags = returnNum | (numReturns << 3) | (scanDir << 6) | (eofFlag << 7);
    uint8_t classification = point.getFieldAs<uint8_t>(Dimension::Id::Classification);
    int8_t scanAngleRank = point.getFieldAs<int8_t>(Dimension::Id::ScanAngleRank);
    uint8_t user = point.getFieldAs<uint8_t>(Dimension::Id::UserData);
    uint16_t pointSourceId = point.getFieldAs<uint16_t>(Dimension::Id::PointSourceId);

    ostream << flags << classification << scanAngleRank << user << pointSourceId;
}

void GpstimeLoader::load(PointRef& point, const char *buf, int bufsize)
{
    buf += m_offset;
    bufsize -= m_offset;

    LeExtractor istream(buf, bufsize);

    double time;
    istream >> time;
    point.setField(Dimension::Id::GpsTime, time);
}

void GpstimeLoader::pack(const PointRef& point, char *buf, int bufsize)
{
    buf += m_offset;
    bufsize -= m_offset;
    LeInserter ostream(buf, bufsize);

    ostream << point.getFieldAs<double>(Dimension::Id::GpsTime);
}

void ColorLoader::load(PointRef& point, const char *buf, int bufsize)
{
    buf += m_offset;
    bufsize -= m_offset;

    LeExtractor istream(buf, bufsize);

    uint16_t red, green, blue;
    istream >> red >> green >> blue;
    point.setField(Dimension::Id::Red, red);
    point.setField(Dimension::Id::Green, green);
    point.setField(Dimension::Id::Blue, blue);
}

void ColorLoader::pack(const PointRef& point, char *buf, int bufsize)
{
    buf += m_offset;
    bufsize -= m_offset;
    LeInserter ostream(buf, bufsize);

    ostream << point.getFieldAs<uint16_t>(Dimension::Id::Red);
    ostream << point.getFieldAs<uint16_t>(Dimension::Id::Green);
    ostream << point.getFieldAs<uint16_t>(Dimension::Id::Blue);
}

V14BaseLoader::V14BaseLoader(const Scaling& scaling) : m_scaling(scaling)
{}

void V14BaseLoader::load(PointRef& point, const char *buf, int bufsize)
{
    LeExtractor istream(buf, bufsize);

    int32_t xi, yi, zi;
    istream >> xi >> yi >> zi;

    double x = m_scaling.m_xXform.fromScaled(xi);
    double y = m_scaling.m_yXform.fromScaled(yi);
    double z = m_scaling.m_zXform.fromScaled(zi);

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
}

void V14BaseLoader::pack(const PointRef& point, char *buf, int bufsize)
{
    LeInserter ostream(buf, bufsize);

    auto converter = [](double val, Dimension::Id dim) -> int32_t
    {
        int32_t i(0);

        if (!Utils::numericCast(val, i))
            throw std::runtime_error("Unable to convert scaled value (" +
                Utils::toString(val) + ") to "
                "int32 for dimension '" + Dimension::name(dim) );
        return i;
    };

    double xOrig = point.getFieldAs<double>(Dimension::Id::X);
    double yOrig = point.getFieldAs<double>(Dimension::Id::Y);
    double zOrig = point.getFieldAs<double>(Dimension::Id::Z);

    int32_t x = converter(m_scaling.m_xXform.toScaled(xOrig), Dimension::Id::X);
    int32_t y = converter(m_scaling.m_yXform.toScaled(yOrig), Dimension::Id::Y);
    int32_t z = converter(m_scaling.m_zXform.toScaled(zOrig), Dimension::Id::Z);

    ostream << x << y << z;

    uint16_t intensity = point.getFieldAs<uint16_t>(Dimension::Id::Intensity);
    int returnNum = point.getFieldAs<int>(Dimension::Id::ReturnNumber);
    int numReturns = point.getFieldAs<int>(Dimension::Id::NumberOfReturns);
    uint8_t returnInfo = returnNum | (numReturns << 4);

    uint8_t scanChannel = point.getFieldAs<uint8_t>(Dimension::Id::ScanChannel);
    uint8_t scanDirFlag = point.getFieldAs<uint8_t>(Dimension::Id::ScanDirectionFlag);
    uint8_t flight = point.getFieldAs<uint8_t>(Dimension::Id::EdgeOfFlightLine);
    uint8_t classification = point.getFieldAs<uint8_t>(Dimension::Id::Classification);

    uint8_t flags;
    uint8_t classFlags = 0;
    if (point.hasDim(Dimension::Id::ClassFlags))
        classFlags = point.getFieldAs<uint8_t>(Dimension::Id::ClassFlags);
    else
        classFlags = classification >> 5;

    flags = (classFlags & 0x0F) |
            ((scanChannel & 0x03) << 4) |
            ((scanDirFlag & 0x01) << 6) |
            ((flight & 0x01) << 7);

    uint8_t user = point.getFieldAs<uint8_t>(Dimension::Id::UserData);
    int16_t scanAngle = static_cast<int16_t>(std::round(
        point.getFieldAs<float>(Dimension::Id::ScanAngleRank) / .006f));
    uint16_t pointSourceId = point.getFieldAs<uint16_t>(Dimension::Id::PointSourceId);
    double gpsTime = point.getFieldAs<double>(Dimension::Id::GpsTime);

    ostream << intensity << returnInfo << flags << classification << user <<
        scanAngle << pointSourceId << gpsTime;
}

void NirLoader::load(PointRef& point, const char *buf, int bufsize)
{
    buf += m_offset;
    bufsize -= m_offset;

    LeExtractor istream(buf, bufsize);

    uint16_t nearInfraRed;

    istream >> nearInfraRed;
    point.setField(Dimension::Id::Infrared, nearInfraRed);
}

void NirLoader::pack(const PointRef& point, char *buf, int bufsize)
{
    buf += m_offset;
    bufsize -= m_offset;

    LeInserter ostream(buf, bufsize);

    ostream << point.getFieldAs<uint16_t>(Dimension::Id::Infrared);
}

void ExtraDimLoader::load(PointRef& point, const char *buf, int bufsize)
{
    for (const ExtraDim& d : m_extraDims)
    {
        const DimType& dt = d.m_dimType;
        LeExtractor istream(buf + d.m_byteOffset, bufsize - d.m_byteOffset);
        Everything e = Utils::extractDim(istream, dt.m_type);
        if (dt.m_xform.nonstandard())
            point.setField(dt.m_id, dt.m_xform.fromScaled(Utils::toDouble(e, dt.m_type)));
        else
            point.setField(dt.m_id, dt.m_type, &e);
    }
}

void ExtraDimLoader::pack(const PointRef& point, char *buf, int bufsize)
{
    Everything e;
    for (const ExtraDim& d : m_extraDims)
    {
        const DimType& dt = d.m_dimType;
        LeInserter ostream(buf + d.m_byteOffset, bufsize - d.m_byteOffset);

        point.getField((char *)&e, d.m_dimType.m_id, d.m_dimType.m_type);
        if (d.m_dimType.m_xform.nonstandard())
            ostream << d.m_dimType.m_xform.toScaled(Utils::toDouble(e, d.m_dimType.m_type));
        else
            Utils::insertDim(ostream, d.m_dimType.m_type, e);
    }
}

} // namespace las
} // namespace pdal
