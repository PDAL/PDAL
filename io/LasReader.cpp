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

#include "LasHeader.hpp"
#include "LasReader.hpp"
#include "private/las/Header.hpp"
#include "private/las/Srs.hpp"
#include "private/las/Utils.hpp"
#include "private/las/Vlr.hpp"

#include <sstream>
#include <string.h>

#include <pdal/pdal_features.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/PointView.hpp>
#include <pdal/QuickInfo.hpp>
#include <pdal/util/Extractor.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/IStream.hpp>
#include <pdal/util/ProgramArgs.hpp>

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

struct LasReader::Options
{
    StringList extraDimSpec;
    //ABELL
    std::string compression;
    bool useEbVlr;
    StringList ignoreVLROption;
    bool fixNames;
    PointId start;
    bool nosrs;
    std::string srsConsumePreference;
};

struct LasReader::Private
{
    Options opts;
    las::Header header;
    LasHeader apiHeader;
    LazPerfVlrDecompressor *decompressor;
    std::vector<char> decompressorBuf;
    point_count_t index;
    las::VlrList ignoreVlrs;
    las::VlrList vlrs;
    las::Srs srs;
    std::vector<las::ExtraDim> extraDims;

    Private() : apiHeader(header, srs, vlrs), decompressor(nullptr), index(0)
    {}
};

LasReader::LasReader() : d(new Private)
{}


LasReader::~LasReader()
{
    delete d->decompressor;
}


void LasReader::addArgs(ProgramArgs& args)
{
    args.add("extra_dims", "Dimensions to assign to extra byte data",
        d->opts.extraDimSpec);
    args.add("compression", "Decompressor to use", d->opts.compression, "EITHER");
    args.add("use_eb_vlr", "Use extra bytes VLR for 1.0 - 1.3 files", d->opts.useEbVlr);
    args.add("ignore_vlr", "VLR userid/recordid to ignore", d->opts.ignoreVLROption );
    args.add("start", "Point at which reading should start (0-indexed).", d->opts.start);
    args.add("fix_dims", "Make invalid dimension names valid by changing "
        "invalid characters to '_'", d->opts.fixNames, true);
    args.add("nosrs", "Skip reading/processing file SRS", d->opts.nosrs);
    args.add("srs_consume_preference", "Preference order to read SRS VLRs",
        d->opts.srsConsumePreference, "wkt2, projjson, wkt1, geotiff");
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

const LasHeader& LasReader::header() const
{
    return d->apiHeader;
}

uint64_t LasReader::vlrData(const std::string& userId, uint16_t recordId, char const * & data)
{
    const las::Vlr *vlr = las::findVlr(userId, recordId, d->vlrs);
    if (!vlr)
	return 0;
    data = vlr->data();
    return vlr->dataVec.size();
}

point_count_t LasReader::getNumPoints() const
{
    return d->header.pointCount() - d->opts.start;
}

void LasReader::initialize(PointTableRef table)
{
    initializeLocal(table, m_metadata);
}

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
    if (!Utils::numericCast(d->header.pointCount(), qi.m_pointCount))
        qi.m_pointCount = (std::numeric_limits<point_count_t>::max)();
    qi.m_bounds = d->header.bounds;
    qi.m_srs = getSpatialReference();
    qi.m_valid = true;
    qi.m_metadata = m_metadata;

    done(table);

    return qi;
}


void LasReader::createStream()
{
    if (!m_streamIf)
        m_streamIf.reset(new LasStreamIf(m_filename));

    if (!m_streamIf->m_istream)
    {
        std::ostringstream oss;
        oss << "Unable to open stream for '"
            << m_filename <<"' with error '" << strerror(errno) <<"'";
        throw pdal_error(oss.str());
    }
}


void LasReader::initializeLocal(PointTableRef table, MetadataNode& m)
{
    try
    {
        d->extraDims = las::parse(d->opts.extraDimSpec, false);
    }
    catch (const las::error& err)
    {
        throwError(err.what());
    }

    std::string error;
    d->ignoreVlrs = las::parseIgnoreVlrs(d->opts.ignoreVLROption, error);
    if (error.size())
        throwError(error);

    createStream();
    std::istream *stream(m_streamIf->m_istream);

    stream->seekg(0);
    // Always try to read as if we have 1.4 size.
    char headerBuf[las::Header::Size14];
    stream->read(headerBuf, las::Header::Size14);
    if (stream->gcount() < (std::streamsize)las::Header::Size12)
        throwError("Couldn't read LAS header. File size insufficient.");
    d->header.fill(headerBuf, las::Header::Size14);

    uint64_t fileSize = Utils::fileSize(m_filename);
    StringList errors = d->header.validate(fileSize, d->opts.nosrs);
    if (errors.size())
        throwError(errors.front());

    // Verify
    if (!las::pointFormatSupported(d->header.pointFormat()))
        throwError("Unsupported LAS input point format: " +
            Utils::toString((int)d->header.pointFormat()) + ".");

    // Go peek into header and see if we are COPC
    // Clear the error state since we potentially over-read the header, leaving
    // the stream in error, when things are really fine for zero-point file.
    stream->clear();
    stream->seekg(377);
    char copcBuf[4] {};
    stream->read(copcBuf, 4);
    m.add("copc", ::memcmp(copcBuf, "copc", 4) == 0);

    // Read VLRs.
    // Clear the error state since the seek or read above may have failed but the file could
    // still be fine.
    stream->clear();
    stream->seekg(d->header.headerSize);

    char vlrHeaderBuf[las::Vlr::HeaderSize];
    std::vector<char> vlrBuf;
    for (uint32_t i = 0; i < d->header.vlrCount; ++i)
    {
        las::Vlr vlr;

        stream->read((char *)vlrHeaderBuf, las::Vlr::HeaderSize);
        if (stream->gcount() != las::Vlr::HeaderSize)
            throwError("Couldn't read VLR " + std::to_string(i + 1) + ". End of file reached.");
        vlr.fillHeader(vlrHeaderBuf);
        if ((uint64_t)stream->tellg() + vlr.promisedDataSize > d->header.pointOffset)
            throwError("VLR " + std::to_string(i + 1) +
                "(" + vlr.userId + "/" + std::to_string(vlr.recordId) + ") "
                "size too large -- flows into point data.");
        if (las::shouldIgnoreVlr(vlr, d->ignoreVlrs))
        {
            stream->seekg(vlr.promisedDataSize, std::ios::cur);
            continue;
        }
        vlr.dataVec.resize(vlr.promisedDataSize);
        stream->read(vlr.data(), vlr.promisedDataSize);

        if (stream->gcount() != (std::streamsize)vlr.promisedDataSize)
            throwError("Couldn't read VLR " + std::to_string(i + 1) + ". End of file reached.");
        d->vlrs.push_back(std::move(vlr));
    }

    // Read EVLRs if we have them.
    if (d->header.evlrOffset && d->header.evlrCount)
    {
        char evlrHeaderBuf[las::Evlr::HeaderSize];
        stream->seekg(d->header.evlrOffset);
        for (uint32_t i = 0; i < d->header.evlrCount; ++i)
        {
            las::Evlr evlr;

            stream->read((char *)evlrHeaderBuf, las::Evlr::HeaderSize);
            if (stream->gcount() != las::Evlr::HeaderSize)
                throwError("Couldn't read EVLR " + std::to_string(i + 1) +
                    ". End of file reached.");
            evlr.fillHeader(evlrHeaderBuf);

            if ((uint64_t)stream->tellg() + evlr.promisedDataSize > fileSize)
                throwError("EVLR " + std::to_string(i + 1) +
                    "(" + evlr.userId + "/" + std::to_string(evlr.recordId) + ") "
                    "size too large -- exceeds file size.");
            if (las::shouldIgnoreVlr(evlr, d->ignoreVlrs))
            {
                stream->seekg(evlr.promisedDataSize, std::ios::cur);
                continue;
            }
            evlr.dataVec.resize(evlr.promisedDataSize);
            stream->read(evlr.data(), evlr.promisedDataSize);

            //ABELL - Better error message.
            if (stream->gcount() != (std::streamsize)evlr.promisedDataSize)
                throwError("Couldn't read EVLR " + std::to_string(i + 1) +
                    ". End of file reached.");
            d->vlrs.push_back(std::move(evlr));
        }
    }

    if (!d->opts.nosrs)
        d->srs.init(d->vlrs, d->opts.srsConsumePreference, log());

    if (d->opts.start > d->header.pointCount())
        throwError("'start' value of " + std::to_string(d->opts.start) + " is too large. "
            "File contains " + std::to_string(d->header.pointCount()) + " points.");

    if (d->header.versionAtLeast(1, 4) || d->opts.useEbVlr)
        readExtraBytesVlr();

    setSrs(m);
    MetadataNode forward = table.privateMetadata("lasforward");
    las::extractHeaderMetadata(d->header, forward, m);
    las::extractSrsMetadata(d->srs, m);
    for (int i = 0; i < (int)d->vlrs.size(); ++i)
        las::addVlrMetadata(d->vlrs[i], "vlr_" + std::to_string(i), forward, m);

}


void LasReader::ready(PointTableRef table)
{
    createStream();
    std::istream *stream(m_streamIf->m_istream);

    d->index = 0;
    if (d->header.dataCompressed())
    {
        delete d->decompressor;

        const las::Vlr *vlr = las::findVlr(las::LaszipUserId, las::LaszipRecordId, d->vlrs);
        if (!vlr)
            throwError("LAZ file missing required laszip VLR.");
        d->decompressor = new LazPerfVlrDecompressor(*stream, d->header, vlr->data());
        if (d->opts.start > 0)
        {
            if (d->opts.start > d->header.pointCount())
                throwError("'start' option set past end of file.");
            d->decompressor->seek(d->opts.start);
        }
        d->decompressorBuf.resize(d->header.pointSize);
    }
    else
    {
        std::istream::pos_type start = d->header.pointOffset +
            (d->opts.start * d->header.pointSize);
        stream->seekg(start);
    }
}


void LasReader::readExtraBytesVlr()
{
    const las::Vlr *vlr = las::findVlr(las::SpecUserId, las::ExtraBytesRecordId, d->vlrs);
    if (!vlr)
        return;

    if (vlr->dataSize() % las::ExtraBytesSpecSize != 0)
    {
        log()->get(LogLevel::Warning) << "Bad size for extra bytes VLR.  Ignoring.";
        return;
    }

    std::vector<las::ExtraDim> extraDims =
        las::ExtraBytesIf::toExtraDims(vlr->data(), vlr->dataSize(), d->header.baseCount());

    if (d->extraDims.size() && d->extraDims != extraDims)
        log()->get(LogLevel::Warning) << "Extra byte dimensions specified "
            "in pipeline and VLR don't match.  Ignoring pipeline-specified "
            "dimensions";
    d->extraDims = std::move(extraDims);
}


//ABELL - Not sure why this is its own function, but leaving it so as not to break
//  API.
void LasReader::setSrs(MetadataNode& m)
{
    setSpatialReference(m, d->srs.get());
}


void LasReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims(las::pdrfDims(d->header.pointFormat()));

    size_t ebLen = d->header.ebCount();
    for (auto& dim : d->extraDims)
    {
        if (dim.m_size > ebLen)
            throwError("Extra byte specification exceeds point length beyond base format length.");
        ebLen -= dim.m_size;

        Dimension::Type type = dim.m_dimType.m_type;
        if (type == Dimension::Type::None)
            continue;
        if (dim.m_dimType.m_xform.nonstandard())
            type = Dimension::Type::Double;
        if (d->opts.fixNames)
            dim.m_name = Dimension::fixName(dim.m_name);
        dim.m_dimType.m_id = layout->registerOrAssignDim(dim.m_name, type);
    }
}


bool LasReader::processOne(PointRef& point)
{
    if (d->index >= getNumPoints())
        return false;

    if (d->header.dataCompressed())
    {
        if (!d->decompressor->decompress(d->decompressorBuf.data()))
            throwError("Error reading point " + std::to_string(d->index) +
                " from " + m_filename + ". Invalid/corrupt file.");
        loadPoint(point, d->decompressorBuf.data(), d->header.pointSize);
    }
    else
    {
        std::vector<char> buf(d->header.pointSize);
        m_streamIf->m_istream->read(buf.data(), buf.size());
        loadPoint(point, buf.data(), buf.size());
    }
    d->index++;
    return true;
}


point_count_t LasReader::read(PointViewPtr view, point_count_t count)
{
    count = (std::min)(count, getNumPoints() - d->index);

    PointId i = 0;
    if (d->header.dataCompressed())
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
    else
    {
        point_count_t remaining = count;

        // Make a buffer at most a meg.
        size_t bufsize = (std::min)((point_count_t)1000000, count * d->header.pointSize);
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
                    loadPoint(point, pos, d->header.pointSize);
                    if (m_cb)
                        m_cb(*view, id);
                    pos += d->header.pointSize;
                    i++;
                }
            } while (remaining);
        }
        catch (std::out_of_range&)
        {}
        catch (invalid_stream&)
        {}
    }
    d->index += i;
    return (point_count_t)i;
}


point_count_t LasReader::readFileBlock(std::vector<char>& buf, point_count_t maxpoints)
{
    std::istream *stream(m_streamIf->m_istream);

    size_t ptLen = d->header.pointSize;
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


void LasReader::loadPoint(PointRef& point, char *buf, size_t bufsize)
{
    if (d->header.has14PointFormat())
        loadPointV14(point, buf, bufsize);
    else
        loadPointV10(point, buf, bufsize);
}


void LasReader::loadPointV10(PointRef& point, char *buf, size_t bufsize)
{
    LeExtractor istream(buf, bufsize);

    int32_t xi, yi, zi;
    istream >> xi >> yi >> zi;
    const las::Header& h = d->header;

    double x = xi * h.scale.x + h.offset.x;
    double y = yi * h.scale.y + h.offset.y;
    double z = zi * h.scale.z + h.offset.z;

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

    if (d->extraDims.size())
        loadExtraDims(istream, point);
}


void LasReader::loadPointV14(PointRef& point, char *buf, size_t bufsize)
{
    LeExtractor istream(buf, bufsize);

    int32_t xi, yi, zi;
    istream >> xi >> yi >> zi;

    const las::Header& h = d->header;

    double x = xi * h.scale.x + h.offset.x;
    double y = yi * h.scale.y + h.offset.y;
    double z = zi * h.scale.z + h.offset.z;

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

    if (d->extraDims.size())
        loadExtraDims(istream, point);
}


void LasReader::loadExtraDims(LeExtractor& istream, PointRef& point)
{
    for (auto& dim : d->extraDims)
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
    m_streamIf.reset();
}

bool LasReader::eof()
{
    return d->index >= getNumPoints();
}


} // namespace pdal
