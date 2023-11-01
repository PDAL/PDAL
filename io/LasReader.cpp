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

#include "LasHeader.hpp"
#include "LasReader.hpp"
#include "private/las/ChunkInfo.hpp"
#include "private/las/Header.hpp"
#include "private/las/Srs.hpp"
#include "private/las/Tile.hpp"
#include "private/las/Utils.hpp"
#include "private/las/Vlr.hpp"

#include <condition_variable>
#include <mutex>
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
#include <lazperf/readers.hpp>

namespace pdal
{

namespace
{

constexpr int DefaultNumThreads = 7;

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
    int numThreads;
    std::string srsConsumePreference;
};

struct LasReader::Private
{
    Options opts;
    las::Header header;
    LasHeader apiHeader;
    uint64_t index;
    las::VlrList ignoreVlrs;
    las::VlrList vlrs;
    las::Srs srs;
    las::TilePtr currentTile;
    las::ChunkInfo chunkInfo;
    std::vector<las::TilePtr> tiles;
    std::vector<las::ExtraDim> extraDims;
    ThreadPool pool;
    // One past the Index of the last point we want to fetch.
    PointId end;
    // The index of the chunk we want to fetch next.
    uint32_t nextFetchChunk;
    // The point ID of the point we want to fetch next.
    uint64_t nextFetchPoint;
    // The index of the chunk (tile) we want to read data from.
    uint32_t nextReadChunk;
    std::mutex mutex;
    std::condition_variable processedCv;

    Private() : apiHeader(header, srs, vlrs), index(0), pool(DefaultNumThreads)
    {}
};

LasReader::LasReader() : d(new Private)
{}

LasReader::~LasReader()
{}

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
    args.add("threads", "Thread pool size", d->opts.numThreads, DefaultNumThreads);
    args.add("srs_consume_preference", "Preference order to read SRS VLRs",
        d->opts.srsConsumePreference, "wkt1, geotiff, wkt2, projjson");
}


static StaticPluginInfo const s_info {
    "readers.las",
    "ASPRS LAS 1.0 - 1.4 read support",
    "http://pdal.io/stages/readers.las.html",
    { "las", "laz" }
};

CREATE_STATIC_STAGE(LasReader, s_info)

std::string LasReader::getName() const { return s_info.name; }

const LasHeader& LasReader::header() const
{
    return d->apiHeader;
}

const las::Header& LasReader::lasHeader() const
{
    return d->header;
}

uint64_t LasReader::vlrData(const std::string& userId, uint16_t recordId, char const * & data)
{
    const las::Vlr *vlr = las::findVlr(userId, recordId, d->vlrs);
    if (!vlr)
	return 0;
    data = vlr->data();
    return vlr->dataVec.size();
}

// Number of points we're wanting to fetch.
point_count_t LasReader::getNumPoints() const
{
    return d->end - d->opts.start;
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


// IMPORTANT NOTE: Unless you're going to totally overhaul things, you *must* use this
//  virtual function to create a stream that you *must* use. NITF files contain embedded
//  LAS files and this allows native and embedded files to be accessed identically. All
//  the file positioning works assuming that the file is pure LAS/LAZ thanks to this.
LasReader::LasStreamPtr LasReader::createStream()
{
    LasStreamPtr s(new LasStreamIf(m_filename));
    if (!s->isOpen())
    {
        std::ostringstream oss;
        oss << "Unable to open stream for '"
            << m_filename << "' with error '" << strerror(errno) << "'";
        throw pdal_error(oss.str());
    }
    return s;
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

    // This will throw if the stream can't be opened.
    LasStreamPtr lasStream = createStream();
    std::istream& stream(*lasStream);

    stream.seekg(0);
    // Always try to read as if we have 1.4 size.
    char headerBuf[las::Header::Size14];
    stream.read(headerBuf, las::Header::Size14);
    if (stream.gcount() < (std::streamsize)las::Header::Size12)
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
    // If we over-read the file, the error state will be set, but things are really fine for
    // a zero-point file, so clear the error.
    stream.clear();
    stream.seekg(377);
    char copcBuf[4] {};
    stream.read(copcBuf, 4);
    m.add("copc", ::memcmp(copcBuf, "copc", 4) == 0);

    // Read VLRs.
    // Clear the error state since the seek or read above may have failed but the file could
    // still be fine.
    stream.clear();
    stream.seekg(d->header.headerSize);

    char vlrHeaderBuf[las::Vlr::HeaderSize];
    std::vector<char> vlrBuf;
    for (uint32_t i = 0; i < d->header.vlrCount; ++i)
    {
        las::Vlr vlr;

        stream.read((char *)vlrHeaderBuf, las::Vlr::HeaderSize);
        if (stream.gcount() != las::Vlr::HeaderSize)
            throwError("Couldn't read VLR " + std::to_string(i + 1) + ". End of file reached.");
        vlr.fillHeader(vlrHeaderBuf);
        if ((uint64_t)stream.tellg() + vlr.promisedDataSize > d->header.pointOffset)
            throwError("VLR " + std::to_string(i + 1) +
                "(" + vlr.userId + "/" + std::to_string(vlr.recordId) + ") "
                "size too large -- flows into point data.");
        if (las::shouldIgnoreVlr(vlr, d->ignoreVlrs))
        {
            stream.seekg(vlr.promisedDataSize, std::ios::cur);
            continue;
        }
        vlr.dataVec.resize(vlr.promisedDataSize);
        stream.read(vlr.data(), vlr.promisedDataSize);

        if (stream.gcount() != (std::streamsize)vlr.promisedDataSize)
            throwError("Couldn't read VLR " + std::to_string(i + 1) + ". End of file reached.");
        d->vlrs.push_back(std::move(vlr));
    }

    // Read EVLRs if we have them.
    if (d->header.evlrOffset && d->header.evlrCount)
    {
        char evlrHeaderBuf[las::Evlr::HeaderSize];
        stream.seekg(d->header.evlrOffset);
        for (uint32_t i = 0; i < d->header.evlrCount; ++i)
        {
            las::Evlr evlr;

            stream.read((char *)evlrHeaderBuf, las::Evlr::HeaderSize);
            if (stream.gcount() != las::Evlr::HeaderSize)
                throwError("Couldn't read EVLR " + std::to_string(i + 1) +
                    ". End of file reached.");
            evlr.fillHeader(evlrHeaderBuf);

            if ((uint64_t)stream.tellg() + evlr.promisedDataSize > fileSize)
                throwError("EVLR " + std::to_string(i + 1) +
                    "(" + evlr.userId + "/" + std::to_string(evlr.recordId) + ") "
                    "size too large -- exceeds file size.");
            if (las::shouldIgnoreVlr(evlr, d->ignoreVlrs))
            {
                stream.seekg(evlr.promisedDataSize, std::ios::cur);
                continue;
            }
            evlr.dataVec.resize(evlr.promisedDataSize);
            std::streampos pos = stream.tellg();
            stream.read(evlr.data(), evlr.promisedDataSize);

            if (stream.gcount() != (std::streamsize)evlr.promisedDataSize)
                throwError("Couldn't read EVLR " + std::to_string(i + 1) + " at offset " +
                    std::to_string(pos) + ". Location is past the end of the file.");
            d->vlrs.push_back(std::move(evlr));
        }
    }

    if (!d->opts.nosrs)
        d->srs.init(d->vlrs, d->opts.srsConsumePreference, log());

    d->end = d->header.pointCount();
    if (d->header.pointCount())
    {
        if (d->opts.start >= d->header.pointCount())
            throwError("'start' value of " + std::to_string(d->opts.start) + " is too large. "
                "File contains " + std::to_string(d->header.pointCount()) + " points.");

        // maxPoints is positive because start is less than count from above.
        uint64_t maxPoints = d->header.pointCount() - d->opts.start;

        // count() can be a crazy-high value -- don't overflow with the addition.
        if (count() < maxPoints)
            d->end = d->opts.start + count();
    }

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
    d->pool.resize(d->opts.numThreads);
    LasStreamPtr lasStream(createStream());
    std::istream& stream(*lasStream);

    d->currentTile.reset();
    d->tiles.clear();

    d->index = 0;
    if (d->header.dataCompressed())
    {
        const las::Vlr *vlr = las::findVlr(las::LaszipUserId, las::LaszipRecordId, d->vlrs);
        if (!vlr)
            throwError("LAZ file missing required laszip VLR.");
        lazperf::laz_vlr laz_vlr;
        laz_vlr.fill(vlr->data(), vlr->dataSize());
        try
        {
            d->chunkInfo.load(stream, d->header.pointOffset, d->header.pointCount(),
                laz_vlr.chunk_size);
        }
        catch (const pdal_error& e)
        {
            throwError(e.what());
        }

        d->nextFetchChunk = 0;
        d->nextFetchPoint = 0;
        if (d->opts.start > 0)
        {
            if (d->opts.start >= d->header.pointCount())
                throwError("'start' option set past end of file.");
            d->nextFetchChunk = d->chunkInfo.chunk(d->opts.start);
            d->nextFetchPoint = d->chunkInfo.index(d->opts.start, d->nextFetchChunk);
        }
        d->nextReadChunk = d->nextFetchChunk;
    }
    else
    {
        d->nextFetchPoint = d->opts.start;
        d->nextFetchChunk = 0;
        d->nextReadChunk = 0;
    }

    for (int i = 0; i < d->opts.numThreads; ++i)
        queueNext();
}

// Use a function instead of if statement.
void LasReader::queueNext()
{
    if (d->header.dataCompressed())
        queueNextCompressedChunk();
    else
        queueNextStandardChunk();
}

void LasReader::queueNextCompressedChunk()
{
    if ((d->nextFetchChunk >= d->chunkInfo.numChunks()) ||
        (d->chunkInfo.firstPoint(d->nextFetchChunk) >= d->end))
        return;

    uint32_t chunk = d->nextFetchChunk;
    uint32_t start = d->nextFetchPoint;

    d->pool.add([this, chunk, start]()
    {
        uint32_t chunkpoints = d->chunkInfo.chunkPoints(chunk);
        uint64_t chunkoffset = d->chunkInfo.chunkOffset(chunk);
        uint32_t chunksize = d->chunkInfo.chunkSize(chunk);

        LasStreamPtr lasStream = createStream();
        std::istream& in(*lasStream);

        std::vector<char> buf(chunksize);
        in.seekg(chunkoffset);
        in.read(buf.data(), buf.size());

        int32_t tilepoints = chunkpoints - start;
        las::TilePtr tile = std::make_unique<las::Tile>(chunk, tilepoints * d->header.pointSize);

        lazperf::reader::chunk_decompressor decomp(d->header.pointFormat(), d->header.ebCount(),
            buf.data());

        // We have to decompress all the points, even if we're discarding the points at
        // the front because nextFetchPoint isn't 0. Just reuse the front of the tile
        // buffer for discarded points.
        char *pos = tile->data();
        for (uint32_t i = 0; i < chunkpoints; ++i)
        {
            decomp.decompress(pos);

            // Advance the point location in the tile if we're keeping the point.
            if (i >= start)
                pos += d->header.pointSize;
        }
        {
            std::unique_lock l(d->mutex);
            for (las::TilePtr& t : d->tiles)
                if (!t)
                {
                    t = std::move(tile);
                    goto done;
                }
            d->tiles.push_back(std::move(tile));
        }
        done:
        d->processedCv.notify_one();
    });
    d->nextFetchChunk++;
    // After the first chunk, we always start at 0.
    d->nextFetchPoint = 0;
}

void LasReader::queueNextStandardChunk()
{
    const uint64_t chunkSize = 50'000;

    if (d->nextFetchPoint >= d->end)
        return;

    int chunk = d->nextFetchChunk;
    uint64_t start = d->nextFetchPoint;
    uint64_t count = (std::min)(chunkSize, d->end - start);
    d->pool.add([this, chunk, count, start]()
    {
        LasStreamPtr lasStream = createStream();
        std::istream& in(*lasStream);

        las::TilePtr tile = std::make_unique<las::Tile>(chunk, count * d->header.pointSize);
        in.seekg(d->header.pointOffset + start * d->header.pointSize);
        in.read(tile->data(), tile->size());

        {
            std::unique_lock l(d->mutex);
            for (las::TilePtr& t : d->tiles)
                if (!t)
                {
                    t = std::move(tile);
                    goto done;
                }
            d->tiles.push_back(std::move(tile));
        }
        done:
        d->processedCv.notify_one();
    });

    // This check is just to prevent overflow.
    if (d->nextFetchPoint > (std::numeric_limits<uint64_t>::max)() - chunkSize)
        d->nextFetchPoint = d->end;
    else
        d->nextFetchPoint += chunkSize;
    d->nextFetchChunk++;
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
    // This is called under lock. Note that we don't remove the tile *pointer* from the
    // vector, it just gets set to null. When we add a tile, we'll look for a null
    // entry before we add to the vector.
    auto getTile = [this](uint32_t chunk)
    {
        for (las::TilePtr& t : d->tiles)
            if (t && t->chunk() == chunk)
                return std::move(t);
        return las::TilePtr();
    };

    if (eof())
        return false;

    if (!d->currentTile)
    {
        {
            std::unique_lock<std::mutex> l(d->mutex);
            while (true)
            {
                d->currentTile = getTile(d->nextReadChunk);
                if (d->currentTile)
                    break;
                d->processedCv.wait(l);
            }
        }

        // Found the tile we wanted.
        d->nextReadChunk++;
        queueNext();
    }
    loadPoint(point);
    d->index++;
    return true;
}

point_count_t LasReader::read(PointViewPtr view, point_count_t count)
{
    count = (std::min)(count, getNumPoints() - (point_count_t)d->index);

    PointId i = 0;
    for (i = 0; i < count; i++)
    {
        PointRef point = view->point(i);
        PointId id = view->size();
        processOne(point);
        if (m_cb)
            m_cb(*view, id);
    }
    return (point_count_t)i;
}


void LasReader::loadPoint(PointRef& point)
{
    if (d->header.has14PointFormat())
        loadPointV14(point, d->currentTile->pos(), d->header.pointSize);
    else
        loadPointV10(point, d->currentTile->pos(), d->header.pointSize);
    if (!d->currentTile->advance(d->header.pointSize))
        d->currentTile.reset();
}


void LasReader::loadPointV10(PointRef& point, const char *buf, size_t bufsize)
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
    uint8_t classificationWithFlags;
    int8_t scanAngleRank;
    uint8_t user;
    uint16_t pointSourceId;

    istream >> intensity >> flags >> classificationWithFlags >> scanAngleRank >>
        user >> pointSourceId;

    uint8_t returnNum = flags & 0x07;
    uint8_t numReturns = (flags >> 3) & 0x07;
    uint8_t scanDirFlag = (flags >> 6) & 0x01;
    uint8_t flight = (flags >> 7) & 0x01;

    uint8_t classification = classificationWithFlags & 0x1F;
    uint8_t synthetic = (classificationWithFlags >> 5) & 0x01;
    uint8_t keypoint = (classificationWithFlags >> 6) & 0x01;
    uint8_t withheld = (classificationWithFlags >> 7) & 0x01;
    uint8_t overlap = 0;

    // For V10 PDRFs, "Overlap" was encoded as Classification=12.  This was
    // split out into its own bitfield for the V14 PDRFs, so mimic that behavior
    // here, setting the dedicated Overlap flag and resetting the Classification
    // to "Never Classified".
    if (classification == ClassLabel::LegacyOverlap)
    {
        classification = ClassLabel::CreatedNeverClassified;
        overlap = 1;
    }

    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
    point.setField(Dimension::Id::Intensity, intensity);
    point.setField(Dimension::Id::ReturnNumber, returnNum);
    point.setField(Dimension::Id::NumberOfReturns, numReturns);
    point.setField(Dimension::Id::ScanDirectionFlag, scanDirFlag);
    point.setField(Dimension::Id::EdgeOfFlightLine, flight);
    point.setField(Dimension::Id::Classification, classification);
    point.setField(Dimension::Id::Synthetic, synthetic);
    point.setField(Dimension::Id::KeyPoint, keypoint);
    point.setField(Dimension::Id::Withheld, withheld);
    point.setField(Dimension::Id::Overlap, overlap);
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


void LasReader::loadPointV14(PointRef& point, const char *buf, size_t bufsize)
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
    uint8_t synthetic = (flags >> 0) & 0x01;
    uint8_t keypoint = (flags >> 1) & 0x01;
    uint8_t withheld = (flags >> 2) & 0x01;
    uint8_t overlap = (flags >> 3) & 0x01;
    uint8_t scanChannel = (flags >> 4) & 0x03;
    uint8_t scanDirFlag = (flags >> 6) & 0x01;
    uint8_t flight = (flags >> 7) & 0x01;

    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
    point.setField(Dimension::Id::Intensity, intensity);
    point.setField(Dimension::Id::ReturnNumber, returnNum);
    point.setField(Dimension::Id::NumberOfReturns, numReturns);
    point.setField(Dimension::Id::Synthetic, synthetic);
    point.setField(Dimension::Id::KeyPoint, keypoint);
    point.setField(Dimension::Id::Withheld, withheld);
    point.setField(Dimension::Id::Overlap, overlap);
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
    d->pool.join();
}

bool LasReader::eof()
{
    // This breaks when the number of points is the maximum (2^64 - 1), but that's never happening.
    return d->index >= d->end;
}


} // namespace pdal
