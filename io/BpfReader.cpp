/******************************************************************************
* Copyright (c) 2014, Andrew Bell
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

#include "BpfReader.hpp"

#include <climits>

#include <pdal/Options.hpp>
#include <pdal/pdal_features.hpp>

#ifdef PDAL_HAVE_ZLIB
#include <zlib.h>
#endif

namespace pdal
{

static StaticPluginInfo const s_info
{
    "readers.bpf",
    "\"Binary Point Format\" (BPF) reader support. BPF is a simple \n" \
        "DoD and research format that is used by some sensor and \n" \
        "processing chains.",
    "http://pdal.io/stages/readers.bpf.html",
    { "bpf" }
};

CREATE_STATIC_STAGE(BpfReader, s_info)

struct BpfReader::Args
{
    bool m_fixNames;
};

std::string BpfReader::getName() const { return s_info.name; }

BpfReader::BpfReader() : m_args(new BpfReader::Args)
{}


BpfReader::~BpfReader()
{
#ifdef PDAL_HAVE_ZLIB
    if (m_header.m_compression)
    {
        for( auto& stream: m_streams )
        {
            delete stream->popStream();
        }
    }
#endif
}


void BpfReader::addArgs(ProgramArgs& args)
{
    args.add("fix_dims", "Make invalid dimension names valid by changing "
        "invalid characters to '_'", m_args->m_fixNames, true);
}


void BpfReader::addDimensions(PointLayoutPtr layout)
{
    for (size_t i = 0; i < m_dims.size(); ++i)
    {
        BpfDimension& dim = m_dims[i];
        dim.m_id = layout->registerOrAssignDim(dim.m_label, Dimension::Type::Float);
    }
}


QuickInfo BpfReader::inspect()
{
    QuickInfo qi;

    initialize();
    qi.m_valid = true;
    qi.m_pointCount = m_header.m_numPts;
    qi.m_srs = getSpatialReference();
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        BpfDimension& dim = *di;
        qi.m_dimNames.push_back(dim.m_label);
        if (dim.m_label == "X")
        {
            qi.m_bounds.minx = dim.m_min;
            qi.m_bounds.maxx = dim.m_max;
        }
        if (dim.m_label == "Y")
        {
            qi.m_bounds.miny = dim.m_min;
            qi.m_bounds.maxy = dim.m_max;
        }
        if (dim.m_label == "Z")
        {
            qi.m_bounds.minz = dim.m_min;
            qi.m_bounds.maxz = dim.m_max;
        }
    }
    return qi;
}


// When the stage is intialized, the schema needs to be populated with the
// dimensions in order to allow subsequent stages to be aware of or append to
// the dimensions in the PointView.
void BpfReader::initialize()
{
    if (m_filename.empty())
        throwError("Can't read BPF file without filename.");

    // Logfile doesn't get set until options are processed.
    m_header.setLog(log());

    m_istreamPtr = Utils::openFile(m_filename);
    if (!m_istreamPtr)
        throwError("Can't open file '" + m_filename + "'.");
    m_stream = ILeStream(m_istreamPtr);

    // Resets the stream position in case it was already open.
    m_stream.seek(0);
    // In order to know the dimensions we must read the file header.
    try
    {
        if (!m_header.read(m_stream))
            return;
        if (!m_header.readDimensions(m_stream, m_dims, m_args->m_fixNames))
            return;
    }
    catch (const BpfHeader::error& err)
    {
        throwError(err.what());
    }
#ifndef PDAL_HAVE_ZLIB
    if (m_header.m_compression)
        throwError("Can't read compressed BPF. PDAL wasn't built with "
            "Zlib support.");
#endif

    SpatialReference srs;
    if (m_header.m_coordType == static_cast<int>(BpfCoordType::Cartesian))
    {
       srs.set("EPSG:4326");
    }
    else if (m_header.m_coordType == static_cast<int>(BpfCoordType::UTM))
    {
       srs = SpatialReference::wgs84FromZone(m_header.m_coordId);
       if (!srs.valid())
          throwError("BPF file contains an invalid UTM zone " +
            Utils::toString(m_header.m_coordId));
    }
    else if (m_header.m_coordType == static_cast<int>(BpfCoordType::TCR))
    {
        // TCR is ECEF meters, or EPSG:4978
        // According to the 1.0 spec, the m_coordId must be 1 to be
        // valid.
        if (m_header.m_coordId == 1)
            srs.set("EPSG:4978");
        else
        {
            std::ostringstream oss;
            oss << "BPF has ECEF/TCR coordinate type defined, but the ID of '"
                <<  m_header.m_coordId << "' is invalid";
            throwError(oss.str());
        }
    }
    else
    {
       // BPF also supports something East North Up (BpfCoordType::ENU)
       // which we can figure out when we run into a file with these
       // coordinate systems.
        std::ostringstream oss;
        oss << "BPF file contains unsupported coordinate system with "
            << "coordinate type: '" << m_header.m_coordType
            << "' and coordinate id: '" << m_header.m_coordId << "'";
       throwError(oss.str());
    }

    setSpatialReference(srs);

    if (m_header.m_version >= 3)
    {
        readUlemData();
        if (!m_stream)
            return;
        readUlemFiles();
        if (!m_stream)
            return;
        readPolarData();
    }

    // Read thing after the standard header as metadata->
    readHeaderExtraData();

    // Fast forward file to end of header as reported by base header.
    std::streampos pos = m_stream.position();
    if (pos > m_header.m_len)
        throwError("BPF Header length exceeded that reported by file.");
    m_stream.close();
    Utils::closeFile(m_istreamPtr);
}


bool BpfReader::readUlemData()
{
    if (!m_ulemHeader.read(m_stream))
        return false;

    for (size_t i = 0; i < m_ulemHeader.m_numFrames; i++)
    {
        BpfUlemFrame frame;
        if (!frame.read(m_stream))
            return false;
        m_ulemFrames.push_back(frame);
    }
    return (bool)m_stream;
}


bool BpfReader::readUlemFiles()
{
    BpfUlemFile file;
    while (file.read(m_stream))
    {
        MetadataNode m = m_metadata.add("bundled_file");
        m.addEncoded(file.m_filename,
            (const unsigned char *)file.m_buf.data(), file.m_len);
    }
    return (bool)m_stream;
}


/// Encode all data that follows the headers as metadata->
/// \return  Whether the stream is still valid.
bool BpfReader::readHeaderExtraData()
{
    if (m_stream.position() < m_header.m_len)
    {
        std::streampos size = m_header.m_len - m_stream.position();
        std::vector<uint8_t> buf(size);
        m_stream.get(buf);
        m_metadata.addEncoded("header_data", buf.data(), buf.size());
    }
    return (bool)m_stream;
}


bool BpfReader::readPolarData()
{
    if (!m_polarHeader.read(m_stream))
        return false;
    for (size_t i = 0; i < m_polarHeader.m_numFrames; ++i)
    {
        BpfPolarFrame frame;
        if (!frame.read(m_stream))
            return false;
        m_polarFrames.push_back(frame);
    }
    return (bool)m_stream;
}


void BpfReader::ready(PointTableRef)
{
    m_istreamPtr = Utils::openFile(m_filename);
    m_stream = ILeStream(m_istreamPtr);
    m_stream.seek(m_header.m_len);
    m_index = 0;
    m_start = m_stream.position();
#ifdef PDAL_HAVE_ZLIB
    if (m_header.m_compression)
    {
        m_deflateBuf.resize(numPoints() * m_dims.size() * sizeof(float));
        size_t index = 0;
        size_t bytesRead = 0;
        do
        {
            bytesRead = readBlock(m_deflateBuf, index);
            index += bytesRead;
        } while (bytesRead > 0 && index < m_deflateBuf.size());
        m_charbuf.initialize(m_deflateBuf.data(), m_deflateBuf.size(), m_start);
        m_stream.pushStream(new std::istream(&m_charbuf));
    }
#endif // PDAL_HAVE_ZLIB
}


void BpfReader::done(PointTableRef)
{
    if (auto s = m_stream.popStream())
        delete s;
    m_stream.close();
    Utils::closeFile(m_istreamPtr);
}


bool BpfReader::processOne(PointRef& point)
{
    if (eof() || m_index >= m_count)
        return false;

    switch (m_header.m_pointFormat)
    {
    case BpfFormat::PointMajor:
        readPointMajor(point);
        break;
    case BpfFormat::DimMajor:
        readDimMajor(point);
        break;
    case BpfFormat::ByteMajor:
        readByteMajor(point);
        break;
    }
    return true;
}


point_count_t BpfReader::read(PointViewPtr data, point_count_t count)
{
    switch (m_header.m_pointFormat)
    {
    case BpfFormat::PointMajor:
        return readPointMajor(data, count);
    case BpfFormat::DimMajor:
        return readDimMajor(data, count);
    case BpfFormat::ByteMajor:
        return readByteMajor(data, count);
    }
    return 0;
}


bool BpfReader::eof()
{
    return m_index >= numPoints();
}


void BpfReader::readPointMajor(PointRef& point)
{
    double x(0), y(0), z(0);

    seekPointMajor(m_index);
    for (size_t dim = 0; dim < m_dims.size(); ++dim)
    {
        float f;

        m_stream >> f;
        double d = f + m_dims[dim].m_offset;
        if (m_dims[dim].m_id == Dimension::Id::X)
            x = d;
        else if (m_dims[dim].m_id == Dimension::Id::Y)
            y = d;
        else if (m_dims[dim].m_id == Dimension::Id::Z)
            z = d;
        else
            point.setField(m_dims[dim].m_id, d);
    }

    m_header.m_xform.apply(x, y, z);
    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
    m_index++;
}


point_count_t BpfReader::readPointMajor(PointViewPtr view, point_count_t count)
{
    PointId nextId = view->size();
    PointId idx = m_index;
    point_count_t numRead = 0;
    seekPointMajor(idx);
    while (numRead < count && idx < numPoints())
    {
        for (size_t d = 0; d < m_dims.size(); ++d)
        {
            float f;

            m_stream >> f;
            view->setField(m_dims[d].m_id, nextId, f + m_dims[d].m_offset);
        }

        // Transformation only applies to X, Y and Z
        double x = view->getFieldAs<double>(Dimension::Id::X, nextId);
        double y = view->getFieldAs<double>(Dimension::Id::Y, nextId);
        double z = view->getFieldAs<double>(Dimension::Id::Z, nextId);
        m_header.m_xform.apply(x, y, z);
        view->setField(Dimension::Id::X, nextId, x);
        view->setField(Dimension::Id::Y, nextId, y);
        view->setField(Dimension::Id::Z, nextId, z);
        if (m_cb)
            m_cb(*view, nextId);

        idx++;
        numRead++;
        nextId++;
    }
    m_index = idx;
    return numRead;
}


void BpfReader::readDimMajor(PointRef& point)
{
    if (m_streams.empty())
    {
        for (std::size_t dim(0); dim < m_dims.size(); ++dim)
        {
            std::streamoff offset = sizeof(float) * dim * numPoints();

            m_streams.emplace_back(new ILeStream());
            m_streams.back()->open(m_filename);

#ifdef PDAL_HAVE_ZLIB
            if (m_header.m_compression)
            {
                m_charbufs.emplace_back(new Charbuf());
                m_charbufs.back()->initialize(
                        m_deflateBuf.data(), m_deflateBuf.size(), m_start);

                m_streams.back()->pushStream(
                        new std::istream(m_charbufs.back().get()));
            }
#endif // PDAL_HAVE_ZLIB

            m_streams.back()->seek(m_start + offset);
        }
    }

    double x(0), y(0), z(0);
    float f(0);
    double d(0);

    for (size_t dim = 0; dim < m_dims.size(); ++dim)
    {
        *m_streams[dim] >> f;
        d = f + m_dims[dim].m_offset;
        if (m_dims[dim].m_id == Dimension::Id::X)
            x = d;
        else if (m_dims[dim].m_id == Dimension::Id::Y)
            y = d;
        else if (m_dims[dim].m_id == Dimension::Id::Z)
            z = d;
        else
            point.setField(m_dims[dim].m_id, d);
    }

    // Transformation only applies to X, Y and Z
    m_header.m_xform.apply(x, y, z);
    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
    m_index++;
}


point_count_t BpfReader::readDimMajor(PointViewPtr data, point_count_t count)
{
    PointId idx(0);
    PointId startId = data->size();
    point_count_t numRead = 0;
    for (size_t d = 0; d < m_dims.size(); ++d)
    {
        idx = m_index;
        PointId nextId = startId;
        numRead = 0;
        seekDimMajor(d, idx);
        for (; numRead < count && idx < numPoints(); idx++, numRead++, nextId++)
        {
            float f;

            m_stream >> f;
            data->setField(m_dims[d].m_id, nextId, f + m_dims[d].m_offset);
        }
    }
    m_index = idx;

    // Transformation only applies to X, Y and Z
    for (PointId idx = startId; idx < data->size(); idx++)
    {
        double x = data->getFieldAs<double>(Dimension::Id::X, idx);
        double y = data->getFieldAs<double>(Dimension::Id::Y, idx);
        double z = data->getFieldAs<double>(Dimension::Id::Z, idx);
        m_header.m_xform.apply(x, y, z);
        data->setField(Dimension::Id::X, idx, x);
        data->setField(Dimension::Id::Y, idx, y);
        data->setField(Dimension::Id::Z, idx, z);

        if (m_cb)
            m_cb(*data, idx);
    }

    return numRead;
}


void BpfReader::readByteMajor(PointRef& point)
{
    // We need a temp buffer for the point data
    union uu
    {
        float f;
        uint32_t u32;
    } u;
    double x(0), y(0), z(0);
    uint8_t u8;

    for (size_t dim = 0; dim < m_dims.size(); ++dim)
    {
        u.u32 = 0;
        for (size_t b = 0; b < sizeof(float); ++b)
        {
            seekByteMajor(dim, b, m_index);

            m_stream >> u8;
            u.u32 |= ((uint32_t)u8 << (b * CHAR_BIT));
        }
        double d = u.f + m_dims[dim].m_offset;
        if (m_dims[dim].m_id == Dimension::Id::X)
            x = d;
        else if (m_dims[dim].m_id == Dimension::Id::Y)
            y = d;
        else if (m_dims[dim].m_id == Dimension::Id::Z)
            z = d;
        else
            point.setField(m_dims[dim].m_id, d);
    }

    m_header.m_xform.apply(x, y, z);
    point.setField(Dimension::Id::X, x);
    point.setField(Dimension::Id::Y, y);
    point.setField(Dimension::Id::Z, z);
    m_index++;
}


point_count_t BpfReader::readByteMajor(PointViewPtr data, point_count_t count)
{
    PointId idx(0);
    PointId startId = data->size();
    point_count_t numRead = 0;

    // We need a temp buffer for the point data
    union uu
    {
        float f;
        uint32_t u32;
    };
    std::unique_ptr<union uu[]> uArr(
        new uu[(std::min)(count, numPoints() - m_index)]);

    for (size_t d = 0; d < m_dims.size(); ++d)
    {
        for (size_t b = 0; b < sizeof(float); ++b)
        {
            idx = m_index;
            numRead = 0;
            PointId nextId = startId;
            seekByteMajor(d, b, idx);

            for (;numRead < count && idx < numPoints();
                idx++, numRead++, nextId++)
            {
                union uu& u = *(uArr.get() + numRead);

                if (b == 0)
                    u.u32 = 0;
                uint8_t u8;
                m_stream >> u8;
                u.u32 |= ((uint32_t)u8 << (b * CHAR_BIT));
                if (b == 3)
                {
                    u.f += static_cast<float>(m_dims[d].m_offset);
                    data->setField(m_dims[d].m_id, nextId, u.f);
                }
            }
        }
    }
    m_index = idx;

    // Transformation only applies to X, Y and Z
    for (PointId idx = startId; idx < data->size(); idx++)
    {
        double x = data->getFieldAs<double>(Dimension::Id::X, idx);
        double y = data->getFieldAs<double>(Dimension::Id::Y, idx);
        double z = data->getFieldAs<double>(Dimension::Id::Z, idx);
        m_header.m_xform.apply(x, y, z);
        data->setField(Dimension::Id::X, idx, x);
        data->setField(Dimension::Id::Y, idx, y);
        data->setField(Dimension::Id::Z, idx, z);

        if (m_cb)
            m_cb(*data, idx);
    }

    return numRead;
}


void BpfReader::seekPointMajor(PointId ptIdx)
{
    std::streamoff offset = ptIdx * sizeof(float) * m_dims.size();
    m_stream.seek(m_start + offset);
}


void BpfReader::seekDimMajor(size_t dimIdx, PointId ptIdx)
{
    std::streamoff offset = ((sizeof(float) * dimIdx * numPoints()) +
        (sizeof(float) * ptIdx));
    m_stream.seek(m_start + offset);
}


void BpfReader::seekByteMajor(size_t dimIdx, size_t byteIdx, PointId ptIdx)
{
    std::streamoff offset =
        (dimIdx * numPoints() * sizeof(float)) +
        (byteIdx * numPoints()) +
        ptIdx;
    m_stream.seek(m_start + offset);
}


#ifdef PDAL_HAVE_ZLIB
size_t BpfReader::readBlock(std::vector<char>& outBuf, size_t index)
{
    uint32_t finalBytes;
    uint32_t compressBytes;

    m_stream >> finalBytes;
    m_stream >> compressBytes;

    std::vector<char> in(compressBytes);

    // Fill the input bytes from the stream.
    m_stream.get(in);
    int ret = inflate(in.data(), compressBytes,
        outBuf.data() + index, finalBytes);
    return (ret ? 0 : finalBytes);
}


int BpfReader::inflate(char *buf, uint32_t insize,
    char *outbuf, uint32_t outsize)
{
   if (insize == 0)
        return 0;

    int ret;
    z_stream strm;

    /* allocate inflate state */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
    if (inflateInit(&strm) != Z_OK)
        return -2;

    strm.avail_in = insize;
    strm.next_in = (unsigned char *)buf;
    strm.avail_out = outsize;
    strm.next_out = (unsigned char *)outbuf;

    ret = ::inflate(&strm, Z_NO_FLUSH);
    (void)inflateEnd(&strm);
    return ret == Z_STREAM_END ? 0 : -1;
}
#endif // PDAL_HAVE_ZLIB

} //namespace pdal
