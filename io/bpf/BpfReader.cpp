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

#include <zlib.h>

#include <pdal/Options.hpp>
#include <pdal/pdal_export.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.bpf",
    "\"Binary Point Format\" (BPF) reader support. BPF is a simple \n" \
        "DoD and research format that is used by some sensor and \n" \
        "processing chains.",
    "http://pdal.io/stages/readers.bpf.html" );

CREATE_STATIC_PLUGIN(1, 0, BpfReader, Reader, s_info)

std::string BpfReader::getName() const { return s_info.name; }

void BpfReader::processOptions(const Options&)
{
    if (m_filename.empty())
        throw pdal_error("Can't read BPF file without filename.");

    // Logfile doesn't get set until options are processed.
    m_header.setLog(log());
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
    m_stream.open(m_filename);

    // Resets the stream position in case it was already open.
    m_stream.seek(0);
    // In order to know the dimensions we must read the file header.
    if (!m_header.read(m_stream))
        return;

    if (!m_header.readDimensions(m_stream, m_dims))
        return;

    uint32_t zone(abs(m_header.m_coordId));
    std::string code("");
    if (m_header.m_coordId > 0)
        code = "EPSG:326" + boost::lexical_cast<std::string>(zone);
    else
        code = "EPSG:327" + boost::lexical_cast<std::string>(zone);
    SpatialReference srs(code);
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
        throw pdal_error("BPF Header length exceeded that reported by file.");
    else if (pos < m_header.m_len)
        m_stream.seek(m_header.m_len);
}


void BpfReader::addDimensions(PointLayoutPtr layout)
{
    for (size_t i = 0; i < m_dims.size(); ++i)
    {
        Dimension::Type::Enum type = Dimension::Type::Float;

        BpfDimension& dim = m_dims[i];
        if (dim.m_label == "X" ||
            dim.m_label == "Y" ||
            dim.m_label == "Z")
            type = Dimension::Type::Double;
        dim.m_id = layout->registerOrAssignDim(dim.m_label, type);
    }
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
        m_metadata.addEncoded(file.m_filename,
            (const unsigned char *)file.m_buf.data(), file.m_len);
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
    m_index = 0;
    m_start = m_stream.position();
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
}


void BpfReader::done(PointTableRef)
{
     delete m_stream.popStream();
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
    default:
        break;
    }
    return 0;
}


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


bool BpfReader::eof()
{
    return m_index >= numPoints();
}


point_count_t BpfReader::readPointMajor(PointViewPtr data, point_count_t count)
{
    PointId nextId = data->size();
    PointId idx = m_index;
    point_count_t numRead = 0;
    seekPointMajor(idx);
    while (numRead < count && idx < numPoints())
    {
        for (size_t d = 0; d < m_dims.size(); ++d)
        {
            float f;

            m_stream >> f;
            data->setField(m_dims[d].m_id, nextId, f + m_dims[d].m_offset);
        }

        // Transformation only applies to X, Y and Z
        double x = data->getFieldAs<double>(Dimension::Id::X, nextId);
        double y = data->getFieldAs<double>(Dimension::Id::Y, nextId);
        double z = data->getFieldAs<double>(Dimension::Id::Z, nextId);
        m_header.m_xform.apply(x, y, z);
        data->setField(Dimension::Id::X, nextId, x);
        data->setField(Dimension::Id::Y, nextId, y);
        data->setField(Dimension::Id::Z, nextId, z);

        if (m_cb)
            m_cb(*data, nextId);

        idx++;
        numRead++;
        nextId++;
    }
    m_index = idx;
    return numRead;
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

point_count_t BpfReader::readByteMajor(PointViewPtr data, point_count_t count)
{
    PointId idx(0);
    PointId startId = data->size();
    point_count_t numRead = 0;

    // We need a temp buffer for the point data->
    union uu
    {
        float f;
        uint32_t u32;
    };
    std::unique_ptr<union uu> uArr(
        new uu[std::min(count, numPoints() - m_index)]);

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
                    u.f += m_dims[d].m_offset;
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


int BpfReader::inflate(char *buf, size_t insize, char *outbuf, size_t outsize)
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

} //namespace pdal
