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

#include <zlib.h>

#include <vector>
#include <algorithm>

#include <pdal/drivers/bpf/BpfHeader.hpp>
#include <pdal/drivers/bpf/BpfReader.hpp>
#include <pdal/drivers/bpf/BpfSeqIterator.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Schema.hpp>

namespace pdal
{

BpfSeqIterator::BpfSeqIterator(PointBuffer& data, boost::uint32_t numPoints,
        BpfFormat::Enum pointFormat, bool compression, ILeStream& stream) :
    ReaderSequentialIterator(data), m_numPoints(numPoints),
    m_pointFormat(pointFormat), m_compression(compression), m_stream(stream), 
    m_index(0)
{
    m_start = m_stream.position();

}

boost::uint32_t BpfSeqIterator::readBufferImpl(PointBuffer& data)
{
    std::vector<Dimension> dims;

    m_dims.clear();
    const Schema& schema = data.getSchema();
    for (size_t i = 0; i < schema.numDimensions(); ++i)
    {
        Dimension d = schema.getDimension(i);
        if (d.getNamespace() == "bpf")
            m_dims.push_back(d);
    }

    uint32_t numRead = 0;
    if (m_compression)
        numRead = readCompressed(data);
    else
        numRead = read(data);
    return numRead;
}

uint32_t BpfSeqIterator::readCompressed(PointBuffer& data)
{
    std::vector<char> outBuf(m_numPoints * m_dims.size() * sizeof(float));
    size_t index = 0;
    size_t bytesRead = 0;
    do
    {
        bytesRead = readBlock(outBuf, index);
        index += bytesRead;
    } while (bytesRead > 0 && index < outBuf.size());

    class charbuf : public std::streambuf
    {
    public:
        charbuf(std::vector<char>& v)
            { setg(v.data(), v.data(), v.data() + v.size()); }
    };

    charbuf buf(outBuf);
    std::unique_ptr<std::istream> stream(new std::istream(&buf));

    m_stream.pushStream(stream.get());
    uint32_t numRead = read(data);
    m_stream.popStream();
    return numRead;
}

size_t BpfSeqIterator::readBlock(std::vector<char>& outBuf, size_t index)
{
    boost::uint32_t finalBytes;
    boost::uint32_t compressBytes;

    m_stream >> finalBytes;
    m_stream >> compressBytes;

    std::vector<char> in(compressBytes);

    // Fill the input bytes from the stream.
    m_stream.get(in);
    int ret = inflate(in.data(), compressBytes,
        outBuf.data() + index, finalBytes);
    return (ret ? 0 : finalBytes);
}
    
boost::uint32_t BpfSeqIterator::read(PointBuffer& data)
{
    switch (m_pointFormat)
    {
    case BpfFormat::PointMajor:
        return readPointMajor(data);
    case BpfFormat::DimMajor:
        return readDimMajor(data);
    case BpfFormat::ByteMajor:
        return readByteMajor(data);
    default:
        break;
    }
    return 0;
}

boost::uint64_t BpfSeqIterator::skipImpl(boost::uint64_t pointsToSkip)
{
    boost::uint32_t lastIndex = m_index;
    m_index += (boost::uint32_t)pointsToSkip;
    m_index = std::min(m_index, m_numPoints);
    return std::min((uint32_t)pointsToSkip, m_index - lastIndex);
}

bool BpfSeqIterator::atEndImpl() const
{
    return m_index >= m_numPoints;
}

boost::uint32_t BpfSeqIterator::readPointMajor(PointBuffer& data)
{
    uint32_t capacity = data.getCapacity();
    uint32_t idx = m_index;
    uint32_t numRead = 0;
    seekPointMajor(idx);
    while (numRead < capacity && idx < m_numPoints)
    {
        for (size_t d = 0; d < m_dims.size(); ++d)
        {
            float f;

            m_stream >> f;
            //NOTE - Each time this function is called, we start writing at
            //  point position 0 in the buffer.
            data.setField<float>(m_dims[d], numRead, f);
        }
        idx++;
        numRead++;
    }
    m_index = idx;
    data.setNumPoints(m_index);
    return numRead;
}

boost::uint32_t BpfSeqIterator::readDimMajor(PointBuffer& data)
{
    uint32_t capacity = data.getCapacity();
    uint32_t idx;
    uint32_t numRead = 0;
    for (size_t d = 0; d < m_dims.size(); ++d)
    {
        idx = m_index;
        numRead = 0;
        seekDimMajor(d, idx);
        for (; numRead < capacity && idx < m_numPoints; idx++, numRead++)
        {
            float f;

            m_stream >> f;
            //NOTE - Each time this function is called, we start writing at
            //  point position 0 in the buffer.
            data.setField<float>(m_dims[d], numRead, f);
        }
    }
    m_index = idx;
    data.setNumPoints(numRead);
    return numRead;
}

boost::uint32_t BpfSeqIterator::readByteMajor(PointBuffer& data)
{
    uint32_t idx;
    uint32_t numRead = 0;
    uint32_t capacity = data.getCapacity();

    for (size_t d = 0; d < m_dims.size(); ++d)
    {
        for (size_t b = 0; b < sizeof(float); ++b)
        {
            idx = m_index;
            numRead = 0;
            seekByteMajor(d, b, idx);
            for (;numRead < capacity && idx < m_numPoints; idx++, numRead++)
            {
                union 
                {
                    float f;
                    uint32_t u32;
                } u;

                u.u32 = 0;

                //NOTE - Each time this function is called, we start writing at
                //  point position 0 in the buffer.
                if (b)
                {
                    u.f = data.getField<float>(m_dims[d], numRead);
                }
                uint8_t u8;
                m_stream >> u8;
                u.u32 |= ((uint32_t)u8 << (b * CHAR_BIT));
                data.setField<float>(m_dims[d], numRead, u.f);
            }
        }
    }
    m_index = idx;
    data.setNumPoints(numRead);
    return numRead;
}

void BpfSeqIterator::seekPointMajor(uint32_t ptIdx)
{
    std::streamoff offset = ptIdx * sizeof(float) * m_dims.size();
    m_stream.seek(m_start + offset);
}

void BpfSeqIterator::seekDimMajor(size_t dimIdx, uint32_t ptIdx)
{
    std::streamoff offset = ((sizeof(float) * dimIdx * m_numPoints) +
        (sizeof(float) * ptIdx));
    m_stream.seek(m_start + offset);
}

void BpfSeqIterator::seekByteMajor(size_t dimIdx, size_t byteIdx, uint32_t ptIdx)
{
    std::streamoff offset =
        (dimIdx * m_numPoints * sizeof(float)) +
        (byteIdx * m_numPoints) +
        ptIdx;
    m_stream.seek(m_start + offset);
}

int BpfSeqIterator::inflate(char *buf, size_t insize, char *outbuf,
    size_t outsize)
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

} //namespace
