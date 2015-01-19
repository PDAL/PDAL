/******************************************************************************
* Copyright (c) 2015, Hobu Inc., hobu@hobu.co
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

#include "BpfWriter.hpp"

#include <pdal/pdal_internal.hpp>

#include <zlib.h>

#include <pdal/Charbuf.hpp>
#include <pdal/Options.hpp>

#include "BpfCompressor.hpp"

namespace pdal
{

void BpfWriter::processOptions(const Options& options)
{
    if (m_filename.empty())
        throw pdal_error("Can't write BPF file without filename.");
    bool compression = options.getValueOrDefault("compression", false);
    m_header.m_compression = compression ? BpfCompression::Zlib :
        BpfCompression::None;

    std::string fileFormat =
        options.getValueOrDefault<std::string>("format", "POINT");
    std::transform(fileFormat.begin(), fileFormat.end(), fileFormat.begin(),
        ::toupper);
    if (fileFormat.find("POINT") != std::string::npos)
        m_header.m_pointFormat = BpfFormat::PointMajor;
    else if (fileFormat.find("BYTE") != std::string::npos)
        m_header.m_pointFormat = BpfFormat::ByteMajor;
    else
        m_header.m_pointFormat = BpfFormat::DimMajor;
}


void BpfWriter::ready(PointContextRef ctx)
{
    Dimension::IdList dims = ctx.dims(); 
    for (auto id : dims)
    {
        BpfDimension dim;
        dim.m_id = id;
        dim.m_label = ctx.dimName(id);
        m_dims.push_back(dim);
    }

    m_stream = FileUtils::createFile(m_filename, true);
    m_header.m_version = 3;
    m_header.m_numDim = dims.size();
    m_header.m_coordType = BpfCoordType::None;
    m_header.m_coordId = 0;
    m_header.setLog(log());

    // We will re-write the header and dimensions to account for the point
    // count and dimension min/max.
    m_header.write(m_stream);
    m_header.writeDimensions(m_stream, m_dims);
    m_header.m_len = m_stream.position();
}


void BpfWriter::write(const PointBuffer& data)
{
    switch (m_header.m_pointFormat)
    {
    case BpfFormat::PointMajor:
        writePointMajor(data);
        break;
    case BpfFormat::DimMajor:
        writeDimMajor(data);
        break;
    case BpfFormat::ByteMajor:
        writeByteMajor(data);
        break;
    }
    m_header.m_numPts += data.size();
}


void BpfWriter::writePointMajor(const PointBuffer& data)
{
    Charbuf charbuf;
    size_t blockpoints = data.size();

    // For compression we're going to write to a buffer so that it can be
    // compressed before it's written to the file stream.
    if (m_header.m_compression)
    {
        // Blocks of 10,000 points will ensure that we're under 16MB, even
        // for 255 dimensions.
        blockpoints = std::min(10000UL, data.size());
        m_compressBuf.resize(blockpoints * sizeof(float) * m_dims.size());
    }
    PointId idx = 0;
    while (idx < data.size())
    {
        if (m_header.m_compression)
        {
            charbuf.initialize(m_compressBuf.data(), m_compressBuf.size());
            m_stream.pushStream(new std::ostream(&charbuf));
        }
        size_t blockId;
        for (blockId = 0; idx < data.size() && blockId < blockpoints;
            ++idx, ++blockId)
        {
            for (auto & bpfDim : m_dims)
            {
                float v = data.getFieldAs<float>(bpfDim.m_id, idx);
                bpfDim.m_min = std::min(bpfDim.m_min, bpfDim.m_offset + v);
                bpfDim.m_max = std::max(bpfDim.m_max, bpfDim.m_offset + v);
                m_stream << v;
            }
        }
        if (m_header.m_compression)
        {
            m_stream.popStream();
            writeCompressedBlock(m_compressBuf.data(),
                blockId * sizeof(float) * m_dims.size());
        }
    }
}


void BpfWriter::writeDimMajor(const PointBuffer& data)
{
    // We're going to pretend for now that we only even have one point buffer.
    Charbuf charbuf;

    if (m_header.m_compression)
        m_compressBuf.resize(data.size() * sizeof(float));
    for (auto & bpfDim : m_dims)
    {
        if (m_header.m_compression)
        {
            charbuf.initialize(m_compressBuf.data(), m_compressBuf.size());
            m_stream.pushStream(new std::ostream(&charbuf));
        }
        for (PointId idx = 0; idx < data.size(); ++idx)
        {
            float v = data.getFieldAs<float>(bpfDim.m_id, idx);
            bpfDim.m_min = std::min(bpfDim.m_min, bpfDim.m_offset + v);
            bpfDim.m_max = std::max(bpfDim.m_max, bpfDim.m_offset + v);
            m_stream << v;
        }
        if (m_header.m_compression)
        {
            std::cerr << "Writing compressed block!\n";
            m_stream.popStream();
            writeCompressedBlock(m_compressBuf.data(), m_compressBuf.size());
        }
    }
}


void BpfWriter::writeByteMajor(const PointBuffer& data)
{
    (void)data;
    throw pdal_error("Writing of byte-segregated is not currently supported.");
}


void BpfWriter::writeCompressedBlock(char *buf, size_t size)
{
    uint32_t rawSize = (uint32_t)size;
    uint32_t compressedSize = 0;

    OStreamMarker blockstart(m_stream);

    // Write dummy size and compressed size to properly position the stream.
    m_stream << rawSize << compressedSize;

    // Have the compressor write to the raw output stream - no byte ordering.
    //ABELL - Perhaps we should add write() support to the OStream.
    std::ostream *out = m_stream.stream();
    BpfCompressor compressor(*out);
    compressor.compress(buf, size);
    compressedSize = (uint32_t)compressor.finish();

    OStreamMarker blockend(m_stream);

    // Now rewind to the start of the block and write the size bytes.
    blockstart.rewind();
    m_stream << rawSize << compressedSize;

    // Now set the position back at the end of the block.
    blockend.rewind();
}


void BpfWriter::done(PointContextRef)
{
    // Rewrite the header to update the the correct number of points and
    // statistics.
    m_stream.seek(0);
    m_header.write(m_stream);
    m_header.writeDimensions(m_stream, m_dims);
    m_stream.flush();
}

} //namespace pdal
