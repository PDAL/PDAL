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
    // Blocks of 10,000 points will ensure that we're under 16MB, even
    // for 255 dimensions.
    size_t blockpoints = std::min(10000UL, data.size());

    // For compression we're going to write to a buffer so that it can be
    // compressed before it's written to the file stream.
    BpfCompressor compressor(m_stream,
        blockpoints * sizeof(float) * m_dims.size());
    PointId idx = 0;
    while (idx < data.size())
    {
        if (m_header.m_compression)
            compressor.startBlock();
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
            compressor.compress();
            compressor.finish();
        }
    }
}


void BpfWriter::writeDimMajor(const PointBuffer& data)
{
    // We're going to pretend for now that we only even have one point buffer.
    BpfCompressor compressor(m_stream, data.size() * sizeof(float));

    for (auto & bpfDim : m_dims)
    {
        
        if (m_header.m_compression)
            compressor.startBlock();
        for (PointId idx = 0; idx < data.size(); ++idx)
        {
            float v = data.getFieldAs<float>(bpfDim.m_id, idx);
            bpfDim.m_min = std::min(bpfDim.m_min, bpfDim.m_offset + v);
            bpfDim.m_max = std::max(bpfDim.m_max, bpfDim.m_offset + v);
            m_stream << v;
        }
        if (m_header.m_compression)
        {
            compressor.compress();
            compressor.finish();
        }
    }
}


void BpfWriter::writeByteMajor(const PointBuffer& data)
{
    union
    {
        float f;
        uint32_t u32;
    } uu;

    // We're going to pretend for now that we only even have one point buffer.

    BpfCompressor compressor(m_stream,
        data.size() * sizeof(float) * m_dims.size());

    if (m_header.m_compression)
        compressor.startBlock();
    for (auto & bpfDim : m_dims)
    {
        for (size_t b = 0; b < sizeof(float); b++)
        {
            for (PointId idx = 0; idx < data.size(); ++idx)
            {
                uu.f = data.getFieldAs<float>(bpfDim.m_id, idx);
                uint8_t u8 = (uint8_t)(uu.u32 >> (b * CHAR_BIT));
                m_stream << u8;
            }
        }
    }
    if (m_header.m_compression)
    {
        compressor.compress();
        compressor.finish();
    }
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
