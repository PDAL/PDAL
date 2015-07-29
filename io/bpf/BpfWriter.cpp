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

#include <pdal/Options.hpp>
#include <pdal/pdal_export.hpp>

#include <zlib.h>

#include "BpfCompressor.hpp"

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.bpf",
    "\"Binary Point Format\" (BPF) writer support. BPF is a simple \n" \
        "DoD and research format that is used by some sensor and \n" \
        "processing chains.",
    "http://pdal.io/stages/writers.bpf.html" );

CREATE_STATIC_PLUGIN(1, 0, BpfWriter, Writer, s_info)

std::string BpfWriter::getName() const { return s_info.name; }

Options BpfWriter::getDefaultOptions()
{
    Options ops;

    ops.add("filename", "", "Filename for BPF output");
    ops.add("compression", false, "Whether zlib compression should be used");
    ops.add("format", "dimension", "Point output format: "
        "non-interleaved(\"dimension\"), interleaved(\"point\") or "
        "byte-segregated(\"byte\")");
    ops.add("coord_id", 0, "Coordinate ID (UTM zone).");
    return ops;
}


void BpfWriter::processOptions(const Options& options)
{
    bool compression = options.getValueOrDefault("compression", false);
    m_header.m_compression = compression ? BpfCompression::Zlib :
        BpfCompression::None;

    std::string encodedHeader =
        options.getValueOrDefault<std::string>("header_data");
    m_extraData = Utils::base64_decode(encodedHeader);

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
    if (options.hasOption("coord_id"))
    {
        m_header.m_coordType = BpfCoordType::UTM;
        m_header.m_coordId = options.getValueOrThrow<int>("coord_id");
    }
    else
    {
        m_header.m_coordType = BpfCoordType::None;
        m_header.m_coordId = 0;
    }
    StringList files = options.getValues<std::string>("bundledfile");
    for (auto file : files)
    {
        if (!FileUtils::fileExists(file))
        {
            std::ostringstream oss;

            oss << getName() << ": bundledfile '" << file << "' doesn't exist.";
            throw pdal_error(oss.str());
        }

        size_t size = FileUtils::fileSize(file);
        if (size > (std::numeric_limits<uint32_t>::max)())
        {
            std::ostringstream oss;
            oss << getName() << ": bundledfile '" << file << "' too large.";
            throw pdal_error(oss.str());
        }

        BpfUlemFile ulemFile;
        ulemFile.m_len = size;
        ulemFile.m_filespec = file;
        ulemFile.m_filename = FileUtils::getFilename(file);
        if (ulemFile.m_filename.length() > 32)
        {
            std::ostringstream oss;
            oss << getName() << ": bundledfile '" << file << "' name "
                "exceeds maximum length of 32.";
            throw pdal_error(oss.str());
        }
        m_bundledFiles.push_back(ulemFile);
    }
}


void BpfWriter::readyTable(PointTableRef table)
{
    loadBpfDimensions(table.layout());
}


void BpfWriter::readyFile(const std::string& filename)
{
    m_stream.open(filename);
    m_header.m_version = 3;
    m_header.m_numDim = m_dims.size();
    m_header.m_numPts = 0;
    m_header.setLog(log());

    // We will re-write the header and dimensions to account for the point
    // count and dimension min/max.
    m_header.write(m_stream);
    m_header.writeDimensions(m_stream, m_dims);
    for (auto& file : m_bundledFiles)
        file.write(m_stream);
    m_stream.put((const char *)m_extraData.data(), m_extraData.size());

    m_header.m_len = m_stream.position();

    m_header.m_xform.m_vals[0] = m_xXform.m_scale;
    m_header.m_xform.m_vals[5] = m_yXform.m_scale;
    m_header.m_xform.m_vals[10] = m_zXform.m_scale;
}


void BpfWriter::loadBpfDimensions(PointLayoutPtr layout)
{
    // Verify that we have X, Y and Z and that they're the first three
    // dimensions.
    Dimension::IdList dims = layout->dims();
    std::sort(dims.begin(), dims.end());
    if (dims.size() < 3 || dims[0] != Dimension::Id::X ||
        dims[1] != Dimension::Id::Y || dims[2] != Dimension::Id::Z)
    {
        throw pdal_error("Missing one of dimensions X, Y or Z.  "
            "Can't write BPF.");
    }

    for (auto id : dims)
    {
        BpfDimension dim;
        dim.m_id = id;
        dim.m_label = layout->dimName(id);
        m_dims.push_back(dim);
    }
}


void BpfWriter::writeView(const PointViewPtr dataShared)
{
    setAutoXForm(dataShared);

    // Avoid reference count overhead internally.
    const PointView* data(dataShared.get());

    // We know that X, Y and Z are dimensions 0, 1 and 2.
    m_dims[0].m_offset = m_xXform.m_offset;
    m_dims[1].m_offset = m_yXform.m_offset;
    m_dims[2].m_offset = m_zXform.m_offset;

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
    m_header.m_numPts += data->size();
}


void BpfWriter::writePointMajor(const PointView* data)
{
    // Blocks of 10,000 points will ensure that we're under 16MB, even
    // for 255 dimensions.
    size_t blockpoints = std::min<point_count_t>(10000UL, data->size());

    // For compression we're going to write to a buffer so that it can be
    // compressed before it's written to the file stream.
    BpfCompressor compressor(m_stream,
        blockpoints * sizeof(float) * m_dims.size());
    PointId idx = 0;
    while (idx < data->size())
    {
        if (m_header.m_compression)
            compressor.startBlock();
        size_t blockId;
        for (blockId = 0; idx < data->size() && blockId < blockpoints;
            ++idx, ++blockId)
        {
            for (auto & bpfDim : m_dims)
            {
                double d = getAdjustedValue(data, bpfDim, idx);
                m_stream << (float)d;
            }
        }
        if (m_header.m_compression)
        {
            compressor.compress();
            compressor.finish();
        }
    }
}


void BpfWriter::writeDimMajor(const PointView* data)
{
    // We're going to pretend for now that we only even have one point buffer.
    BpfCompressor compressor(m_stream, data->size() * sizeof(float));

    for (auto & bpfDim : m_dims)
    {

        if (m_header.m_compression)
            compressor.startBlock();
        for (PointId idx = 0; idx < data->size(); ++idx)
        {
            double d = getAdjustedValue(data, bpfDim, idx);
            m_stream << (float)d;
        }
        if (m_header.m_compression)
        {
            compressor.compress();
            compressor.finish();
        }
    }
}


void BpfWriter::writeByteMajor(const PointView* data)
{
    union
    {
        float f;
        uint32_t u32;
    } uu;

    // We're going to pretend for now that we only ever have one point buffer.
    BpfCompressor compressor(m_stream,
        data->size() * sizeof(float) * m_dims.size());

    if (m_header.m_compression)
        compressor.startBlock();
    for (auto & bpfDim : m_dims)
    {
        for (size_t b = 0; b < sizeof(float); b++)
        {
            for (PointId idx = 0; idx < data->size(); ++idx)
            {
                uu.f = (float)getAdjustedValue(data, bpfDim, idx);
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


double BpfWriter::getAdjustedValue(const PointView* data,
    BpfDimension& bpfDim, PointId idx)
{
    double d = data->getFieldAs<double>(bpfDim.m_id, idx);
    bpfDim.m_min = std::min(bpfDim.m_min, d);
    bpfDim.m_max = std::max(bpfDim.m_max, d);

    if (bpfDim.m_id == Dimension::Id::X)
        d /= m_xXform.m_scale;
    else if (bpfDim.m_id == Dimension::Id::Y)
        d /= m_yXform.m_scale;
    else if (bpfDim.m_id == Dimension::Id::Z)
        d /= m_zXform.m_scale;
    return (d - bpfDim.m_offset);
}


void BpfWriter::doneFile()
{
    // Rewrite the header to update the the correct number of points and
    // statistics.
    m_stream.seek(0);
    m_header.write(m_stream);
    m_header.writeDimensions(m_stream, m_dims);
    m_stream.close();
}

} //namespace pdal

