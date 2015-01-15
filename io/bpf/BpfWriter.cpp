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

#include <pdal/pdal_internal.hpp>

#include <zlib.h>

#include <pdal/Options.hpp>

#include "BpfWriter.hpp"

namespace pdal
{

void BpfWriter::processOptions(const Options&)
{
    if (m_filename.empty())
        throw pdal_error("Can't write BPF file without filename.");
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
    //ABELL - For now.
    m_header.m_pointFormat = BpfFormat::PointMajor;
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
    writePointMajor(data);
    m_header.m_numPts += data.size();
}


void BpfWriter::writePointMajor(const PointBuffer& data)
{
    for (PointId idx = 0; idx < data.size(); ++idx)
    {
        for (auto & bpfDim : m_dims)
        {
            float v = data.getFieldAs<float>(bpfDim.m_id, idx);
            bpfDim.m_min = std::min(bpfDim.m_min, bpfDim.m_offset + v);
            bpfDim.m_max = std::min(bpfDim.m_max, bpfDim.m_offset + v);
            m_stream << v;
        }
    }
}


void BpfWriter::done(PointContextRef)
{
    m_stream.seek(0);
    m_header.write(m_stream);
    m_header.writeDimensions(m_stream, m_dims);
    m_stream.flush();
}

} //namespace pdal
