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

#pragma once

#include <vector>

#include <pdal/Charbuf.hpp>
#include <pdal/IStream.hpp>
#include <pdal/Reader.hpp>

#include "BpfHeader.hpp"

namespace pdal
{

#define BPFREADERDOC "\"Binary Point Format\" (BPF) reader support. BPF is a simple \n" \
                     "DoD and research format that is used by some sensor and \n" \
                     "processing chains."

class PDAL_DLL BpfReader : public Reader
{
public:
    SET_STAGE_NAME("readers.bpf", BPFREADERDOC)
    SET_STAGE_LINK("http://pdal.io/stages/readers.bpf.html")

    virtual point_count_t numPoints() const
        {  return (point_count_t)m_header.m_numPts; }
private:
    ILeStream m_stream;
    BpfHeader m_header;
    BpfDimensionList m_dims;
    Dimension::IdList m_schemaDims;
    BpfUlemHeader m_ulemHeader;
    std::vector<BpfUlemFrame> m_ulemFrames;
    BpfPolarHeader m_polarHeader;
    std::vector<BpfPolarFrame> m_polarFrames;
    /// Stream position at the beginning of point records.
    std::streampos m_start;
    /// Index of the next point to read.
    point_count_t m_index;
    /// Buffer for deflated data.
    std::vector<char> m_deflateBuf;
    /// Streambuf for deflated data.
    Charbuf m_charbuf;

    virtual void processOptions(const Options& options);
    virtual QuickInfo inspect();
    virtual void initialize();
    virtual void addDimensions(PointContextRef ctx);
    virtual void ready(PointContextRef ctx);
    virtual point_count_t read(PointBuffer& buf, point_count_t num);
    virtual void done(PointContextRef ctx);
    virtual bool eof();

    bool readUlemData();
    bool readUlemFiles();
    bool readHeaderExtraData();
    bool readPolarData();
    point_count_t readPointMajor(PointBuffer& data, point_count_t count);
    point_count_t readDimMajor(PointBuffer& data, point_count_t count);
    point_count_t readByteMajor(PointBuffer& data, point_count_t count);
    size_t readBlock(std::vector<char>& outBuf, size_t index);

    int inflate(char *inbuf, size_t insize, char *outbuf, size_t outsize);

    void seekPointMajor(PointId ptIdx);
    void seekDimMajor(size_t dimIdx, PointId ptIdx);
    void seekByteMajor(size_t dimIdx, size_t byteIdx, PointId ptIdx);
};

} // namespace pdal
