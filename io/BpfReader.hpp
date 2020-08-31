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

// BPF is an NGA specification for point cloud data. The specification can be
// found at https://nsgreg.nga.mil/doc/view?i=4202

#pragma once

#include <vector>

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/Charbuf.hpp>
#include <pdal/util/IStream.hpp>
#include <pdal/pdal_export.hpp>

#include "BpfHeader.hpp"

#include <vector>

namespace pdal
{

class PDAL_DLL BpfReader : public Reader, public Streamable
{
    struct Args;
public:
    BpfReader();
    ~BpfReader();

    std::string getName() const;

    virtual point_count_t numPoints() const
        { return (point_count_t)m_header.m_numPts; }

private:
    std::istream* m_istreamPtr;
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
    std::unique_ptr<Args> m_args;

    // For dimension-major point-at-a-time usage.
    std::vector<std::unique_ptr<ILeStream>> m_streams;
    std::vector<std::unique_ptr<Charbuf>> m_charbufs;

    virtual QuickInfo inspect();
    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr Layout);
    virtual void addArgs(ProgramArgs& args);
    virtual void ready(PointTableRef table);
    virtual bool processOne(PointRef& point);
    virtual point_count_t read(PointViewPtr data, point_count_t num);
    virtual void done(PointTableRef table);

    bool readUlemData();
    bool readUlemFiles();
    bool readHeaderExtraData();
    bool readPolarData();
    void readPointMajor(PointRef& point);
    point_count_t readPointMajor(PointViewPtr data, point_count_t count);
    void readDimMajor(PointRef& point);
    point_count_t readDimMajor(PointViewPtr data, point_count_t count);
    void readByteMajor(PointRef& point);
    point_count_t readByteMajor(PointViewPtr data, point_count_t count);
    size_t readBlock(std::vector<char>& outBuf, size_t index);
    bool eof();
    int inflate(char *inbuf, uint32_t insize, char *outbuf, uint32_t outsize);

    void seekPointMajor(PointId ptIdx);
    void seekDimMajor(size_t dimIdx, PointId ptIdx);
    void seekByteMajor(size_t dimIdx, size_t byteIdx, PointId ptIdx);
};

} // namespace pdal
