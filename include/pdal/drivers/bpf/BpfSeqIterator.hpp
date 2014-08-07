/******************************************************************************
* Copyright (c) 2014, Hobu Inc.
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

#include <pdal/Charbuf.hpp>
#include <pdal/IStream.hpp>
#include <pdal/ReaderIterator.hpp>
#include "BpfHeader.hpp"

#include <vector>

namespace pdal
{

class BpfReader;
class Dimension;
class PointBuffer;

class BpfSeqIterator : public ReaderSequentialIterator
{
public:
    BpfSeqIterator(const std::vector<Dimension *>& dims,
        point_count_t numPoints, BpfFormat::Enum pointFormat, bool compression,
        ILeStream& stream);
    ~BpfSeqIterator();

protected:
    point_count_t readBufferImpl(PointBuffer&);
    virtual point_count_t readImpl(PointBuffer& data, point_count_t count);
    boost::uint64_t skipImpl(boost::uint64_t);
    bool atEndImpl() const;

    point_count_t read(PointBuffer& data, point_count_t count);
    point_count_t readPointMajor(PointBuffer& data, point_count_t count);
    point_count_t readDimMajor(PointBuffer& data, point_count_t count);
    point_count_t readByteMajor(PointBuffer& data, point_count_t count);

private:
    size_t readBlock(std::vector<char>& outBuf, size_t index);
#ifdef PDAL_HAVE_ZLIB
    int inflate(char *inbuf, size_t insize, char *outbuf, size_t outsize);
#endif
    void seekPointMajor(uint32_t ptIdx);
    void seekDimMajor(size_t dimIdx, uint32_t ptIdx);
    void seekByteMajor(size_t dimIdx, size_t byteIdx, uint32_t ptIdx);

    /// Dimensions
    std::vector<Dimension *> m_dims;
    /// Total number of points in the file.
    point_count_t m_numPoints;
    /// Bpf point format being read.
    BpfFormat::Enum m_pointFormat;
    /// Input stream
    ILeStream& m_stream;
    /// Index of the next point to read.
    point_count_t m_index;
    /// Stream position when the iterator is created (should always be
    /// the start of point data).
    std::streampos m_start;
    /// Buffer for deflated data.
    std::vector<char> m_deflateBuf;
    /// Streambuf for deflated data.
    Charbuf m_charbuf;
};

} // namespace pdal

