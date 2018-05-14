/******************************************************************************
* Copyright (c) 2014, Howard Butler (howard@hobu.co)
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

#include <memory>
#include <pdal/util/OStream.hpp>

namespace laszip
{
namespace factory
{
    struct record_schema;
}
}

namespace pdal
{
    class LazPerfVlrCompressorImpl;

// This compressor write data in chunks to a stream. At the beginning of the
// data is an offset to the end of the data, where the chunk table is
// stored.  The chunk table keeps a list of the offsets to the beginning of
// each chunk.  Chunks consist of a fixed number of points (last chunk may
// have fewer points).  Each time a chunk starts, the compressor is reset.
// This allows decompression of some set of points that's less than the
// entire set when desired.
// The compressor uses the schema of the point data in order to compress
// the point stream.  The schema is also stored in a VLR that isn't
// handled as part of the compression process itself.
class LazPerfVlrCompressor
{
    typedef laszip::factory::record_schema Schema;

public:
    PDAL_DLL LazPerfVlrCompressor(std::ostream& stream, const Schema& schema,
        uint32_t chunksize);
    PDAL_DLL ~LazPerfVlrCompressor();

    PDAL_DLL void compress(const char *inbuf);
    PDAL_DLL void done();

private:
    std::unique_ptr<LazPerfVlrCompressorImpl> m_impl;
};


class LazPerfVlrDecompressorImpl;

class LazPerfVlrDecompressor
{
public:
    PDAL_DLL LazPerfVlrDecompressor(std::istream& stream, const char *vlrData,
        std::streamoff pointOffset);
    PDAL_DLL ~LazPerfVlrDecompressor();

    PDAL_DLL size_t pointSize() const;
    PDAL_DLL void decompress(char *outbuf);

private:
    std::unique_ptr<LazPerfVlrDecompressorImpl> m_impl;
};

} // namespace pdal

