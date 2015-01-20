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

#include "BpfCompressor.hpp"

#include <pdal/pdal_internal.hpp>

namespace pdal
{

void BpfCompressor::startBlock()
{
    // Initialize the stream.
    m_strm.zalloc = Z_NULL;
    m_strm.zfree = Z_NULL;
    m_strm.opaque = Z_NULL;
    if (deflateInit(&m_strm, Z_DEFAULT_COMPRESSION) != Z_OK)
        throw pdal_error("Could not initialize BPF compressor.");

    m_rawSize = 0;
    m_compressedSize = 0;
    uint32_t rawSize = 0;
    uint32_t compressedSize = 0;

    // Initiailize the streambuf with the backing buffer.
    m_charbuf.initialize(m_inbuf.data(), m_inbuf.size());

    // Note our position so that we can rewind here.
    m_blockStart.mark();
    m_out << rawSize << compressedSize;

    // Make a new stream from our charbuf and push it so that future writes
    // to our stream go to the backing vector.
    m_out.pushStream(new std::ostream(&m_charbuf));
}


void BpfCompressor::compress()
{
    // Note our position so that we know how much we've written.
    std::size_t rawWritten = m_out.position();

    // Pop our temp stream so that we can write the real output file.
    m_out.popStream();

    m_rawSize += rawWritten;

    // Deflate the data in the buffer and write it to the output stream.
    m_strm.avail_in = rawWritten;
    m_strm.next_in = (unsigned char *)m_inbuf.data();
    m_strm.avail_out = CHUNKSIZE;
    m_strm.next_out = m_tmpbuf;
    while (m_strm.avail_in)
    {
        int ret = ::deflate(&m_strm, Z_NO_FLUSH);
        size_t written = CHUNKSIZE - m_strm.avail_out;
        m_compressedSize += written;
        m_out.put(m_tmpbuf, written);
        m_strm.avail_out = CHUNKSIZE;
        m_strm.next_out = m_tmpbuf;
    }

    // All data has been written.  Reinitialize input buffer's streambuf and
    // push it.
    m_charbuf.initialize(m_inbuf.data(), m_inbuf.size());
    m_out.pushStream(new std::ostream(&m_charbuf));
}


void BpfCompressor::finish()
{
    // Pop our special stream so that we can write the the file.
    m_out.popStream();

    // Deflate and write the result to the output file.
    int ret = Z_OK;
    while (ret == Z_OK)
    {
        ret = ::deflate(&m_strm, Z_FINISH);
        size_t written = CHUNKSIZE - m_strm.avail_out;
        m_compressedSize += written;
        m_out.put(m_tmpbuf, written);
        m_strm.avail_out = CHUNKSIZE;
        m_strm.next_out = m_tmpbuf;
    }
    if (ret != Z_STREAM_END)
        throw pdal_error("Couldn't close BPF compression stream.");
    deflateEnd(&m_strm);

    // Mark our position so that we can get back here.
    OStreamMarker blockEnd(m_out);

    // Rewind to the start of the block and write the size bytes.
    m_blockStart.rewind();
    m_out << (uint32_t)m_rawSize << (uint32_t)m_compressedSize;

    // Set the position back to the end of the block.
    blockEnd.rewind();
}
   
} // namespace pdal
