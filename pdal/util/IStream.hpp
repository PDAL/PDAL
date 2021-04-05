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

#include <sys/types.h>
#include <stdint.h>

#include <cassert>
#include <fstream>
#include <memory>
#include <stack>
#include <vector>
#include <cstring>

#include "portable_endian.hpp"
#include "pdal_util_export.hpp"

namespace pdal
{

class IStreamMarker;

/**
  Stream wrapper for input of binary data.
*/
class IStream
{
public:
    //ABELL - Should prevent copy construction.
    //ABELL - Should enable construction from rvalue ref.
    //ABELL - Should provide an operator << (..., string) that delegates
    //  to std::istream
    /**
      Default constructor.
    */
    PDAL_DLL IStream() : m_stream(NULL), m_fstream(NULL)
        {}

    /**
      Construct an IStream from a filename.

      \param filename  File from which to read.
    */
    PDAL_DLL IStream(const std::string& filename) :
        m_stream(NULL), m_fstream(NULL)
    { open(filename); }

    /**
      Construct an IStream from an input stream pointer.

      \param stream  Stream from which to read.
    */
    PDAL_DLL IStream(std::istream *stream) : m_stream(stream), m_fstream(NULL)
        {}

    PDAL_DLL ~IStream()
        { delete m_fstream; }

    /**
      Open a file to extract.

      \param filename  Filename.
      \return  -1 if a stream is already assigned, 0 otherwise.
    */
    PDAL_DLL int open(const std::string& filename)
    {
        if (m_stream)
             return -1;
        m_stream = m_fstream = new std::ifstream(filename,
            std::ios_base::in | std::ios_base::binary);
        return 0;
    }

    /**
      Close the underlying stream.
    */
    PDAL_DLL void close()
    {
        delete m_fstream;
        m_fstream = NULL;
        m_stream = NULL;
    }

    /**
      Return the state of the stream.

      \return  The state of the underlying stream.
    */
    PDAL_DLL operator bool ()
        { return (bool)(*m_stream); }

    /**
      Seek to a position in the underlying stream.

      \param pos  Position to seek to,
    */
    PDAL_DLL void seek(std::streampos pos)
        { m_stream->seekg(pos, std::istream::beg); }


    /**
      Seek to an offset from a specified position.

      \param off  Offset.
      \param way  Absolute position for offset (beg, end or cur)
    */
    PDAL_DLL void seek(std::streampos off, std::ios_base::seekdir way)
        { m_stream->seekg(off, way); }

    /**
      Skip relative to the current position.

      \param offset  Offset from the current position.
    */
    PDAL_DLL void skip(std::streamoff offset)
        { m_stream->seekg(offset, std::istream::cur); }

    /**
      Determine the position of the get pointer.

      \return  Current get position.
    */
    PDAL_DLL std::streampos position() const
        { return m_stream->tellg(); }

    /**
      Determine if the underlying stream is good.

      \return  Whether the underlying stream is good.
    */
    PDAL_DLL bool good() const
        { return m_stream->good(); }

    /**
      Fetch a pointer to the underlying stream.

      \return  Pointer to the underlying stream.
    */
    PDAL_DLL std::istream *stream()
        { return m_stream; }

    /**
      Temporarily push a stream to read from.

      \param strm  New stream to read from.
    */
    PDAL_DLL void pushStream(std::istream *strm)
    {
        m_streams.push(m_stream);
        m_stream = strm;
    }

    /**
      Pop the current stream and return it.  The last stream on the stack
      cannot be popped.

      \return  Pointer to the popped stream.
    */
    PDAL_DLL std::istream *popStream()
    {
        // Can't pop the last stream for now.
        if (m_streams.empty())
            return nullptr;
        std::istream *strm = m_stream;
        m_stream = m_streams.top();
        m_streams.pop();
        return strm;
    }

    /**
      Fetch data from the stream into a string.  NOTE - Stops when
      a null byte is encountered.  Use a buffer/vector reader to
      read data with embedded nulls.

      \param s  String to fill.
      \param size  Maximum number of bytes to extract.
    */
    PDAL_DLL void get(std::string& s, size_t size)
    {
        // Could do this by appending to a string with a stream, but this
        // is probably fast enough for now (there's only a simple increment
        // to advance an istream iterator, which you'd have to call in a loop).

        // Zero-fill for null termination and to avoid reading uninitiallized
        // memory when, for example, trying to read an empty file.
        std::vector<char> buf(size + 1, 0);
        m_stream->read(buf.data(), size);
        s = buf.data();
    }

    /**
      Fetch data from the stream into a vector of char.

      \param buf  Buffer to fill.
    */
    PDAL_DLL void get(std::vector<char>& buf)
    {
        assert(buf.size() != 0);
        m_stream->read((char *)&buf[0], buf.size());
    }

    /**
      Fetch data from the stream into a vector of unsigned char.

      \param buf  Buffer to fill.
    */
    PDAL_DLL void get(std::vector<unsigned char>& buf) {
        assert(buf.size() != 0);
        m_stream->read((char *)&buf[0], buf.size());
    }

    /**
      Fetch data from the stream into the specified buffer of char.

      \param buf  Buffer to fill.
      \param size  Number of bytes to extract.
    */
    PDAL_DLL void get(char *buf, size_t size)
        { m_stream->read(buf, size); }

    /**
      Fetch data from the stream into the specified buffer of unsigned char.

      \param buf  Buffer to fill.
      \param size  Number of bytes to extract.
    */
    PDAL_DLL void get(unsigned char *buf, size_t size)
        { m_stream->read((char *)buf, size); }

protected:
    std::istream *m_stream;
    std::ifstream *m_fstream; // Dup of above to facilitate cleanup.

private:
    std::stack<std::istream *> m_streams;
};

/**
  Stream wrapper for input of binary data that converts from little-endian
  to host ordering.
*/
class ILeStream : public IStream
{
public:
    /**
      Default constructor.
    */
    PDAL_DLL ILeStream()
    {}

    /**
      Constructor that opens the file and maps it to a stream.

      \param filename  Filename.
    */
    PDAL_DLL ILeStream(const std::string& filename) : IStream(filename)
    {}

    /**
      Constructor that maps to a provided stream.

      \param stream  Stream to extract from.
    */
    PDAL_DLL ILeStream(std::istream *stream) : IStream(stream)
    {}

    /**
      Extract an unsigned byte from the stream.

      \param v  unsigned byte to populate
      \return  This stream.
    */
    PDAL_DLL ILeStream& operator >> (uint8_t& v)
    {
        v = (uint8_t)m_stream->get();
        return *this;
    }

    /**
      Extract an unsigned byte from the stream.

      \param v  unsigned byte to populate
      \return  This stream.
    */
    PDAL_DLL ILeStream& operator >> (int8_t& v)
    {
        v = (int8_t)m_stream->get();
        return *this;
    }

    /**
      Extract an unsigned short from the stream.

      \param v  unsigned short to populate
      \return  This stream.
    */
    PDAL_DLL ILeStream& operator >> (uint16_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = le16toh(v);
        return *this;
    }

    /**
      Extract an short from the stream.

      \param v  short to populate
      \return  This stream.
    */
    PDAL_DLL ILeStream& operator >> (int16_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = (int16_t)le16toh((uint16_t)v);
        return *this;
    }

    /**
      Extract an unsigned int from the stream.

      \param v  unsigned int to populate
      \return  This stream.
    */
    PDAL_DLL ILeStream& operator >> (uint32_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = le32toh(v);
        return *this;
    }

    /**
      Extract an int from the stream.

      \param v  int to populate
      \return  This stream.
    */
    PDAL_DLL ILeStream& operator >> (int32_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = (int32_t)le32toh((uint32_t)v);
        return *this;
    }

    /**
      Extract an unsigned long int from the stream.

      \param v  unsigned long int to populate
      \return  This stream.
    */
    PDAL_DLL ILeStream& operator >> (uint64_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = le64toh(v);
        return *this;
    }

    /**
      Extract a long int from the stream.

      \param v  long int to populate
      \return  This stream.
    */
    PDAL_DLL ILeStream& operator >> (int64_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = (int64_t)le64toh((uint64_t)v);
        return *this;
    }

    /**
      Extract a float from the stream.

      \param v  float to populate
      \return  This stream.
    */
    PDAL_DLL ILeStream& operator >> (float& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        uint32_t tmp = le32toh(*(uint32_t *)(&v));
        std::memcpy(&v, &tmp, sizeof(tmp));
        return *this;
    }

    /**
      Extract a double from the stream.

      \param v  double to populate
      \return  This stream.
    */
    PDAL_DLL ILeStream& operator >> (double& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        uint64_t tmp = le64toh(*(uint64_t *)(&v));
        std::memcpy(&v, &tmp, sizeof(tmp));
        return *this;
    }
};


/**
  Stream wrapper for input of binary data that converts from big-endian
  to host ordering.
*/
class IBeStream : public IStream
{
public:
    /**
      Default constructor.
    */
    PDAL_DLL IBeStream()
    {}

    /**
      Constructor that opens the file and maps it to a stream.

      \param filename  Filename.
    */
    PDAL_DLL IBeStream(const std::string& filename) : IStream(filename)
    {}

    /**
      Constructor that maps to a provided stream.

      \param stream  Stream to extract from.
    */
    PDAL_DLL IBeStream(std::istream *stream) : IStream(stream)
    {}

    /**
      Extract an unsigned byte from the stream.

      \param v  unsigned byte to populate
      \return  This stream.
    */
    PDAL_DLL IBeStream& operator >> (uint8_t& v)
    {
        v = (uint8_t)m_stream->get();
        return *this;
    }

    /**
      Extract an unsigned byte from the stream.

      \param v  unsigned byte to populate
      \return  This stream.
    */
    PDAL_DLL IBeStream& operator >> (int8_t& v)
    {
        v = (int8_t)m_stream->get();
        return *this;
    }

    /**
      Extract an unsigned short from the stream.

      \param v  unsigned short to populate
      \return  This stream.
    */
    PDAL_DLL IBeStream& operator >> (uint16_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = be16toh(v);
        return *this;
    }

    /**
      Extract an short from the stream.

      \param v  short to populate
      \return  This stream.
    */
    PDAL_DLL IBeStream& operator >> (int16_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = (int16_t)be16toh((uint16_t)v);
        return *this;
    }

    /**
      Extract an unsigned int from the stream.

      \param v  unsigned int to populate
      \return  This stream.
    */
    PDAL_DLL IBeStream& operator >> (uint32_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = be32toh(v);
        return *this;
    }

    /**
      Extract an int from the stream.

      \param v  int to populate
      \return  This stream.
    */
    PDAL_DLL IBeStream& operator >> (int32_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = (int32_t)be32toh((uint32_t)v);
        return *this;
    }

    /**
      Extract an unsigned long int from the stream.

      \param v  unsigned long int to populate
      \return  This stream.
    */
    PDAL_DLL IBeStream& operator >> (uint64_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = be64toh(v);
        return *this;
    }

    /**
      Extract a long int from the stream.

      \param v  long int to populate
      \return  This stream.
    */
    PDAL_DLL IBeStream& operator >> (int64_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = (int64_t)be64toh((uint64_t)v);
        return *this;
    }

    /**
      Extract a float from the stream.

      \param v  float to populate
      \return  This stream.
    */
    PDAL_DLL IBeStream& operator >> (float& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        uint32_t tmp = be32toh(*(uint32_t *)(&v));
        std::memcpy(&v, &tmp, sizeof(tmp));
        return *this;
    }

    /**
      Extract a double from the stream.

      \param v  double to populate
      \return  This stream.
    */
    PDAL_DLL IBeStream& operator >> (double& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        uint64_t tmp = be64toh(*(uint64_t *)(&v));
        std::memcpy(&v, &tmp, sizeof(tmp));
        return *this;
    }
};


/**
  Stream wrapper for input of binary data that converts from either
  little-endian or big-endian to host ordering, depending on object
  settings.
*/
class ISwitchableStream : public IStream
{
public:
    static const bool DefaultIsLittleEndian = true;

    PDAL_DLL ISwitchableStream() : m_isLittleEndian(DefaultIsLittleEndian)
    {}

    PDAL_DLL ISwitchableStream(const std::string& filename)
        : IStream(filename)
        , m_isLittleEndian(DefaultIsLittleEndian)
    {}

    PDAL_DLL ISwitchableStream(std::istream* stream)
        : IStream(stream)
        , m_isLittleEndian(DefaultIsLittleEndian)
    {}

    PDAL_DLL bool isLittleEndian() const
        { return m_isLittleEndian; }
    PDAL_DLL void switchToLittleEndian()
        { m_isLittleEndian = true; }
    PDAL_DLL void switchToBigEndian()
        { m_isLittleEndian = false; }

    PDAL_DLL ISwitchableStream& operator>>(uint8_t& v)
    {
        v = (uint8_t)m_stream->get();
        return *this;
    }

    PDAL_DLL ISwitchableStream& operator>>(int8_t& v)
    {
        v = (int8_t)m_stream->get();
        return *this;
    }

    PDAL_DLL ISwitchableStream& operator>>(uint16_t& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        v = isLittleEndian() ? le16toh(v) : be16toh(v);
        return *this;
    }

    PDAL_DLL ISwitchableStream& operator>>(int16_t& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        v = isLittleEndian() ? (int16_t)le16toh((uint16_t)v)
                             : (int16_t)be16toh((uint16_t)v);
        return *this;
    }

    PDAL_DLL ISwitchableStream& operator>>(uint32_t& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        v = isLittleEndian() ? le32toh(v) : be32toh(v);
        return *this;
    }

    PDAL_DLL ISwitchableStream& operator>>(int32_t& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        v = isLittleEndian() ? (int32_t)le32toh((uint32_t)v)
                             : (int32_t)be32toh((uint32_t)v);
        return *this;
    }

    PDAL_DLL ISwitchableStream& operator>>(uint64_t& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        v = isLittleEndian() ? le64toh(v) : be64toh(v);
        return *this;
    }

    PDAL_DLL ISwitchableStream& operator>>(int64_t& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        v = isLittleEndian() ? (int64_t)le64toh((uint64_t)v)
                             : (int64_t)be64toh((uint64_t)v);
        return *this;
    }

    PDAL_DLL ISwitchableStream& operator>>(float& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        uint32_t tmp = isLittleEndian() ? le32toh(*(uint32_t*)(&v))
                                        : be32toh(*(uint32_t*)(&v));
        std::memcpy(&v, &tmp, sizeof(tmp));
        return *this;
    }

    PDAL_DLL ISwitchableStream& operator>>(double& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        uint64_t tmp = isLittleEndian() ? be64toh(*(uint64_t*)(&v))
                                        : be64toh(*(uint64_t*)(&v));
        std::memcpy(&v, &tmp, sizeof(tmp));
        return *this;
    }

private:
    bool m_isLittleEndian;
};


/// Stream position marker with rewinding support.
class IStreamMarker
{
public:
    PDAL_DLL IStreamMarker(IStream& stream) : m_stream(stream)
        { m_pos = m_stream.position(); }

    PDAL_DLL void rewind()
        { m_stream.seek(m_pos); }

private:
    std::streampos m_pos;
    IStream& m_stream;
	IStreamMarker(const IStreamMarker&);
    IStreamMarker& operator=(const IStreamMarker&); // not implemented
};

} // namespace pdal
