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

#include <fstream>
#include <memory>
#include <stack>
#include <vector>
#include <cstring>

#include <pdal/pdal_export.hpp>

#include "portable_endian.hpp"

namespace pdal
{

class IStreamMarker;

/// Stream wrapper for input of binary data.
class PDAL_DLL IStream
{
public:
    IStream() : m_stream(NULL), m_fstream(NULL)
        {}
    IStream(const std::string& filename) : m_stream(NULL), m_fstream(NULL)
        { open(filename); }
    IStream(std::istream *stream) : m_stream(stream), m_fstream(NULL)
        {}
    ~IStream()
        { delete m_fstream; }

    int open(const std::string& filename)
    {
        if (m_stream)
             return -1;
        m_stream = m_fstream = new std::ifstream(filename,
            std::ios_base::in | std::ios_base::binary);
        return 0;
    }
    operator bool ()
        { return (bool)(*m_stream); }
    void seek(std::streampos pos)
        { m_stream->seekg(pos, std::istream::beg); }
    void seek(std::streampos pos, std::ios_base::seekdir way)
        { m_stream->seekg(pos, way); }
    void skip(std::streamoff offset)
        { m_stream->seekg(offset, std::istream::cur); }
    std::streampos position() const
        { return m_stream->tellg(); }
    bool good() const
        { return m_stream->good(); }
    std::istream *stream()
        { return m_stream; }
    void pushStream(std::istream *strm)
    {
        m_streams.push(m_stream);
        m_stream = strm;
    }
    std::istream *popStream()
    {
        // Can't pop the last stream for now.
        if (m_streams.empty())
            return nullptr;
        std::istream *strm = m_stream;
        m_stream = m_streams.top();
        m_streams.pop();
        return strm;
    }

    void get(std::string& s, size_t size)
    {
        // Could do this by appending to a string with a stream, but this
        // is probably fast enough for now (there's only a simple increment
        // to advance an istream iterator, which you'd have to call in a loop).
        std::unique_ptr<char[]> buf(new char[size+1]);
        m_stream->read(buf.get(), size);
        buf[size] = '\0';
        s = buf.get();
    }

    void get(std::vector<char>& buf)
        { m_stream->read(&buf[0], buf.size()); }

    void get(std::vector<unsigned char>& buf)
        { m_stream->read((char *)&buf[0], buf.size()); }

    void get(char *buf, size_t size)
        { m_stream->read(buf, size); }

    void get(unsigned char *buf, size_t size)
        { m_stream->read((char *)buf, size); }

protected:
    std::istream *m_stream;
    std::istream *m_fstream; // Dup of above to facilitate cleanup.

private:
    std::stack<std::istream *> m_streams;
	IStream(const IStream&);
};

/// Stream wrapper for input of binary data that converts from little-endian
/// to host ordering.
class ILeStream : public IStream
{
public:
    ILeStream()
    {}
    ILeStream(const std::string& filename) : IStream(filename)
    {}
    ILeStream(std::istream *stream) : IStream(stream)
    {}

    ILeStream& operator >> (uint8_t& v)
    {
        v = (uint8_t)m_stream->get();
        return *this;
    }

    ILeStream& operator >> (int8_t& v)
    {
        v = (int8_t)m_stream->get();
        return *this;
    }

    ILeStream& operator >> (uint16_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = le16toh(v);
        return *this;
    }

    ILeStream& operator >> (int16_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = (int16_t)le16toh((uint16_t)v);
        return *this;
    }

    ILeStream& operator >> (uint32_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = le32toh(v);
        return *this;
    }

    ILeStream& operator >> (int32_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = (int32_t)le32toh((uint32_t)v);
        return *this;
    }

    ILeStream& operator >> (uint64_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = le64toh(v);
        return *this;
    }

    ILeStream& operator >> (int64_t& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        v = (int64_t)le64toh((uint64_t)v);
        return *this;
    }

    ILeStream& operator >> (float& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        uint32_t tmp = le32toh(*(uint32_t *)(&v));
        std::memcpy(&v, &tmp, sizeof(tmp));
        return *this;
    }

    ILeStream& operator >> (double& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        uint64_t tmp = le64toh(*(uint64_t *)(&v));
        std::memcpy(&v, &tmp, sizeof(tmp));
        return *this;
    }
};


/// Stream wrapper for input of binary data that converts from
/// either little-endian or big-endian to host ordering,
/// depending on object settings
class ISwitchableStream : public IStream
{
public:
    static const bool DefaultIsLittleEndian = true;

    ISwitchableStream()
        : m_isLittleEndian(DefaultIsLittleEndian)
    {
    }
    ISwitchableStream(const std::string& filename)
        : IStream(filename)
        , m_isLittleEndian(DefaultIsLittleEndian)
    {
    }
    ISwitchableStream(std::istream* stream)
        : IStream(stream)
        , m_isLittleEndian(DefaultIsLittleEndian)
    {
    }

    bool isLittleEndian() const { return m_isLittleEndian; }
    void switchToLittleEndian() { m_isLittleEndian = true; }
    void switchToBigEndian() { m_isLittleEndian = false; }

    ISwitchableStream& operator>>(uint8_t& v)
    {
        v = (uint8_t)m_stream->get();
        return *this;
    }

    ISwitchableStream& operator>>(int8_t& v)
    {
        v = (int8_t)m_stream->get();
        return *this;
    }

    ISwitchableStream& operator>>(uint16_t& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        v = isLittleEndian() ? le16toh(v) : be16toh(v);
        return *this;
    }

    ISwitchableStream& operator>>(int16_t& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        v = isLittleEndian() ? (int16_t)le16toh((uint16_t)v)
                             : (int16_t)be16toh((uint16_t)v);
        return *this;
    }

    ISwitchableStream& operator>>(uint32_t& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        v = isLittleEndian() ? le32toh(v) : be32toh(v);
        return *this;
    }

    ISwitchableStream& operator>>(int32_t& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        v = isLittleEndian() ? (int32_t)le32toh((uint32_t)v)
                             : (int32_t)be32toh((uint32_t)v);
        return *this;
    }

    ISwitchableStream& operator>>(uint64_t& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        v = isLittleEndian() ? le64toh(v) : be64toh(v);
        return *this;
    }

    ISwitchableStream& operator>>(int64_t& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        v = isLittleEndian() ? (int64_t)le64toh((uint64_t)v)
                             : (int64_t)be64toh((uint64_t)v);
        return *this;
    }

    ISwitchableStream& operator>>(float& v)
    {
        m_stream->read((char*)&v, sizeof(v));
        uint32_t tmp = isLittleEndian() ? le32toh(*(uint32_t*)(&v))
                                        : be32toh(*(uint32_t*)(&v));
        std::memcpy(&v, &tmp, sizeof(tmp));
        return *this;
    }

    ISwitchableStream& operator>>(double& v)
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
    IStreamMarker(IStream& stream) : m_stream(stream)
        { m_pos = m_stream.position(); }

    void rewind()
        { m_stream.seek(m_pos); }

private:
    std::streampos m_pos;
    IStream& m_stream;
	IStreamMarker(const IStreamMarker&);
    IStreamMarker& operator=(const IStreamMarker&); // not implemented
};

} // namespace pdal
