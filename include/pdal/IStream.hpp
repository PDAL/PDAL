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

#include <pdal/portable_endian.hpp>

namespace pdal
{

class IStreamMarker;

/// Stream wrapper for input of binary data.
class IStream
{
    friend class IStreamMarker;

public:
    IStream(const std::string& filename)
        { m_stream = new std::ifstream(filename); }
    ~IStream()
        { delete m_stream; }

    operator bool ()
        { return (bool)(*m_stream); }
    void seek(std::streampos pos)
        { m_stream->seekg(pos, std::istream::beg); }
    void skip(std::streamoff offset)
        { m_stream->seekg(offset, std::istream::cur); }
    std::streampos position() const
        { return m_stream->tellg(); }
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

protected:
    std::istream *m_stream;

private:
    std::stack<std::istream *> m_streams;
};

/// Stream wrapper for input of binary data that converts from little-endian
/// to host ordering.
class ILeStream : public IStream
{
public:
    ILeStream(const std::string& filename) : IStream(filename)
    {}

    void get(std::string& s, size_t size)
    {
        // Could do this by appending to a string with a stream, but this
        // is probably fast enough for now (there's only a simple increment
        // to advance an istream iterator, which you'd have to call in a loop).
        std::unique_ptr<char[]> buf(new char[size+1]);
        m_stream->get(buf.get(), size + 1);
        s = buf.get();
    }

    void get(std::vector<char>& buf)
    {
        m_stream->read(&buf[0], buf.size());
    }

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
        v = *(float *)(&tmp);
        return *this;
    }

    ILeStream& operator >> (double& v)
    {
        m_stream->read((char *)&v, sizeof(v));
        uint64_t tmp = le64toh(*(uint64_t *)(&v));
        v = *(double *)(&tmp);
        return *this;
    }
};

/// Stream position marker with rewinding support.
class IStreamMarker
{
public:
    IStreamMarker(IStream& stream) : m_stream(stream)
    {
        m_pos = m_stream.m_stream->tellg();
    }

    void rewind()
    {
        m_stream.m_stream->seekg(m_pos);
    }

private:
    std::streampos m_pos;
    IStream& m_stream;
};

} // namespace pdal
