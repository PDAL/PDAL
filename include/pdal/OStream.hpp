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

#include <pdal/portable_endian.hpp>

namespace pdal
{

class OStream
{
public:
    OStream() : m_stream(NULL), m_fstream(NULL)
        {}
    OStream(const std::string& filename) : m_stream(NULL), m_fstream(NULL)
        { open(filename); }
    OStream(std::ostream *stream) : m_stream(stream), m_fstream(NULL)
        {}
    ~OStream()
        { delete m_fstream; }

    int open(const std::string& filename)
    {
        if (m_stream)
            return -1;
        m_stream = m_fstream = new std::ofstream(filename);
        return 0;
    }
    operator bool ()
        { return (bool)(*m_stream); }
    void seek(std::streampos pos)
        { m_stream->seekp(pos, std::ostream::beg); }
    void put(const std::string& s)
        { put(s, s.size()); }

    void put(const std::string& s, size_t len)
    {
        std::string os = s;
        os.resize(len);
        m_stream->write(os.c_str(), len);
    }

    void put(const char *c, size_t len)
        { m_stream->write(c, len); }

    void put(const unsigned char *c, size_t len)
        { m_stream->write((const char *)c, len); }

    std::streampos position() const
        { return m_stream->tellp(); }

protected:
    std::ostream *m_stream;
    std::ostream *m_fstream; // Dup of above to facilitate cleanup.
};

/// Stream wrapper for output of binary data that converts from host ordering
/// to little endian format
class OLeStream : public OStream
{
public:
    OLeStream()
    {}
    OLeStream(const std::string& filename) : OStream(filename)
    {}
    OLeStream(std::ostream *stream) : OStream(stream)
    {}

    OLeStream& operator << (uint8_t v)
    {
        m_stream->put((char)v);
        return *this;
    }

    OLeStream& operator << (int8_t v)
    {
        m_stream->put((char)v);
        return *this;
    }

    OLeStream& operator << (uint16_t v)
    {
        v = htole16(v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    OLeStream& operator << (int16_t v)
    { 
        v = (int16_t)htole16((uint16_t)v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    OLeStream& operator << (uint32_t v)
    {
        v = htole32(v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    OLeStream& operator << (int32_t v)
    {
        v = (int32_t)htole32((uint32_t)v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    OLeStream& operator << (uint64_t v)
    {
        v = htole64(v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    OLeStream& operator << (int64_t v)
    {
        v = (int64_t)htole64((uint64_t)v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    OLeStream& operator << (float v)
    {
        uint32_t tmp = le32toh(*(uint32_t *)(&v));
        m_stream->write((char *)&tmp, sizeof(tmp));
        return *this;
    }

    OLeStream& operator << (double v)
    {
        uint64_t tmp = le64toh(*(uint64_t *)(&v));
        m_stream->write((char *)&tmp, sizeof(tmp));
        return *this;
    }
};

} // namespace pdal
