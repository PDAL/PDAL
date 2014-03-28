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

class IStreamMarker;

class ILeStream
{
    friend class IStreamMarker;
private:
    std::istream *m_stream;

public:
    ILeStream(const std::string& filename)
        { m_stream = new std::ifstream(filename); }

    ~ILeStream()
        { delete m_stream; }

    operator bool ()
        { return (bool)(*m_stream); }

    void skip(std::streamoff offset)
        { m_stream->seekg(offset, std::istream::cur); }

    void get(std::string& s, size_t size)
    {
        s.reserve(size);
        m_stream->width(size);
        *m_stream >> s;
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

class IStreamMarker
{
public:
    IStreamMarker(ILeStream& stream) : m_stream(stream)
    {
        m_pos = m_stream.m_stream->tellg();
    }

    void rewind()
    {
        m_stream.m_stream->seekg(m_pos);
    }

private:
    std::streampos m_pos;
    ILeStream& m_stream;
};

class OLeStream
{
private:
    std::ostream *m_stream;

public:
    OLeStream(const std::string& s)
       { m_stream = new std::ofstream(s); }

    ~OLeStream()
        { delete m_stream; }

    operator bool ()
        { return (bool)(*m_stream); }

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
