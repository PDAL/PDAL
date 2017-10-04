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
#include <cstring>
#include <stack>

#include "portable_endian.hpp"
#include "pdal_util_export.hpp"

namespace pdal
{

class OStream
{
public:
    PDAL_DLL OStream() : m_stream(NULL), m_fstream(NULL)
        {}
    PDAL_DLL OStream(const std::string& filename) :
            m_stream(NULL), m_fstream(NULL)
        { open(filename); }
    PDAL_DLL OStream(std::ostream *stream) : m_stream(stream), m_fstream(NULL)
        {}
    PDAL_DLL ~OStream()
        { delete m_fstream; }

    PDAL_DLL int open(const std::string& filename)
    {
        if (m_stream)
            return -1;
        m_stream = m_fstream = new std::ofstream(filename,
            std::ios_base::out | std::ios_base::binary);
        return 0;
    }
    PDAL_DLL void close()
    {
        flush();
        delete m_fstream;
        m_fstream = NULL;
        m_stream = NULL;
    }
    PDAL_DLL bool isOpen() const
        { return (bool)m_stream; }
    PDAL_DLL void flush()
        { m_stream->flush(); }
    PDAL_DLL operator bool ()
        { return (bool)(*m_stream); }
    PDAL_DLL void seek(std::streampos pos)
        { m_stream->seekp(pos, std::ostream::beg); }
    PDAL_DLL void put(const std::string& s)
        { put(s, s.size()); }
    PDAL_DLL void put(const std::string& s, size_t len)
    {
        std::string os = s;
        os.resize(len);
        m_stream->write(os.c_str(), len);
    }
    PDAL_DLL void put(const char *c, size_t len)
        { m_stream->write(c, len); }
    PDAL_DLL void put(const unsigned char *c, size_t len)
        { m_stream->write((const char *)c, len); }
    PDAL_DLL std::streampos position() const
        { return m_stream->tellp(); }
    PDAL_DLL void pushStream(std::ostream *strm)
    {
        m_streams.push(m_stream);
        m_stream = strm;
    }
    PDAL_DLL std::ostream *popStream()
    {
        // Can't pop the last stream for now.
        if (m_streams.empty())
            return nullptr;
        std::ostream *strm = m_stream;
        m_stream = m_streams.top();
        m_streams.pop();
        return strm;
    }

protected:
    std::ostream *m_stream;
    std::ostream *m_fstream; // Dup of above to facilitate cleanup.

private:
    std::stack<std::ostream *> m_streams;
    OStream(const OStream&);
};

/// Stream wrapper for output of binary data that converts from host ordering
/// to little endian format
class OLeStream : public OStream
{
public:
    PDAL_DLL OLeStream()
    {}
    PDAL_DLL OLeStream(const std::string& filename) : OStream(filename)
    {}
    PDAL_DLL OLeStream(std::ostream *stream) : OStream(stream)
    {}

    PDAL_DLL OLeStream& operator << (uint8_t v)
    {
        m_stream->put((char)v);
        return *this;
    }

    PDAL_DLL OLeStream& operator << (int8_t v)
    {
        m_stream->put((char)v);
        return *this;
    }

    PDAL_DLL OLeStream& operator << (uint16_t v)
    {
        v = htole16(v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    PDAL_DLL OLeStream& operator << (int16_t v)
    {
        v = (int16_t)htole16((uint16_t)v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    PDAL_DLL OLeStream& operator << (uint32_t v)
    {
        v = htole32(v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    PDAL_DLL OLeStream& operator << (int32_t v)
    {
        v = (int32_t)htole32((uint32_t)v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    PDAL_DLL OLeStream& operator << (uint64_t v)
    {
        v = htole64(v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    PDAL_DLL OLeStream& operator << (int64_t v)
    {
        v = (int64_t)htole64((uint64_t)v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    PDAL_DLL OLeStream& operator << (float v)
    {
        uint32_t tmp(0);
        std::memcpy(&tmp, &v, sizeof(v));
        tmp = htole32(tmp);
        m_stream->write((char *)&tmp, sizeof(tmp));
        return *this;
    }

    PDAL_DLL OLeStream& operator << (double v)
    {
        uint64_t tmp(0);
        std::memcpy(&tmp, &v, sizeof(v));
        tmp = htole64(tmp);
        m_stream->write((char *)&tmp, sizeof(tmp));
        return *this;
    }
};


/// Stream wrapper for output of binary data that converts from host ordering
/// to big endian format
class OBeStream : public OStream
{
public:
    PDAL_DLL OBeStream()
    {}
    PDAL_DLL OBeStream(const std::string& filename) : OStream(filename)
    {}
    PDAL_DLL OBeStream(std::ostream *stream) : OStream(stream)
    {}

    PDAL_DLL OBeStream& operator << (uint8_t v)
    {
        m_stream->put((char)v);
        return *this;
    }

    PDAL_DLL OBeStream& operator << (int8_t v)
    {
        m_stream->put((char)v);
        return *this;
    }

    PDAL_DLL OBeStream& operator << (uint16_t v)
    {
        v = htobe16(v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    PDAL_DLL OBeStream& operator << (int16_t v)
    {
        v = (int16_t)htobe16((uint16_t)v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    PDAL_DLL OBeStream& operator << (uint32_t v)
    {
        v = htobe32(v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    PDAL_DLL OBeStream& operator << (int32_t v)
    {
        v = (int32_t)htobe32((uint32_t)v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    PDAL_DLL OBeStream& operator << (uint64_t v)
    {
        v = htobe64(v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    PDAL_DLL OBeStream& operator << (int64_t v)
    {
        v = (int64_t)htobe64((uint64_t)v);
        m_stream->write((char *)&v, sizeof(v));
        return *this;
    }

    PDAL_DLL OBeStream& operator << (float v)
    {
        uint32_t tmp(0);
        std::memcpy(&tmp, &v, sizeof(v));
        tmp = htobe32(tmp);
        m_stream->write((char *)&tmp, sizeof(tmp));
        return *this;
    }

    PDAL_DLL OBeStream& operator << (double v)
    {
        uint64_t tmp(0);
        std::memcpy(&tmp, &v, sizeof(v));
        tmp = htobe64(tmp);
        m_stream->write((char *)&tmp, sizeof(tmp));
        return *this;
    }
};

/// Stream position marker with rewinding/reset support.
class OStreamMarker
{
public:
    PDAL_DLL OStreamMarker(OStream& stream) : m_stream(stream)
    {
        if (m_stream.isOpen())
            m_pos = m_stream.position();
        else
            m_pos = 0;
    }

    PDAL_DLL void mark()
        { m_pos = m_stream.position(); }
    PDAL_DLL void rewind()
        { m_stream.seek(m_pos); }

private:
    std::streampos m_pos;
    OStream& m_stream;

    OStreamMarker(const OStreamMarker&);  // not implemented
    OStreamMarker& operator=(const OStreamMarker&);  // not implemented
};

} // namespace pdal
