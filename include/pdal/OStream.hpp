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

/// Stream wrapper for output of binary data that converts from host ordering
/// to little endian format
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
