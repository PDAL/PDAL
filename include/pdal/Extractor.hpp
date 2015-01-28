/******************************************************************************
* Copyright (c) 2014, Hobu Inc., hobu@hobu.co
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

#include <pdal/portable_endian.hpp>

namespace pdal
{

/// Stream wrapper for input of binary data from a buffer.
class PDAL_DLL Extractor
{
public:
    Extractor(const char *buf, std::size_t size) : m_eback(buf),
        m_egptr(buf + size), m_gptr(buf)
    {}
    
public:
    operator bool ()
        { return good(); }
    void seek(std::size_t pos)
        { m_gptr = m_eback + pos; }
    void skip(std::size_t cnt)
        { m_gptr += cnt; }
    size_t position() const
        { return m_gptr - m_eback; }
    bool good() const
        { return m_gptr < m_egptr; }

    void get(std::string& s, size_t size)
    {
        s = string(m_gptr, size);
        m_gptr += size;
    }

    void get(std::vector<char>& buf)
    {
        memcpy((char *)buf.data(), m_gptr, buf.size());
        m_gptr += buf.size();
    }

    void get(std::vector<unsigned char>& buf)
    {
        memcpy((char *)buf.data(), m_gptr, buf.size());
        m_gptr += buf.size();
    }

    void get(char *buf, size_t size)
    {
        memcpy(buf, m_gptr, size);
        m_gptr += size;
    }

    void get(unsigned char *buf, size_t size)
    {
        memcpy(buf, m_gptr, size);
        m_gptr += size;
    }

protected:
    // Beginning of the buffer (names come from std::streambuf)
    const char *m_eback;
    // End of the buffer.
    const char *m_egptr;
    // Current get location.
    const char *m_gptr;
};

/// Stream wrapper for input of binary data that converts from little-endian
/// to host ordering.
class PDAL_DLL LeExtractor : public Extractor
{
public:
    LeExtractor(const char *buf, std::size_t size) : Extractor(buf, size)
    {}

    LeExtractor& operator >> (uint8_t& v)
    {
        v = *(uint8_t *)m_gptr++;
        return *this;
    }

    LeExtractor& operator >> (int8_t& v)
    {
        v = *(int8_t *)m_gptr++;
        return *this;
    }

    LeExtractor& operator >> (uint16_t& v)
    {
        memcpy(&v, m_gptr, sizeof(v));
        v = le16toh(v);
        m_gptr += sizeof(v);
        return *this;
    }

    LeExtractor& operator >> (int16_t& v)
    {
        memcpy(&v, m_gptr, sizeof(v));
        v = (int16_t)le16toh((uint16_t)v);
        m_gptr += sizeof(v);
        return *this;
    }

    LeExtractor& operator >> (uint32_t& v)
    {
        memcpy(&v, m_gptr, sizeof(v));
        v = le32toh(v);
        m_gptr += sizeof(v);
        return *this;
    }

    LeExtractor& operator >> (int32_t& v)
    {
        memcpy(&v, m_gptr, sizeof(v));
        v = (int32_t)le32toh((uint32_t)v);
        m_gptr += sizeof(v);
        return *this;
    }

    LeExtractor& operator >> (uint64_t& v)
    {
        memcpy(&v, m_gptr, sizeof(v));
        v = le64toh(v);
        m_gptr += sizeof(v);
        return *this;
    }

    LeExtractor& operator >> (int64_t& v)
    {
        memcpy(&v, m_gptr, sizeof(v));
        v = (int64_t)le64toh((uint64_t)v);
        m_gptr += sizeof(v);
        return *this;
    }

    LeExtractor& operator >> (float& v)
    {
        memcpy(&v, m_gptr, sizeof(v));
        uint32_t tmp = le32toh(*(uint32_t *)(&v));
        std::memcpy(&v, &tmp, sizeof(tmp));
        m_gptr += sizeof(v);
        return *this;
    }

    LeExtractor& operator >> (double& v)
    {
        memcpy(&v, m_gptr, sizeof(v));
        uint64_t tmp = le64toh(*(uint64_t *)(&v));
        std::memcpy(&v, &tmp, sizeof(tmp));
        m_gptr += sizeof(v);
        return *this;
    }
};

} // namespace pdal
