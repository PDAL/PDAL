/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <stdint.h>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include <pdal/pdal_internal.hpp>

namespace pdal
{

typedef uint32_t PointId;
typedef uint32_t point_count_t;
typedef std::vector<std::string> StringList;

typedef union
{
    float f;
    double d;
    int8_t s8;
    int16_t s16;
    int32_t s32;
    int64_t s64;
    uint8_t u8;
    uint16_t u16;
    uint32_t u32;
    uint64_t u64;
} Everything;

struct XForm
{
public:
    XForm() : m_scale(1.0), m_autoScale(false), m_offset(0.0),
        m_autoOffset(false)
    {}

    XForm(double scale, double offset) : m_scale(scale), m_autoScale(false),
        m_offset(offset), m_autoOffset(false)
    {}

    double m_scale;
    // Whether a scale value should be determined by examining the data.
    bool m_autoScale;
    double m_offset;
    // Whether an offset value should be determined by examining the data.
    bool m_autoOffset;

    bool nonstandard() const
    {
        return m_autoScale || m_autoOffset || m_scale != 1.0 || m_offset != 0.0;
    }

    void setOffset(const std::string& sval)
    {
        if (sval == "auto")
            m_autoOffset = true;
        else
            m_offset = std::strtod(sval.c_str(), NULL);
    }

    void setScale(const std::string& sval)
    {
        if (sval == "auto")
            m_autoScale = true;
        else
            m_scale = std::strtod(sval.c_str(), NULL);
    }
};

namespace LogLevel
{
enum Enum
{
    Error = 0,
    Warning,
    Info,
    Debug,
    Debug1,
    Debug2,
    Debug3,
    Debug4,
    Debug5
};
} // namespace LogLevel

namespace Orientation
{
enum Enum
{
    PointMajor,
    DimensionMajor
};
} // namespace Orientation

class pdal_error : public std::runtime_error
{
public:
    inline pdal_error(std::string const& msg) : std::runtime_error(msg)
        {}
};

} // namespace pdal

