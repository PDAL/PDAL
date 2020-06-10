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
#include <istream>
#include <stdexcept>
#include <string>
#include <vector>

#include <iostream>

#include <pdal/util/Utils.hpp>

namespace pdal
{

typedef uint64_t PointId;
typedef uint64_t point_count_t;
typedef std::vector<std::string> StringList;
typedef std::vector<PointId> PointIdList;

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
    struct XFormComponent
    {
        XFormComponent() : m_val(0.0), m_auto(false)
        {}

        XFormComponent(double val) : m_val(val), m_auto(false)
        {}

        double m_val;
        bool m_auto;

        bool set(const std::string& sval)
        {
            if (sval == "auto")
                m_auto = true;
            else
            {
                size_t pos;
                try
                {
                    m_val = std::stod(sval, &pos);
                }
                catch (...)
                {
                    // Set error condition.
                    pos = sval.size() + 1;
                }
                if (pos != sval.size())
                {
                    m_val = 0;
                    return false;
                }
            }
            return true;
        }

        friend std::istream& operator>>(std::istream& in, XFormComponent& xfc);
        friend std::ostream& operator<<(std::ostream& in,
            const XFormComponent& xfc);
    };

    XForm() : m_scale(1.0), m_offset(0.0)
    {}

    XForm(double scale, double offset) : m_scale(scale), m_offset(offset)
    {}

    // Scale component of the transform.
    XFormComponent m_scale;
    // Offset component of the transform.
    XFormComponent m_offset;

    double toScaled(double val) const
        { return (val - m_offset.m_val) / m_scale.m_val; }

    bool nonstandard() const
    {
        return m_scale.m_auto || m_offset.m_auto ||
            m_scale.m_val != 1.0 || m_offset.m_val != 0.0;
    }
};

inline std::istream& operator>>(std::istream& in, XForm::XFormComponent& xfc)
{
    std::string sval;

    in >> sval;
    if (!xfc.set(sval))
        in.setstate(std::ios_base::failbit);
    return in;
}

inline std::ostream& operator<<(std::ostream& out,
    const XForm::XFormComponent& xfc)
{
    if (xfc.m_auto)
        out << "auto";
    else
        out << xfc.m_val;
    return out;
}

enum class LogLevel
{
    Error = 0,
    Warning,
    Info,
    Debug,
    Debug1,
    Debug2,
    Debug3,
    Debug4,
    Debug5,
    None
};

namespace ClassLabel
{
    const uint8_t CreatedNeverClassified = 0;
    const uint8_t Unclassified = 1;
    const uint8_t Ground = 2;
    const uint8_t LowVegetation = 3;
    const uint8_t MediumVegetation = 4;
    const uint8_t HighVegetation = 5;
    const uint8_t Building = 6;
    const uint8_t LowPoint = 7;
    const uint8_t ModelKeypoint = 8;
    const uint8_t Water = 9;
    const uint8_t Rail = 10;
    const uint8_t RoadSurface = 11;
    const uint8_t Overlap = 12;
    const uint8_t WireGuard = 13;
    const uint8_t WireConductor = 14;
    const uint8_t TransmissionTower = 15;
    const uint8_t WireStructureConnector = 16;
    const uint8_t BridgeDeck = 17;
    const uint8_t HighNoise = 18;
    const uint8_t OverheadStructure = 19;
    const uint8_t IgnoredGround = 20;
    const uint8_t Snow = 21;
    const uint8_t TemporalExclusion = 22;
    const uint8_t Synthetic = 32;
    const uint8_t Keypoint = 64;
    const uint8_t Withheld = 128;
}

namespace
{
    const StringList logNames { "error", "warning", "info", "debug", "debug1",
        "debug2", "debug3", "debug4", "debug5" };
}

inline std::istream& operator>>(std::istream& in, LogLevel& level)
{
    std::string sval;
    level = LogLevel::None;

    in >> sval;
    try
    {
        int val = std::stoi(sval);
        if (val >= 0 && val < (int)logNames.size())
            level = (LogLevel)val;
    }
    catch (std::exception&)
    {
        sval = Utils::tolower(sval);
        for (size_t i = 0; i < logNames.size(); ++i)
            if (logNames[i] == sval)
            {
                level = (LogLevel)i;
                break;
            }
    }
    if (level == LogLevel::None)
        in.setstate(std::ios_base::failbit);
    return in;
}

inline std::ostream& operator<<(std::ostream& out, const LogLevel& level)
{
    std::string sval("None");

    if ((size_t)level < logNames.size())
    {
        sval = logNames[(size_t)level];
        sval[0] = (char)toupper(sval[0]);   // Make "Debug", "Error", etc.
    }
    out << sval;
    return out;
}


enum class Orientation
{
    PointMajor,
    DimensionMajor
};

class pdal_error : public std::runtime_error
{
public:
    inline pdal_error(std::string const& msg) : std::runtime_error(msg)
        {}
};

enum class ExecMode
{
    Standard,
    Stream,
    PreferStream,
    None
};

} // namespace pdal

