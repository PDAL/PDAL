/******************************************************************************
* Copyright (c) 2014, Howard Butler, hobu.inc@gmail.com
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
#include <string>
#include <vector>

#include <pdal/Dimension.hpp>
#include <pdal/Log.hpp>
#include <pdal/Metadata.hpp>

namespace pdal
{

class ILeStream;
class OLeStream;

struct BpfMuellerMatrix
{
    BpfMuellerMatrix()
    {
        static const double vals[] = {1.0, 0.0, 0.0, 0.0,
                         0.0, 1.0, 0.0, 0.0,
                         0.0, 0.0, 1.0, 0.0,
                         0.0, 0.0, 0.0, 1.0};
        memcpy(m_vals, vals, sizeof(vals));
    }

    double m_vals[16];

    void dump()
    {
        for (size_t i = 0; i < 4; ++i)
            std::cerr << m_vals[i] << '\t';
        std::cerr << "\n";
        for (size_t i = 4; i < 8; ++i)
            std::cerr << m_vals[i] << '\t';
        std::cerr << "\n";
        for (size_t i = 8; i < 12; ++i)
            std::cerr << m_vals[i] << '\t';
        std::cerr << "\n";
        for (size_t i = 12; i < 16; ++i)
            std::cerr << m_vals[i] << '\t';
        std::cerr << "\n\n";
        
    }

    void apply(double& x, double& y, double& z)
    {
        double w = x * m_vals[12] + y * m_vals[13] + z * m_vals[14] +
            m_vals[15];

        x = (x * m_vals[0] + y * m_vals[1] + z * m_vals[2] + m_vals[3]) / w;
        y = (x * m_vals[4] + y * m_vals[5] + z * m_vals[6] + m_vals[7]) / w;
        z = (x * m_vals[8] + y * m_vals[9] + z * m_vals[10] + m_vals[11]) / w;
    }
};
ILeStream& operator >> (ILeStream& stream, BpfMuellerMatrix& m);
OLeStream& operator << (OLeStream& stream, BpfMuellerMatrix& m);

namespace BpfFormat
{
enum Enum
{
    DimMajor,
    PointMajor,
    ByteMajor
};
}

namespace BpfCoordType
{
enum Enum
{
    None,
    UTM,
    TCR,
    ENU
};
}

namespace BpfCompression
{
enum Enum
{
    None,
    QuickLZ,
    FastLZ,
    Zlib
};
}

struct BpfDimension
{
    BpfDimension() : m_offset(0.0),
        m_min((std::numeric_limits<double>::max)()),
        m_max(std::numeric_limits<double>::lowest()),
        m_id(Dimension::Id::Unknown)
    {}

    double m_offset;
    double m_min;
    double m_max;
    std::string m_label;
    Dimension::Id::Enum m_id;

    static bool read(ILeStream& stream, std::vector<BpfDimension>& dims,
        size_t start);
    static bool write(OLeStream& stream, std::vector<BpfDimension>& dims);
};
typedef std::vector<BpfDimension> BpfDimensionList;

struct BpfHeader
{
    BpfHeader() : m_version(0), m_len(176), m_numDim(0),
        m_compression(BpfCompression::None), m_numPts(0),
        m_coordType(BpfCoordType::None), m_coordId(0), m_spacing(0.0),
        m_startTime(0.0), m_endTime(0.0)
    {}

    int32_t m_version;
    std::string m_ver;
    int32_t m_len;
    int32_t m_numDim;
    BpfFormat::Enum m_pointFormat;
    uint8_t m_compression;
    int32_t m_numPts;
    int32_t m_coordType;
    int32_t m_coordId;
    float m_spacing;
    BpfMuellerMatrix m_xform;
    double m_startTime;
    double m_endTime;
    std::vector<BpfDimension> m_staticDims;
    LogPtr m_log;

    void setLog(const LogPtr& log)
         { m_log = log; }
    bool read(ILeStream& stream);
    bool write(OLeStream& stream);
    bool readV3(ILeStream& stream);
    bool readV1(ILeStream& stream);
    bool readDimensions(ILeStream& stream, std::vector<BpfDimension>& dims);
    void writeDimensions(OLeStream& stream, std::vector<BpfDimension>& dims);
    void dump();
};

struct BpfUlemHeader
{
    uint32_t m_numFrames;
    uint16_t m_year;
    uint8_t m_month;
    uint8_t m_day;
    uint16_t m_lidarMode;
    uint16_t m_wavelen;  // In nm.
    uint16_t m_pulseFreq;  // In Hz.
    uint16_t m_focalWidth;
    uint16_t m_focalHeight;
    float m_pixelPitchWidth;
    float m_pixelPitchHeight;
    std::string m_classCode;

    bool read(ILeStream& stream);
};

struct BpfUlemFrame
{
    int32_t m_num;
    double m_roll; //x
    double m_pitch; //y
    double m_heading; //z
    BpfMuellerMatrix m_xform;
    int16_t m_shortEncoder;
    int16_t m_longEncoder;

    bool read(ILeStream& stream);
};

struct BpfUlemFile
{
    uint32_t m_len;
    std::string m_filename;
    std::vector<char> m_buf;
    std::string m_filespec;

    bool read(ILeStream& stream);
    bool write(OLeStream& stream);
};

struct BpfPolarStokesParam
{
    float m_x;
    float m_y;
    float m_z;
    float m_a;

    bool read(ILeStream& stream);
};

struct BpfPolarHeader
{
    uint32_t m_numFrames;
    uint16_t m_fpaId;
    uint32_t m_numXmit;
    uint32_t m_numRcv;
    std::vector<BpfPolarStokesParam> m_xmitStates;
    std::vector<BpfMuellerMatrix> m_psaSettings;

    bool read(ILeStream& stream);
};

struct BpfPolarFrame
{
public:
    uint32_t m_num;
    int16_t m_stokesIdx;
    float m_stokesParam[4];
    float m_stokesOutParam[4];
    BpfMuellerMatrix m_xform;
    int16_t m_truncation;

    bool read(ILeStream& stream);
};

} //namespace pdal
