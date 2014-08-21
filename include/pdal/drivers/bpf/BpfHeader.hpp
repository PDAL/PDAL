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

namespace pdal
{

class ILeStream;

struct BpfMuellerMatrix
{
    double m_vals[16];
};
ILeStream& operator >> (ILeStream& stream, BpfMuellerMatrix& m);

namespace BpfFormat
{
enum Enum
{
    DimMajor,
    PointMajor,
    ByteMajor
};
}

struct BpfHeader
{
    std::string m_ver;
    int32_t m_len;
    uint8_t m_numDim;
    BpfFormat::Enum m_pointFormat;
    uint8_t m_compression;
    int32_t m_numPts;
    int32_t m_coordType;
    int32_t m_coordId;
    float m_spacing;
    BpfMuellerMatrix m_xform;
    double m_startTime;
    double m_endTime;

    bool read(ILeStream& stream);
    void dump();
};

struct BpfDimension
{
    double m_offset;
    double m_min;
    double m_max;
    std::string m_label;
    Dimension::Id::Enum m_id;

    static bool read(ILeStream& stream, std::vector<BpfDimension>& dims);
};
typedef std::vector<BpfDimension> BpfDimensionList;

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

// For now we don't actually store the file, just its name and length.
struct BpfUlemFile
{
    uint32_t m_len;
    std::string m_filename;

    bool read(ILeStream& stream);
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
