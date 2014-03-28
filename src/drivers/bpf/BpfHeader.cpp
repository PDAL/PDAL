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

#include <pdal/BoStream.hpp>

#include "BpfHeader.hpp"

namespace pdal
{

ILeStream& operator >> (ILeStream& stream, BpfMuellerMatrix& m)
{
    for (size_t i = 0; i < (sizeof(m.m_vals) / sizeof(m.m_vals[0])); ++i)
        stream >> m.m_vals[i]; 
    return stream;
}

bool BpfHeader::read(ILeStream& stream)
{
    uint8_t dummyChar;
    std::string magic;

    stream.get(magic, 4);
    if (magic != "BPF!")
        return false;
    stream.get(m_ver, 4);
    stream >> m_len >> m_numDim >> m_interleave >> m_compression >>
        dummyChar >> m_numPts >> m_coordType >> m_coordId >> m_spacing >>
        m_xform >> m_startTime >> m_endTime >> m_dimOffset;
    return (bool)stream;
}

bool BpfDimension::read(ILeStream& stream)
{
    stream >> m_min >> m_max;
    stream.get(m_label, 32);
    return (bool)stream;
}

bool BpfUlemHeader::read(ILeStream& stream)
{
    std::string magic;
    IStreamMarker mark(stream);

    stream.get(magic, 4);
    if (magic != "ULEM")
    {
        mark.rewind();
        return false;
    }
    stream >> m_numFrames >> m_year >> m_month >> m_day >> m_lidarMode >>
        m_wavelen >> m_pulseFreq >> m_focalWidth >> m_focalHeight >>
        m_pixelPitchWidth >> m_pixelPitchHeight;
    stream.get(m_classCode, 32);    
    return (bool)stream;
}

bool BpfUlemFrame::read(ILeStream& stream)
{
    stream >> m_num >> m_roll >> m_pitch >> m_heading >> m_xform >>
        m_shortEncoder >> m_longEncoder;
    return (bool)stream;
}

bool BpfUlemFile::read(ILeStream& stream)
{
    IStreamMarker mark(stream);
    std::string magic;

    stream.get(magic, 4);
    if (magic != "FILE")
    {
        mark.rewind();
        return false;
    }
    stream >> m_len;
    stream.get(m_filename, 32);
    stream.skip(m_len);
    return (bool)stream;
}

bool BpfPolarStokesParam::read(ILeStream& stream)
{
    stream >> m_x >> m_y >> m_z >> m_a;
    return (bool)stream;
}

bool BpfPolarHeader::read(ILeStream& stream)
{
    IStreamMarker mark(stream);

    std::string magic;
    stream.get(magic, 4);
    if (magic != "POL$")
    {
        mark.rewind();
        return false;
    }

    int16_t size;
    stream >> size >> m_numFrames >> m_fpaId >> m_numXmit >> m_numRcv;
    for (decltype(m_numXmit) i = 0; i < m_numXmit; ++i)
    {
        BpfPolarStokesParam vec;
        vec.read(stream);
        m_xmitStates.push_back(vec);
    }
    for (decltype(m_numRcv) i = 0; i < m_numRcv; ++i)
    {
        BpfMuellerMatrix mat;
        stream >> mat;
        m_psaSettings.push_back(mat);
    }
    return (bool)stream;
}

bool BpfPolarFrame::read(ILeStream& stream)
{
    stream >> m_num >> m_stokesIdx;
    for (int i = 0; i < 4; ++i)
        stream >> m_stokesParam[i];
    for (int i = 0; i < 4; ++i)
        stream >> m_stokesOutParam[i];
    stream >> m_xform >> m_truncation;
    return (bool)stream;
}

} // namespace pdal

