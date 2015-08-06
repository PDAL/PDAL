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

#include <iostream>

#include <boost/lexical_cast.hpp>

#include <pdal/util/IStream.hpp>
#include <pdal/util/OStream.hpp>

#include "BpfHeader.hpp"

namespace pdal
{

ILeStream& operator >> (ILeStream& stream, BpfMuellerMatrix& m)
{
    for (size_t i = 0; i < (sizeof(m.m_vals) / sizeof(m.m_vals[0])); ++i)
        stream >> m.m_vals[i];
    return stream;
}


OLeStream& operator << (OLeStream& stream, BpfMuellerMatrix& m)
{
    for (size_t i = 0; i < (sizeof(m.m_vals) / sizeof(m.m_vals[0])); ++i)
        stream << m.m_vals[i];
    return stream;
}


bool BpfHeader::read(ILeStream& stream)
{
    IStreamMarker mark(stream);
    if (!readV3(stream))
    {
        mark.rewind();
        if (!readV1(stream))
        {
            if (m_version < 1 || m_version > 3)
                m_log->get(LogLevel::Error) << "Unsupported BPF version = " <<
                    m_version << ".\n";
            else
                m_log->get(LogLevel::Error) << "Couldn't read BPF header.\n";
            return false;
        }
    }
    return true;
}

bool BpfHeader::readV3(ILeStream& stream)
{
    m_log->get(LogLevel::Debug) << "BPF: Reading V3\n";

    uint8_t dummyChar;
    uint8_t interleave;
    std::string magic;

    stream.get(magic, 4);
    if (magic != "BPF!")
        return false;

    stream.get(m_ver, 4);
    m_version = boost::lexical_cast<int32_t>(m_ver);

    uint8_t numDim;
    stream >> m_len >> numDim >> interleave >> m_compression >>
        dummyChar >> m_numPts >> m_coordType >> m_coordId >> m_spacing >>
        m_xform >> m_startTime >> m_endTime;
    m_numDim = (int32_t)numDim;

    switch (interleave)
    {
    case 0:
        m_pointFormat = BpfFormat::DimMajor;
        break;
    case 1:
        m_pointFormat = BpfFormat::PointMajor;
        break;
    case 2:
        m_pointFormat = BpfFormat::ByteMajor;
        break;
    default:
        throw "Invalid BPF file: unknown interleave type.";
    }
    return (bool)stream;
}


bool BpfHeader::readV1(ILeStream& stream)
{
    m_log->get(LogLevel::Debug) << "BPF: Reading V1\n";

    stream >> m_len;
    stream >> m_version;

    stream >> m_numPts >> m_numDim >> m_coordType >> m_coordId >> m_spacing;

    if (m_version == 1)
        m_pointFormat = BpfFormat::DimMajor;
    else if (m_version == 2)
        m_pointFormat = BpfFormat::PointMajor;
    else
        return false;

    // Dimensions should include X, Y, and Z
    m_numDim += 3;

    BpfDimension xDim;
    BpfDimension yDim;
    BpfDimension zDim;

    xDim.m_label = "X";
    yDim.m_label = "Y";
    zDim.m_label = "Z";

    stream >> xDim.m_offset >> yDim.m_offset >> zDim.m_offset;
    stream >> xDim.m_min >> yDim.m_min >> zDim.m_min;
    stream >> xDim.m_max >> yDim.m_max >> zDim.m_max;

    m_staticDims.resize(3);
    m_staticDims[0] = xDim;
    m_staticDims[1] = yDim;
    m_staticDims[2] = zDim;
    return (bool)stream;
}


bool BpfHeader::write(OLeStream& stream)
{
    uint8_t dummyChar = 0;
    uint8_t numDim;

    if (!Utils::numericCast(m_numDim, numDim))
        throw pdal_error("Can't write a BPF file of more than 255 dimensions.");

    stream.put("BPF!");
    stream.put("0003");

    stream << m_len << numDim << (uint8_t)m_pointFormat << m_compression <<
        dummyChar << m_numPts << m_coordType << m_coordId << m_spacing <<
        m_xform << m_startTime << m_endTime;
    return (bool)stream;
}


bool BpfHeader::readDimensions(ILeStream& stream, BpfDimensionList& dims)
{
    size_t staticCnt = m_staticDims.size();

    dims.resize(m_numDim);

    if (static_cast<std::size_t>(m_numDim) < staticCnt)
    {
        m_log->get(LogLevel::Error) << "BPF dimension range looks bad.\n";
        m_log->get(LogLevel::Error) <<
            "BPF: num dims: " << m_numDim << "\n" <<
            "BPF: static count: " << staticCnt << "\n";

        m_log->get(LogLevel::Error) << "Dims:\n";
        for (auto d : dims)
            m_log->get(LogLevel::Error) << "\t" << d.m_label << "\n";

        m_log->get(LogLevel::Error) << "Static:\n";
        for (auto d : m_staticDims)
            m_log->get(LogLevel::Error) << "\t" << d.m_label << "\n";
    }

    for (size_t d = 0; d < staticCnt; d++)
        dims.at(d) = m_staticDims[d];
    if (!BpfDimension::read(stream, dims, staticCnt))
        return false;

    // Verify that we have an X, Y and Z, so that we don't have to worry
    // about it later.
    bool x = false;
    bool y = false;
    bool z = false;
    for (auto d : dims)
    {
        if (d.m_label == "X")
            x = true;
        if (d.m_label == "Y")
            y = true;
        if (d.m_label == "Z")
            z = true;
    }
    if (!x || !y || !z)
        throw pdal_error("BPF file missing at least one of X, Y or Z "
            "dimensions.");
    return true;
}


// This just exists for symmetry.
void BpfHeader::writeDimensions(OLeStream& stream, BpfDimensionList& dims)
{
    BpfDimension::write(stream, dims);
}


void BpfHeader::dump()
{
    using namespace std;

    cerr << "Length: " << m_len << "!\n";
    cerr << "Diemsions: " << (int)m_numDim << "!\n";
    cerr << "Interleave: " << (int)m_pointFormat << "!\n";
    cerr << "Compression: " << (int)m_compression << "!\n";
    cerr << "Point count: " << m_numPts << "!\n";
    cerr << "Coordinate type: " << m_coordType << "!\n";
    cerr << "Coordinate ID: " << m_coordId << "!\n";
    cerr << "Spacing: " << m_spacing << "!\n";
    cerr << "Start time: " << m_startTime << "!\n";
    cerr << "End time: " << m_endTime << "!\n";
}


bool BpfDimension::read(ILeStream& stream, BpfDimensionList& dims, size_t start)
{
    for (size_t d = start; d < dims.size(); ++d)
        stream >> dims[d].m_offset;
    for (size_t d = start; d < dims.size(); ++d)
        stream >> dims[d].m_min;
    for (size_t d = start; d < dims.size(); ++d)
        stream >> dims[d].m_max;
    for (size_t d = start; d < dims.size(); ++d)
        stream.get(dims[d].m_label, 32);
    return (bool)stream;
}


bool BpfDimension::write(OLeStream& stream, BpfDimensionList& dims)
{
    for (size_t d = 0; d < dims.size(); ++d)
        stream << dims[d].m_offset;
    for (size_t d = 0; d < dims.size(); ++d)
        stream << dims[d].m_min;
    for (size_t d = 0; d < dims.size(); ++d)
        stream << dims[d].m_max;
    for (size_t d = 0; d < dims.size(); ++d)
        stream.put(dims[d].m_label, 32);
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
    Utils::trimTrailing(m_filename);
    m_buf.resize(m_len);
    stream.get(m_buf);

    return (bool)stream;
}

bool BpfUlemFile::write(OLeStream& stream)
{
    stream.put("FILE", 4);
    stream << m_len;
    stream.put(m_filename, 32);

    std::ifstream in(m_filespec);
    uint32_t len = m_len;

    const uint32_t MAX_BLOCKSIZE = 1000000;
    char buf[MAX_BLOCKSIZE];
    while (len)
    {
        uint32_t blocksize = std::min(MAX_BLOCKSIZE, len);
        in.read(buf, blocksize);
        stream.put(buf, blocksize);
        len -= blocksize;
    }
    return true;
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

