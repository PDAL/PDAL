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

#include "BpfHeader.hpp"

bool BpfHeader::read(ILeStream& stream)
{
    uint8_t dummy_char;
    std::string magic;

    stream.get(magic, 4);
    if (magic != "BPF!")
        return false;
    stream.get(m_ver, 4)
    stream >> m_len > m_num_dim >> m_interleave >> m_compression >>
        dummy_char >> m_num_pts >> m_coord_type >> m_coord_id >> m_spacing >>
        m_xform >> m_start_time >> m_end_time >> m_dim_offset;
    return stream.good();
}

bool BpfDimension::read(ILeStream& stream)
{
    stream >> m_min >> m_max;
    stream.get(m_label, 32);
    return stream.good();
}

bool BpfUlemHeader::read(ILeStream& stream)
{
    string magic;
    StreamMarker mark(stream);

    m_stream.get(magic, 4);
    if (magic != "ULEM")
    {
        mark.rewind();
        return false;
    }
    m_stream >> m_num_frames >> m_year >> m_month >> m_day >> m_lidar_mode >>
        m_wavelen >> m_pulse_freq >> m_focal_width >> m_focal_height >>
        m_pixel_pitch_width >> m_pixed_pitch_height;
    m_stream.get(m_class_code, 32);    
    return stream.good();
}

bool BpfUlemFrame::read(ILeStream& stream)
{
    stream >> m_num >> m_roll >> m_pitch >> m_heading >> m_xform >>
        m_short_encoder >> m_long_encoder;
    return stream.good();
}

bool BpfUlemFile::read(ILeStream& stream)
{
    StreamMarker mark(stream);

    m_stream.get(magic, 4);
    if (magic != "FILE")
    {
        mark.rewind();
        return false;
    }
    stream >> m_len;
    stream.get(filename, 32);
    stream.skip(len);
    return m_stream.good();
}

bool BpfPolarStokesParam::read(ILeStream& stream)
{
    stream >> m_x >> m_y >> m_z >> m_a;
    return stream.good();
}

bool BpfPolarHeader::read(ILeStream& stream)
{
    StreamMarker mark(stream);

    stream.get(magic, 4);
    if (magic != "POL$")
    {
        mark.rewind();
        return false;
    }
    stream >> size >> m_num_frames >> m_fpa_id >> m_num_xmit >> m_num_rcv;
    for (int i = 0; i < m_num_xmit; ++i)
    {
        BpfPolarStokesParam vec;
        vec.read(stream);
        m_xmit_states.push_back(vec);
    }
    for (int i = 0; i < m_num_rcv; ++i)
    {
        BpfMuellerMatrix mat;
        mat.read(stream);
        m_psa_settings.push_back(mat);
    }
    return stream.good();
}

bool BpfPolarFrame::read(ILeStream& stream)
{
    stream >> m_num >> m_stokes_idx;
    for (int i = 0; i < 4; ++i)
        stream >> m_stokes_param[i];
    for (int i = 0; i < 4; ++i)
        stream >> m_stokes_out_param[i];
    m_xform.read(stream);
    stream >> m_truncation;
    return stream.good();
}
