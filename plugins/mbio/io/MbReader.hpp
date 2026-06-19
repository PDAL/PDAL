/******************************************************************************
* Copyright (c) 2017, Howard Butler (howard@hobu.co)
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

#include <queue>
#include <vector>

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>

extern "C"
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-register"
#include <mb_define.h>
#pragma GCC diagnostic pop
}

#include "MbFormat.hpp"

namespace pdal
{

struct BathData;

class PDAL_EXPORT MbReader : public Reader, public Streamable
{
    struct BathData
    {
        // Level 1 (always populated)
        double m_bathlon;
        double m_bathlat;
        double m_bath;
        double m_amp;
        double m_time;
        // Level 2 per-beam
        double m_travelTime;     // two-way travel time (s)
        double m_beamAngle;      // across-track takeoff angle (deg)
        double m_azimuthAngle;   // forward/azimuthal angle (deg)
        double m_acrossTrack;    // across-track distance, +starboard (m)
        double m_alongTrack;     // along-track distance, +forward (m)
        double m_beamFlag;       // raw MB-System beamflag byte
        double m_detection;      // detection type (1=amp, 2=phase, ...)
        double m_pulseType;      // pulse modulation (1=CW, 2=up, 3=down, 4=lidar)
        // Level 2 per-ping (replicated to each beam of the ping)
        double m_heading;        // deg
        double m_roll;           // deg
        double m_pitch;          // deg
        double m_heave;          // m
        double m_speed;          // km/h
        double m_sonarDepth;     // transducer depth / draft (m)
        double m_altitude;       // height above seafloor (m)
        double m_soundVelocity;  // sound speed at transducer (m/s)
        double m_transmitGain;   // dB
        double m_pulseLength;    // s
        double m_receiveGain;    // dB
        double m_pingNumber;     // sonar ping number
    };

    // Per-ping ancillary values pulled from the MBIO store at Level 2.
    struct PingAux
    {
        double heading = 0.0;
        double roll = 0.0;
        double pitch = 0.0;
        double heave = 0.0;
        double speed = 0.0;
        double sonarDepth = 0.0;
        double altitude = 0.0;
        double soundVelocity = 0.0;
        double transmitGain = 0.0;  // dB
        double pulseLength = 0.0;   // s
        double receiveGain = 0.0;   // dB
        double pingNumber = 0.0;    // sonar ping number
    };

    struct SidescanData
    {
        double m_sslon;
        double m_sslat;
        double m_ss;
        double m_time;

        SidescanData(double sslon, double sslat, double ss, double time) :
            m_sslon(sslon), m_sslat(sslat), m_ss(ss), m_time(time)
        {}
    };

    // One raw (un-georeferenced) sidescan sample, laid out as a waterfall:
    // X = signed slant range (port negative), Y = ping number.
    struct RawData
    {
        double m_x;          // signed slant range (m)
        double m_y;          // ping number (along-track)
        double m_intensity;  // raw backscatter sample
        double m_slantRange; // |slant range| (m)
        double m_time;       // ping GpsTime
        int m_sampleIndex;   // sample number (signed: port negative)
        int m_side;          // 0 = port, 1 = starboard
    };

    // One sample of a sound-velocity profile, emitted as a point in
    // datatype=svp mode: Z = -depth, SoundVelocity = speed.
    struct SvpData
    {
        double m_z;          // -depth (m, elevation convention)
        double m_velocity;   // sound speed (m/s)
        double m_time;       // profile GpsTime
        int m_profile;       // 0-based profile index within the file
    };

    enum class DataType
    {
        Multibeam,
        Sidescan,
        RawSidescan,
        Svp
    };

public:
    MbReader();
    virtual ~MbReader();
    MbReader& operator=(const MbReader&) = delete;
    MbReader(const MbReader&) = delete;
    std::string getName() const;

private:
    virtual void addDimensions(PointLayoutPtr layout);
    virtual QuickInfo inspect();
    virtual void addArgs(ProgramArgs& args);
    virtual bool processOne(PointRef& point);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual void done(PointTableRef table);
    bool loadData();
    bool extractMultibeam(int numBath, int numAmp, double time,
        const PingAux& aux, bool haveTtimes, int nbeams, bool haveExtract,
        bool haveDetects, int ndet, bool havePulses, int npulse);
    bool extractSidescan(int numSs, double time);
    bool extractRawSidescan(double time);
    bool extractSvp(int nsvp, double time);
    void recordSvp(int nsvp);
    void recordSonarMeta(void *store);

    friend std::istream& operator>>(std::istream& in,
        MbReader::DataType& mode);
    friend std::ostream& operator<<(std::ostream& in,
        const MbReader::DataType& mode);

    void *m_ctx;
    double *m_bath;
    double *m_bathlon;
    double *m_bathlat;
    double *m_amp;
    char *m_bathflag;
    double *m_ss;
    double *m_sslon;
    double *m_sslat;
    // Level-2 per-beam ancillary arrays (registered with / owned by MBIO)
    double *m_ttimes;
    double *m_angles;
    double *m_anglesForward;
    double *m_anglesNull;
    double *m_ttHeave;
    double *m_alongtrackOffset;
    double *m_bathacross;        // mb_extract across-track distance
    double *m_bathalong;         // mb_extract along-track distance
    double *m_ssacross;          // mb_extract sidescan scratch (unused output)
    double *m_ssalong;           // mb_extract sidescan scratch (unused output)
    // Scratch buffers so mb_extract() does not clobber the mb_read() arrays.
    double *m_exBath;
    double *m_exAmp;
    char *m_exFlag;
    int *m_detects;              // mb_detects detection type per beam
    int *m_pulses;               // mb_pulses pulse modulation type per beam
    std::queue<BathData> m_bathQueue;
    std::queue<SidescanData> m_ssQueue;
    std::queue<RawData> m_rawQueue;
    std::queue<SvpData> m_svpQueue;
    // Raw-sidescan sample buffers (sized per ping via mb_extract_rawssdimensions)
    std::vector<double> m_rawssPort;
    std::vector<double> m_rawssStbd;
    // Sound-velocity-profile buffers (filled by mb_extract_svp, max MB_SVP_MAX)
    std::vector<double> m_svpDepth;
    std::vector<double> m_svpVel;
    int m_svpCount;
    bool m_sonarMetaDone;
    double m_pingNumber;
    MbFormat m_format;
    double m_timegap;
    double m_speedmin;
    double m_soundspeed;
    DataType m_dataType;
    int m_level;
    bool m_keepFlagged;
    // Level-2 dimension ids
    Dimension::Id m_idTravelTime;
    Dimension::Id m_idBeamAngle;
    Dimension::Id m_idAzimuthAngle;
    Dimension::Id m_idAcrossTrack;
    Dimension::Id m_idAlongTrack;
    Dimension::Id m_idBeamFlag;
    Dimension::Id m_idDetection;
    Dimension::Id m_idHeading;
    Dimension::Id m_idRoll;
    Dimension::Id m_idPitch;
    Dimension::Id m_idHeave;
    Dimension::Id m_idSpeed;
    Dimension::Id m_idSonarDepth;
    Dimension::Id m_idAltitude;
    Dimension::Id m_idSoundVelocity;
    Dimension::Id m_idTransmitGain;
    Dimension::Id m_idPulseLength;
    Dimension::Id m_idReceiveGain;
    Dimension::Id m_idPulseType;
    Dimension::Id m_idPingNumber;
    // Raw-sidescan dimension ids
    Dimension::Id m_idSampleIndex;
    Dimension::Id m_idSide;
    Dimension::Id m_idSlantRange;
    // SVP dimension id (SoundVelocity is shared with the multibeam id above)
    Dimension::Id m_idProfileIndex;
};

} // namespace PDAL
