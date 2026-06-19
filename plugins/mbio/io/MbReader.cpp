/******************************************************************************
* Copyright (c) 2017, Howard Butler (hobu@hob.co)
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

#include "MbReader.hpp"
#include "MbError.hpp"

#include <pdal/util/ProgramArgs.hpp>
#include <pdal/DimUtil.hpp>
#include <pdal/Metadata.hpp>

#include <mb_status.h>

namespace pdal
{

static PluginInfo const s_info
{
    "readers.mbio",
    "MBSystem Reader",
    "https://pdal.org/stages/readers.mbio.html"
};

CREATE_SHARED_STAGE(MbReader, s_info)

std::string MbReader::getName() const { return s_info.name; }

MbReader::MbReader() : m_bath(nullptr), m_bathlon(nullptr),
    m_bathlat(nullptr), m_amp(nullptr), m_bathflag(nullptr), m_ss(nullptr),
    m_sslon(nullptr), m_sslat(nullptr), m_ttimes(nullptr), m_angles(nullptr),
    m_anglesForward(nullptr), m_anglesNull(nullptr), m_ttHeave(nullptr),
    m_alongtrackOffset(nullptr), m_bathacross(nullptr), m_bathalong(nullptr),
    m_ssacross(nullptr), m_ssalong(nullptr), m_exBath(nullptr),
    m_exAmp(nullptr), m_exFlag(nullptr), m_detects(nullptr), m_pulses(nullptr),
    m_svpCount(0), m_sonarMetaDone(false), m_pingNumber(0.0)
{}


MbReader::~MbReader()
{}


void MbReader::addArgs(ProgramArgs& args)
{
    args.add("format", "Name or number of MBIO data format",
        m_format).setPositional();
    args.add("timegap", "Maximum time between records.", m_timegap, 1.0);
    args.add("speedmin", "Minimum vehicle speed for data to be valid",
        m_speedmin);
    args.add("datatype", "Multibeam (default), sidescan, rawsidescan, or svp "
        "(sound-velocity profiles as points).", m_dataType,
        DataType::Multibeam);
    args.add("soundspeed", "Reference sound speed (m/s) used to convert raw "
        "sidescan sample times to slant range (rawsidescan only).",
        m_soundspeed, 1500.0);
    args.add("level", "MBIO read level: 1 = basic (X, Y, Z, Amplitude); "
        "2 = extended (adds per-beam travel time & angles, across/along-track, "
        "beam flag, detection type, and per-ping attitude, heading, speed, "
        "sonar depth, altitude and sound velocity).", m_level, 2);
    args.add("keepflagged", "Keep beams flagged by editing/filtering (null "
        "beams are always dropped); their BeamFlag value is preserved.",
        m_keepFlagged, false);
}


void MbReader::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;

    std::vector<Dimension::Id> dims { Id::X, Id::Y, Id::Z, Id::GpsTime };

    layout->registerDims(dims);
    if (m_dataType == DataType::Multibeam)
        layout->registerDim(Id::Amplitude);
    else if (m_dataType == DataType::Sidescan ||
             m_dataType == DataType::RawSidescan)
        layout->registerDim(Id::Intensity, Type::Double);

    // Level 2 exposes the richer per-beam / per-ping information that MBIO
    // carries in its internal store. These are proprietary dimensions (created
    // if PDAL has no standard equivalent).
    if (m_level >= 2 && m_dataType == DataType::Multibeam)
    {
        m_idTravelTime   = layout->registerOrAssignDim("TravelTime", Type::Double);
        m_idBeamAngle    = layout->registerOrAssignDim("BeamAngle", Type::Double);
        m_idAzimuthAngle = layout->registerOrAssignDim("AzimuthAngle", Type::Double);
        m_idAcrossTrack  = layout->registerOrAssignDim("AcrossTrack", Type::Double);
        m_idAlongTrack   = layout->registerOrAssignDim("AlongTrack", Type::Double);
        m_idBeamFlag     = layout->registerOrAssignDim("BeamFlag", Type::Unsigned8);
        m_idDetection    = layout->registerOrAssignDim("Detection", Type::Unsigned8);
        m_idHeading      = layout->registerOrAssignDim("Heading", Type::Double);
        m_idRoll         = layout->registerOrAssignDim("Roll", Type::Double);
        m_idPitch        = layout->registerOrAssignDim("Pitch", Type::Double);
        m_idHeave        = layout->registerOrAssignDim("Heave", Type::Double);
        m_idSpeed        = layout->registerOrAssignDim("Speed", Type::Double);
        m_idSonarDepth   = layout->registerOrAssignDim("SonarDepth", Type::Double);
        m_idAltitude     = layout->registerOrAssignDim("Altitude", Type::Double);
        m_idSoundVelocity= layout->registerOrAssignDim("SoundVelocity", Type::Double);
        m_idTransmitGain = layout->registerOrAssignDim("TransmitGain", Type::Double);
        m_idPulseLength  = layout->registerOrAssignDim("PulseLength", Type::Double);
        m_idReceiveGain  = layout->registerOrAssignDim("ReceiveGain", Type::Double);
        m_idPulseType    = layout->registerOrAssignDim("PulseType", Type::Unsigned8);
        m_idPingNumber   = layout->registerOrAssignDim("PingNumber", Type::Unsigned32);
    }

    if (m_dataType == DataType::RawSidescan)
    {
        m_idSampleIndex = layout->registerOrAssignDim("SampleIndex", Type::Signed32);
        m_idSide        = layout->registerOrAssignDim("Side", Type::Unsigned8);
        m_idSlantRange  = layout->registerOrAssignDim("SlantRange", Type::Double);
    }

    if (m_dataType == DataType::Svp)
    {
        m_idSoundVelocity = layout->registerOrAssignDim("SoundVelocity", Type::Double);
        m_idProfileIndex  = layout->registerOrAssignDim("ProfileIndex", Type::Unsigned16);
    }
}


void MbReader::ready(PointTableRef table)
{
    int verbose = 0;
    int pings = 0;  // Perhaps an argument for this?
    int lonflip = 0; // Longitude -180 -> 180
    double bounds[4] { -180, 180, -90, 90 };
    int btime_i[7] { 1962, 2, 21, 10, 30, 0, 0 };
    int etime_i[7] { 2062, 2, 21, 10, 30, 0, 0 };
    char *mbio_ptr;
    double btime_d;
    double etime_d;
    int beams_bath;
    int beams_amp;
    int pixels_ss;
    int error;

    mb_read_init(verbose, const_cast<char *>(m_filename.data()),
        (int)m_format, pings, lonflip, bounds, btime_i, etime_i,
        m_speedmin, m_timegap, &m_ctx, &btime_d, &etime_d,
        &beams_bath, &beams_amp, &pixels_ss, &error);
    if (error > 0)
        throwError("Can't initialize mb-system reader: " +
            MbError::text(error));

    mb_register_array(verbose, m_ctx, 1, sizeof(double),
        (void **)&m_bath, &error);
    mb_register_array(verbose, m_ctx, 1, sizeof(double),
        (void **)&m_bathlon, &error);
    mb_register_array(verbose, m_ctx, 1, sizeof(double),
        (void **)&m_bathlat, &error);
    mb_register_array(verbose, m_ctx, 1, sizeof(char),
        (void **)&m_bathflag, &error);
    mb_register_array(verbose, m_ctx, 2, sizeof(double),
        (void **)&m_amp, &error);
    mb_register_array(verbose, m_ctx, 3, sizeof(double),
        (void **)&m_ss, &error);
    mb_register_array(verbose, m_ctx, 3, sizeof(double),
        (void **)&m_sslon, &error);
    mb_register_array(verbose, m_ctx, 3, sizeof(double),
        (void **)&m_sslat, &error);

    // Level-2 per-beam arrays for mb_ttimes() (bathymetry-sized, type 1).
    if (m_level >= 2 && m_dataType == DataType::Multibeam)
    {
        mb_register_array(verbose, m_ctx, 1, sizeof(double),
            (void **)&m_ttimes, &error);
        mb_register_array(verbose, m_ctx, 1, sizeof(double),
            (void **)&m_angles, &error);
        mb_register_array(verbose, m_ctx, 1, sizeof(double),
            (void **)&m_anglesForward, &error);
        mb_register_array(verbose, m_ctx, 1, sizeof(double),
            (void **)&m_anglesNull, &error);
        mb_register_array(verbose, m_ctx, 1, sizeof(double),
            (void **)&m_ttHeave, &error);
        mb_register_array(verbose, m_ctx, 1, sizeof(double),
            (void **)&m_alongtrackOffset, &error);
        // mb_extract across/along-track (bathymetry-sized) + sidescan scratch.
        mb_register_array(verbose, m_ctx, 1, sizeof(double),
            (void **)&m_bathacross, &error);
        mb_register_array(verbose, m_ctx, 1, sizeof(double),
            (void **)&m_bathalong, &error);
        mb_register_array(verbose, m_ctx, 3, sizeof(double),
            (void **)&m_ssacross, &error);
        mb_register_array(verbose, m_ctx, 3, sizeof(double),
            (void **)&m_ssalong, &error);
        // Scratch outputs for mb_extract (kept separate from the mb_read arrays).
        mb_register_array(verbose, m_ctx, 1, sizeof(double),
            (void **)&m_exBath, &error);
        mb_register_array(verbose, m_ctx, 2, sizeof(double),
            (void **)&m_exAmp, &error);
        mb_register_array(verbose, m_ctx, 1, sizeof(char),
            (void **)&m_exFlag, &error);
        // mb_detects / mb_pulses per-beam type (bathymetry-sized int arrays).
        mb_register_array(verbose, m_ctx, 1, sizeof(int),
            (void **)&m_detects, &error);
        mb_register_array(verbose, m_ctx, 1, sizeof(int),
            (void **)&m_pulses, &error);
    }

    // Buffers for sound-velocity profiles (captured as metadata, any level).
    m_svpDepth.resize(MB_SVP_MAX);
    m_svpVel.resize(MB_SVP_MAX);

    // Static format metadata (available without reading any record).
    int fmt = (int)m_format;
    char description[MB_DESCRIPTION_LENGTH] = { 0 };
    int derr = MB_ERROR_NO_ERROR;
    mb_format_description(verbose, &fmt, description, &derr);
    MetadataNode meta = getMetadata();
    meta.add("format", (int)m_format);
    meta.add("format_description", std::string(description));
    meta.add("mbio_level", m_level);
}

bool MbReader::loadData()
{
    int verbose = 0;
    int kind;
    int pings;
    int pingTime[7];
    double pingTimeT;
    double lon;
    double lat;
    double speed;
    double heading;
    double distance;
    double altitude;
    double sonarDepth;
    int numBath;
    int numAmp;
    int numSs;
    char comment[MB_COMMENT_MAXLINE];
    int error;

    while (true)
    {
        int status = mb_read(verbose, m_ctx, &kind, &pings, pingTime,
            &pingTimeT, &lon, &lat, &speed, &heading, &distance, &altitude,
            &sonarDepth, &numBath, &numAmp, &numSs, m_bathflag, m_bath,
            m_amp, m_bathlon, m_bathlat, m_ss, m_sslon, m_sslat, comment,
            &error);

        // mb_read() returns the record time as Unix epoch seconds (UTC) in
        // pingTimeT — use it directly. (The old homebrew timeconvert(pingTime)
        // mis-handled MB-System's full-year and 1-based month, putting every
        // timestamp ~1900 years and one month off.)
        double gpsTime = pingTimeT;

        if (error > 0)
        {
            if (error != MB_ERROR_EOF)
                throwError("Error reading data: " + MbError::text(error));
            return false;
        }

        // Sound-velocity-profile records arrive interspersed with survey pings.
        // In datatype=svp they ARE the point data; otherwise they're captured
        // as stage metadata.
        if (kind == MB_DATA_VELOCITY_PROFILE)
        {
            void *store = nullptr;
            int se = MB_ERROR_NO_ERROR;
            if (mb_get_store(0, m_ctx, &store, &se) == MB_SUCCESS && store)
            {
                int ks = 0; int nsvp = 0; int es = MB_ERROR_NO_ERROR;
                if (mb_extract_svp(0, m_ctx, store, &ks, &nsvp,
                        m_svpDepth.data(), m_svpVel.data(), &es) == MB_SUCCESS &&
                        nsvp > 0)
                {
                    if (m_dataType == DataType::Svp)
                    {
                        if (extractSvp(nsvp, gpsTime))
                            break;
                    }
                    else
                        recordSvp(nsvp);
                }
            }
            continue;
        }

        // datatype=svp emits only sound-velocity profiles; skip everything else.
        if (m_dataType == DataType::Svp)
            continue;

        if (status == MB_SUCCESS && kind == 1)
        {
            // Level 2: pull ancillary data for this ping from the MBIO store.
            // mb_read() already gave us per-beam lon/lat/depth/amplitude; the
            // store holds the richer information that level-1 i/o discards.
            PingAux aux;
            bool haveTtimes = false;
            bool haveExtract = false;
            bool haveDetects = false;
            bool havePulses = false;
            int nbeams = 0;
            int ndet = 0;
            int npulse = 0;
            if (m_level >= 2 && m_dataType == DataType::Multibeam)
            {
                void *store = nullptr;
                int serr = MB_ERROR_NO_ERROR;
                if (mb_get_store(0, m_ctx, &store, &serr) == MB_SUCCESS && store)
                {
                    // Navigation + attitude (heading, speed, draft, roll,
                    // pitch, heave).
                    int k; int ti[7]; double td;
                    double nlon, nlat, nspeed, nheading;
                    double ndraft, nroll, npitch, nheave;
                    int e = MB_ERROR_NO_ERROR;
                    if (mb_extract_nav(0, m_ctx, store, &k, ti, &td, &nlon,
                            &nlat, &nspeed, &nheading, &ndraft, &nroll, &npitch,
                            &nheave, &e) == MB_SUCCESS)
                    {
                        aux.heading = nheading;
                        aux.speed = nspeed;
                        aux.sonarDepth = ndraft;
                        aux.roll = nroll;
                        aux.pitch = npitch;
                        aux.heave = nheave;
                    }

                    // Sonar transducer depth and altitude above the seafloor.
                    int ka; double tdepth = 0.0; double alt = 0.0;
                    int ea = MB_ERROR_NO_ERROR;
                    if (mb_extract_altitude(0, m_ctx, store, &ka, &tdepth, &alt,
                            &ea) == MB_SUCCESS)
                    {
                        aux.altitude = alt;
                        if (tdepth != 0.0)
                            aux.sonarDepth = tdepth;
                    }

                    // Per-beam travel times and takeoff / azimuthal angles.
                    int kt; double tdraft = 0.0; double ssv = 0.0;
                    int et = MB_ERROR_NO_ERROR;
                    if (mb_ttimes(0, m_ctx, store, &kt, &nbeams, m_ttimes,
                            m_angles, m_anglesForward, m_anglesNull, m_ttHeave,
                            m_alongtrackOffset, &tdraft, &ssv, &et) ==
                            MB_SUCCESS)
                    {
                        haveTtimes = true;
                        aux.soundVelocity = ssv;
                    }

                    // Across/along-track beam distances (acoustic geometry).
                    int kx; int tix[7]; double tdx;
                    double xlon, xlat, xspeed, xheading;
                    int xnb, xna, xns;
                    char xcomment[MB_COMMENT_MAXLINE];
                    int ex = MB_ERROR_NO_ERROR;
                    if (mb_extract(0, m_ctx, store, &kx, tix, &tdx, &xlon, &xlat,
                            &xspeed, &xheading, &xnb, &xna, &xns, m_exFlag,
                            m_exBath, m_exAmp, m_bathacross, m_bathalong, m_ss,
                            m_ssacross, m_ssalong, xcomment, &ex) == MB_SUCCESS)
                        haveExtract = true;

                    // Per-beam bottom-detection type (amplitude / phase / ...).
                    int kdt; int edt = MB_ERROR_NO_ERROR;
                    if (mb_detects(0, m_ctx, store, &kdt, &ndet, m_detects,
                            &edt) == MB_SUCCESS)
                        haveDetects = true;

                    // Per-ping transmit/receive gains and pulse length.
                    int kg; double tgain = 0.0, plen = 0.0, rgain = 0.0;
                    int eg = MB_ERROR_NO_ERROR;
                    if (mb_gains(0, m_ctx, store, &kg, &tgain, &plen, &rgain,
                            &eg) == MB_SUCCESS)
                    {
                        aux.transmitGain = tgain;
                        aux.pulseLength = plen;
                        aux.receiveGain = rgain;
                    }

                    // Per-beam pulse modulation type (CW / chirp / lidar).
                    int kp; int ep = MB_ERROR_NO_ERROR;
                    if (mb_pulses(0, m_ctx, store, &kp, &npulse, m_pulses,
                            &ep) == MB_SUCCESS)
                        havePulses = true;

                    // Sonar ping number (from the mbio descriptor, per ping).
                    unsigned int pn = 0; int epn = MB_ERROR_NO_ERROR;
                    if (mb_pingnumber(0, m_ctx, &pn, &epn) == MB_SUCCESS)
                        aux.pingNumber = (double)pn;

                    // Sonar / sidescan type — static, captured once as metadata.
                    if (!m_sonarMetaDone)
                    {
                        recordSonarMeta(store);
                        m_sonarMetaDone = true;
                    }
                }
            }

            bool ok(false);
            if (m_dataType == DataType::Multibeam)
                ok = extractMultibeam(numBath, numAmp, gpsTime, aux,
                    haveTtimes, nbeams, haveExtract, haveDetects, ndet,
                    havePulses, npulse);
            else if (m_dataType == DataType::Sidescan)
                ok = extractSidescan(numSs, gpsTime);
            else
                ok = extractRawSidescan(gpsTime);
            if (ok)
                break;
        }
    }
    return true;
}


bool MbReader::extractMultibeam(int numBath, int numAmp, double gpsTime,
    const PingAux& aux, bool haveTtimes, int nbeams, bool haveExtract,
    bool haveDetects, int ndet, bool havePulses, int npulse)
{
    for (size_t i = 0; i < (size_t)numBath; ++i)
    {
        const char flag = m_bathflag[i];
        // Null beams carry no sounding and are always dropped. Flagged beams
        // (edited/filtered) are dropped unless the user asked to keep them.
        if (flag == MB_FLAG_NULL)
            continue;
        if ((flag & MB_FLAG_FLAG) && !m_keepFlagged)
            continue;

        BathData bd{};
        bd.m_bathlon = m_bathlon[i];
        bd.m_bathlat = m_bathlat[i];
        bd.m_bath = -m_bath[i];
        bd.m_amp = m_amp[i];
        bd.m_time = gpsTime;
        bd.m_beamFlag = (double)(unsigned char)flag;
        // Per-ping values (same for every beam of this ping).
        bd.m_heading = aux.heading;
        bd.m_roll = aux.roll;
        bd.m_pitch = aux.pitch;
        bd.m_heave = aux.heave;
        bd.m_speed = aux.speed;
        bd.m_sonarDepth = aux.sonarDepth;
        bd.m_altitude = aux.altitude;
        bd.m_soundVelocity = aux.soundVelocity;
        bd.m_transmitGain = aux.transmitGain;
        bd.m_pulseLength = aux.pulseLength;
        bd.m_receiveGain = aux.receiveGain;
        bd.m_pingNumber = aux.pingNumber;
        // Per-beam travel time / angles, when mb_ttimes() supplied them and the
        // beam index is in range (ttimes is indexed like the bathymetry array).
        if (haveTtimes && (int)i < nbeams)
        {
            bd.m_travelTime = m_ttimes[i];
            bd.m_beamAngle = m_angles[i];
            bd.m_azimuthAngle = m_anglesForward[i];
        }
        if (haveExtract)
        {
            bd.m_acrossTrack = m_bathacross[i];
            bd.m_alongTrack = m_bathalong[i];
        }
        if (haveDetects && (int)i < ndet)
            bd.m_detection = (double)m_detects[i];
        if (havePulses && (int)i < npulse)
            bd.m_pulseType = (double)m_pulses[i];
        m_bathQueue.push(bd);
    }
    if (numBath != numAmp)
        log()->get(LogLevel::Warning) << getName() << ": Number of "
            "bathymetry values doesn't match number of amplitude "
            "values." << std::endl;
    return m_bathQueue.size();
}


bool MbReader::extractSidescan(int numSs, double gpsTime)
{
    for (size_t i = 0; i < (size_t)numSs; ++i)
    {
        // Skip null pixels (MB-System pads the sidescan array with
        // MB_SIDESCAN_NULL where there is no return).
        if (m_ss[i] == MB_SIDESCAN_NULL)
            continue;
        m_ssQueue.emplace(m_sslon[i], m_sslat[i], m_ss[i], gpsTime);
    }
    return m_ssQueue.size();
}


// Raw (un-georeferenced) sidescan: the port/starboard backscatter time series
// for this ping, emitted as a waterfall. Each sample becomes one point with
// X = signed slant range (port negative) and Y = ping number. Slant range is
// derived from the sample interval and the reference sound speed:
//   range = 0.5 * sampleIndex * sample_interval * soundspeed.
bool MbReader::extractRawSidescan(double gpsTime)
{
    void *store = nullptr;
    int serr = MB_ERROR_NO_ERROR;
    if (mb_get_store(0, m_ctx, &store, &serr) != MB_SUCCESS || !store)
        return false;

    // Size the sample buffers for this ping first.
    int kd = 0; double si = 0.0; int nPort = 0; int nStbd = 0;
    int ed = MB_ERROR_NO_ERROR;
    if (mb_extract_rawssdimensions(0, m_ctx, store, &kd, &si, &nPort, &nStbd,
            &ed) != MB_SUCCESS)
        return false;
    if (nPort < 0) nPort = 0;
    if (nStbd < 0) nStbd = 0;
    if (nPort == 0 && nStbd == 0)
        return false;
    if ((int)m_rawssPort.size() < nPort)
        m_rawssPort.resize(nPort);
    if ((int)m_rawssStbd.size() < nStbd)
        m_rawssStbd.resize(nStbd);

    int kr = 0; int sstype = 0; double sampleInterval = 0.0;
    double bwx = 0.0; double bwl = 0.0;
    int nPort2 = 0; int nStbd2 = 0;
    int er = MB_ERROR_NO_ERROR;
    if (mb_extract_rawss(0, m_ctx, store, &kr, &sstype, &sampleInterval, &bwx,
            &bwl, &nPort2, m_rawssPort.data(), &nStbd2, m_rawssStbd.data(),
            &er) != MB_SUCCESS)
        return false;

    const double halfC = 0.5 * (m_soundspeed > 0.0 ? m_soundspeed : 1500.0);
    const double pingY = m_pingNumber;

    auto emit = [&](int i, double value, int side)
    {
        if (value == MB_SIDESCAN_NULL)
            return;
        const double range = i * sampleInterval * halfC;
        RawData rd{};
        rd.m_slantRange = range;
        rd.m_x = (side == 0) ? -range : range;
        rd.m_y = pingY;
        rd.m_intensity = value;
        rd.m_time = gpsTime;
        rd.m_sampleIndex = (side == 0) ? -i : i;
        rd.m_side = side;
        m_rawQueue.push(rd);
    };

    for (int i = 0; i < nPort2; ++i)
        emit(i, m_rawssPort[i], 0);
    for (int i = 0; i < nStbd2; ++i)
        emit(i, m_rawssStbd[i], 1);

    m_pingNumber += 1.0;
    return m_rawQueue.size();
}


bool MbReader::extractSvp(int nsvp, double gpsTime)
{
    // Emit one point per profile sample: Z = -depth, SoundVelocity = speed.
    const int profile = m_svpCount++;
    for (int i = 0; i < nsvp; ++i)
    {
        SvpData s{};
        s.m_z = -m_svpDepth[i];
        s.m_velocity = m_svpVel[i];
        s.m_time = gpsTime;
        s.m_profile = profile;
        m_svpQueue.push(s);
    }
    return m_svpQueue.size();
}


void MbReader::recordSvp(int nsvp)
{
    // Attach each sound-velocity profile to the stage metadata as a list with
    // parallel depth[] / velocity[] arrays.
    MetadataNode svp = getMetadata().addList("svp");
    svp.add("index", m_svpCount++);
    svp.add("nsvp", nsvp);
    for (int i = 0; i < nsvp; ++i)
        svp.addList("depth", m_svpDepth[i]);
    for (int i = 0; i < nsvp; ++i)
        svp.addList("velocity", m_svpVel[i]);
}


void MbReader::recordSonarMeta(void *store)
{
    MetadataNode meta = getMetadata();

    int st = 0; int e = MB_ERROR_NO_ERROR;
    if (mb_sonartype(0, m_ctx, store, &st, &e) == MB_SUCCESS)
    {
        std::string name;
        switch (st)
        {
        case MB_TOPOGRAPHY_TYPE_ECHOSOUNDER:     name = "echosounder"; break;
        case MB_TOPOGRAPHY_TYPE_MULTIBEAM:       name = "multibeam"; break;
        case MB_TOPOGRAPHY_TYPE_SIDESCAN:        name = "sidescan"; break;
        case MB_TOPOGRAPHY_TYPE_INTERFEROMETRIC: name = "interferometric"; break;
        case MB_TOPOGRAPHY_TYPE_LIDAR:           name = "lidar"; break;
        case MB_TOPOGRAPHY_TYPE_CAMERA:          name = "camera"; break;
        default:                                 name = "unknown"; break;
        }
        meta.add("sonar_type", name);
    }

    int sst = 0; int e2 = MB_ERROR_NO_ERROR;
    if (mb_sidescantype(0, m_ctx, store, &sst, &e2) == MB_SUCCESS)
        meta.add("sidescan_type",
            std::string(sst == MB_SIDESCAN_LINEAR ? "linear" : "logarithmic"));
}


bool MbReader::processOne(PointRef& point)
{
    if (m_dataType == DataType::Multibeam)
    {
        if (m_bathQueue.empty())
            if (!loadData())
            {
                return false;
            }

        BathData& bd = m_bathQueue.front();

        point.setField(Dimension::Id::X, bd.m_bathlon);
        point.setField(Dimension::Id::Y, bd.m_bathlat);
        point.setField(Dimension::Id::Z, bd.m_bath);
        point.setField(Dimension::Id::GpsTime, bd.m_time);
        point.setField(Dimension::Id::Amplitude, bd.m_amp);
        if (m_level >= 2)
        {
            point.setField(m_idTravelTime, bd.m_travelTime);
            point.setField(m_idBeamAngle, bd.m_beamAngle);
            point.setField(m_idAzimuthAngle, bd.m_azimuthAngle);
            point.setField(m_idAcrossTrack, bd.m_acrossTrack);
            point.setField(m_idAlongTrack, bd.m_alongTrack);
            point.setField(m_idBeamFlag, bd.m_beamFlag);
            point.setField(m_idDetection, bd.m_detection);
            point.setField(m_idHeading, bd.m_heading);
            point.setField(m_idRoll, bd.m_roll);
            point.setField(m_idPitch, bd.m_pitch);
            point.setField(m_idHeave, bd.m_heave);
            point.setField(m_idSpeed, bd.m_speed);
            point.setField(m_idSonarDepth, bd.m_sonarDepth);
            point.setField(m_idAltitude, bd.m_altitude);
            point.setField(m_idSoundVelocity, bd.m_soundVelocity);
            point.setField(m_idTransmitGain, bd.m_transmitGain);
            point.setField(m_idPulseLength, bd.m_pulseLength);
            point.setField(m_idReceiveGain, bd.m_receiveGain);
            point.setField(m_idPulseType, bd.m_pulseType);
            point.setField(m_idPingNumber, bd.m_pingNumber);
        }
        m_bathQueue.pop();
    }
    else if (m_dataType == DataType::Sidescan)
    {
        if (m_ssQueue.empty())
            if (!loadData())
                return false;

        SidescanData& ss = m_ssQueue.front();

        point.setField(Dimension::Id::X, ss.m_sslon);
        point.setField(Dimension::Id::Y, ss.m_sslat);
        point.setField(Dimension::Id::GpsTime, ss.m_time);
        point.setField(Dimension::Id::Intensity, ss.m_ss);
        m_ssQueue.pop();
    }
    else if (m_dataType == DataType::RawSidescan)
    {
        if (m_rawQueue.empty())
            if (!loadData())
                return false;

        RawData& rd = m_rawQueue.front();

        point.setField(Dimension::Id::X, rd.m_x);
        point.setField(Dimension::Id::Y, rd.m_y);
        point.setField(Dimension::Id::Z, 0.0);
        point.setField(Dimension::Id::GpsTime, rd.m_time);
        point.setField(Dimension::Id::Intensity, rd.m_intensity);
        point.setField(m_idSampleIndex, rd.m_sampleIndex);
        point.setField(m_idSide, rd.m_side);
        point.setField(m_idSlantRange, rd.m_slantRange);
        m_rawQueue.pop();
    }
    else // Svp
    {
        if (m_svpQueue.empty())
            if (!loadData())
                return false;

        SvpData& s = m_svpQueue.front();

        point.setField(Dimension::Id::X, 0.0);
        point.setField(Dimension::Id::Y, 0.0);
        point.setField(Dimension::Id::Z, s.m_z);
        point.setField(Dimension::Id::GpsTime, s.m_time);
        point.setField(m_idSoundVelocity, s.m_velocity);
        point.setField(m_idProfileIndex, s.m_profile);
        m_svpQueue.pop();
    }
    return true;
}


QuickInfo MbReader::inspect()
{
    QuickInfo qi;
    std::unique_ptr<PointLayout> layout(new PointLayout());

    addDimensions(layout.get());

    Dimension::IdList dims = layout->dims();
    for (auto di = dims.begin(); di != dims.end(); ++di)
        qi.m_dimNames.push_back(layout->dimName(*di));
    qi.m_valid = true;
    return qi;
}


point_count_t MbReader::read(PointViewPtr view, point_count_t count)
{
    // Bind a PointRef to the view WITHOUT materializing point 0 — referencing
    // view->point(0) up front adds a stray all-zero point when the file yields
    // no matching records (e.g. datatype=svp on a file with no SVP). Matches
    // the canonical PDAL streamable-reader pattern (cf. TextReader).
    PointId idx = view->size();
    point_count_t cnt = 0;
    PointRef point(*view);
    while (cnt < count)
    {
        point.setPointId(idx);
        if (!processOne(point))
            break;
        cnt++;
        idx++;
    }
    return cnt;
}


void MbReader::done(PointTableRef table)
{
    int error;

    mb_close(0, &m_ctx, &error);
    getMetadata().addList("filename", m_filename);
}

inline std::istream& operator>>(std::istream& in, MbReader::DataType& dt)
{
    std::string s;
    in >> s;
    s = Utils::toupper(s);
    if (s == "MULTIBEAM")
        dt = MbReader::DataType::Multibeam;
    else if (s == "SIDESCAN")
        dt = MbReader::DataType::Sidescan;
    else if (s == "RAWSIDESCAN")
        dt = MbReader::DataType::RawSidescan;
    else if (s == "SVP")
        dt = MbReader::DataType::Svp;
    else
        in.setstate(std::ios_base::failbit);
    return in;
}

std::ostream& operator<<(std::ostream& out, const MbReader::DataType& dt)
{
    if (dt == MbReader::DataType::Multibeam)
        out << "Multibeam";
    else if (dt == MbReader::DataType::Sidescan)
        out << "Sidescan";
    else if (dt == MbReader::DataType::RawSidescan)
        out << "RawSidescan";
    else if (dt == MbReader::DataType::Svp)
        out << "Svp";
    return out;
}

} // namespace pdal
