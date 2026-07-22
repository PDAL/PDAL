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
    m_sslon(nullptr), m_sslat(nullptr)
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
    args.add("datatype", "Multibeam (default) or sidescan.", m_dataType,
        DataType::Multibeam);
}


void MbReader::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;

    std::vector<Dimension::Id> dims { Id::X, Id::Y, Id::Z, Id::GpsTime };

    layout->registerDims(dims);
    if (m_dataType == DataType::Multibeam)
        layout->registerDim(Id::Amplitude);
    else
        layout->registerDim(Id::Intensity, Type::Double);
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

        // mb_read() already returns the ping time as a Unix epoch (UTC) in
        // pingTimeT; use it directly. The previous hand-rolled timeconvert()
        // added 1900 to MB-System's already-4-digit year and indexed the
        // cumulative-days table with a 1-based month, skewing GpsTime by ~1900
        // years and one month.
        double gpsTime = pingTimeT;

        if (error > 0)
        {
            if (error != MB_ERROR_EOF)
                throwError("Error reading data: " + MbError::text(error));
            return false;
        }

        if (status == MB_SUCCESS && kind == 1)
        {
            bool ok(false);
            if (m_dataType == DataType::Multibeam)
                ok = extractMultibeam(numBath, numAmp, gpsTime);
            else
                ok = extractSidescan(numSs, gpsTime);
            if (ok)
                break;
        }
    }
    return true;
}


bool MbReader::extractMultibeam(int numBath, int numAmp, double gpsTime)
{
    for (size_t i = 0; i < (size_t)numBath; ++i)
    {
        if (m_bathflag[i] & 1)
            continue;
        m_bathQueue.emplace(m_bathlon[i], m_bathlat[i], -m_bath[i],
                m_amp[i], gpsTime);
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
        // skip the null-fill pixels (MB_SIDESCAN_NULL == -1e9) so they are not
        // emitted as spurious points
        if (m_ss[i] == MB_SIDESCAN_NULL)
            continue;
        m_ssQueue.emplace(m_sslon[i], m_sslat[i], m_ss[i], gpsTime);
    }
    return m_ssQueue.size();
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
        m_bathQueue.pop();
    }
    else // Sidescan
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
    using namespace pdal::Dimension;

    PointRef point(*view);
    PointId id;
    for (id = 0; id < count; ++id)
    {
        point.setPointId(id);
        if (!processOne(point))
            break;
    }
    return id;
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
    return out;
}

} // namespace pdal
