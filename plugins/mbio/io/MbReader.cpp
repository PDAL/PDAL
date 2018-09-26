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
    "http://pdal.io/stages/readers.mbio.html"
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
    int btime_i[7] { 0, 0, 0, 0, 0, 0, 0 };
    int etime_i[7] { (std::numeric_limits<int>::max)(), 0, 0, 0, 0, 0 };
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

namespace
{

int leapSeconds(int y, int m)
{
    struct leap
    {
        int year;
        int month;
    };

    const std::vector<leap> leaps
    {
        {1981, 7}, {1982, 7}, {1983, 7}, {1985, 7}, {1988, 1}, {1990, 1},
        {1991, 1}, {1992, 7}, {1993, 7}, {1994, 7}, {1996, 1}, {1997, 1},
        {1999, 1}, {2006, 1}, {2009, 1}, {2012, 7}, {2015, 7}, {2017, 1}
    };

    int secs = 0;
    for (size_t i = 0; i < leaps.size(); ++i)
    {
        if (y > leaps[i].year)
            secs++;
        else if (y == leaps[i].year && m >= leaps[i].month)
        {
            secs++;
            break;
        }
        else
            break;
    }
    return secs;
}

// This is an attempt to convert mb_system's poorly defined time_i to
// adjusted GPS standard time (GPS standard time offset by 1.0E9)
// Most of this is from converting struct tm (assuming UTC) to time_t
// from Eric Raymond.
double timeconvert(int time_i[7])
{
    const uint8_t yearIdx = 0;
    const uint8_t monthIdx = 1;
    const uint8_t dayIdx = 2;
    const uint8_t hourIdx = 3;
    const uint8_t minuteIdx = 4;
    const uint8_t secondIdx = 5;
    const uint8_t microsecIdx = 6;
    const uint8_t monthsPerYear = 12;
    const int cumdays[monthsPerYear] =
        { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

    long year;
    time_t result;

    year = 1900 + time_i[yearIdx] + time_i[monthIdx] / monthsPerYear;
    result = (year - 1970) * 365 + cumdays[time_i[monthIdx] % monthsPerYear];
    result += (year - 1968) / 4;
    result -= (year - 1900) / 100;
    result += (year - 1600) / 400;
    if ((year % 4) == 0 && ((year % 100) != 0 || (year % 400) == 0) &&
        (time_i[monthIdx] % monthsPerYear) < 2)
        result--;
    result += time_i[dayIdx] - 1;
    result *= 24;
    result += time_i[hourIdx];
    result *= 60;
    result += time_i[minuteIdx];
    result *= 60;
    result += time_i[secondIdx];
    /**
    if (t->tm_isdst == 1)
        result -= 3600;
    **/
    // We should now have time_t in result.  Adjust offset to GPS standard
    // 1/1/1970 to 6/1/1980 in seconds.  Perhaps cleaner than messing with
    // the calculations above which are known to work.
    result -= 315964800;
    // Now subtract seconds for each leap second that has happened between
    // the GPS time root.  These are "announced", so we can't really plan here.
    result -= leapSeconds(time_i[yearIdx], (time_i[monthIdx] / 12) + 1);
    // Now subtract for "adjusted" GPS standard time.
    result -= 1000000000UL;
    // Add back the microseconds as a fraction of a second and convert
    // to double.
    return result + (time_i[microsecIdx] / 1000000.0);
}

} // namespace

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

        double gpsTime = timeconvert(pingTime);

        if (status == 0)
        {
            if (error > 0 && error != MB_ERROR_EOF)
                throwError("Error reading data: " + MbError::text(error));
            return false;
        }

        if (kind == 1)
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
        m_ssQueue.emplace(m_sslon[i], m_sslat[i], m_ss[i], gpsTime);
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

    PointRef point = view->point(0);
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
