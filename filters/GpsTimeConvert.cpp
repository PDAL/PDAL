/******************************************************************************
 * Copyright (c) 2021, Preston J. Hartzell (preston.hartzell@gmail.com)
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
 *     * Neither the name of Hobu, inc. or Flaxen Geo Consulting nor the
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

#include "GpsTimeConvert.hpp"

namespace pdal
{

static PluginInfo const s_info{
    "filters.gpstimeconvert",
    "Convert between GPS Time, GPS Standard Time, and GPS Week Seconds",
    "http://pdal.io/stages/filters.gpstimeconvert.html"};

CREATE_STATIC_STAGE(GpsTimeConvert, s_info)

std::string GpsTimeConvert::getName() const
{
    return s_info.name;
}

void GpsTimeConvert::addArgs(ProgramArgs& args)
{
    args.add("conversion", "conversion (deprecated)", m_conversion);
    args.add("in_time", "input time type", m_inTime).setPositional();
    args.add("out_time", "output time type", m_outTime).setPositional();
    args.add("start_date", "GMT start date of data in 'YYYY-MM-DD' format",
             m_strDate, "");
    args.add(
        "wrap",
        "reset output week seconds to zero on Sundays, day second at midnight",
        m_wrap, false);
    args.add(
        "wrapped",
        "input weeks seconds reset to zero on Sundays, day second at midnight",
        m_wrapped, false);
}

void GpsTimeConvert::testTimeType(std::string& type)
{
    // clean time type
    if (Utils::iequals(type, "gt"))
        type = "gt";
    else if (Utils::iequals(type, "gst"))
        type = "gst";
    else if (Utils::iequals(type, "gws"))
        type = "gws";
    else if (Utils::iequals(type, "gds"))
        type = "gds";
    else
        throwError("Invalid time type.");
}

void GpsTimeConvert::initialize()
{
    m_first = true;
    m_lastTime = std::numeric_limits<double>::lowest();
    m_numSeconds = std::numeric_limits<int>::lowest();
    if (!m_conversion.empty() && (m_inTime.empty()) && (m_outTime.empty()))
    {
        m_conversion = Utils::tolower(m_conversion);
        std::vector<std::string> s = Utils::split(m_conversion, '2');
        m_inTime = s[0];
        m_outTime = s[1];
    }
    else if (m_conversion.empty() && (!m_inTime.empty()) &&
             (!m_outTime.empty()))
    {
        m_inTime = Utils::tolower(m_inTime);
        m_outTime = Utils::tolower(m_outTime);
    }
    else
    {
        Stage::throwError("Use 'conversion' or 'in_time' and 'out_time'.");
    }

    testTimeType(m_inTime);
    testTimeType(m_outTime);

    // if converting from week or day seconds, 'start_date' is required and must
    // be in YYYY-MM-DD format
    if (m_inTime == "gws" || m_inTime == "gds")
    {
        if (m_strDate == "")
            Stage::throwError("'start_date' option is required.");

        m_tmDate = {};
        std::istringstream ss(m_strDate);
        ss >> std::get_time(&m_tmDate, "%Y-%m-%d");
        if (ss.fail())
            Stage::throwError("'start_date' must be in YYYY-MM-DD format.");
        else
            std::mktime(&m_tmDate);

        if (m_inTime == "gws")
            // seconds from GPS zero to first day of week
            m_numSeconds = weekStartGpsSeconds(m_tmDate);
        else
            // seconds from GPS zero to first day of week
            m_numSeconds = dayStartGpsSeconds(m_tmDate);

        // adjust for gps standard time
        if (m_outTime == "gst")
            m_numSeconds -= 1000000000;
    }
}

std::tm GpsTimeConvert::gpsTime2Date(int seconds)
{
    std::tm gpsZero = {};
    gpsZero.tm_year = 80;
    gpsZero.tm_mon = 0;
    gpsZero.tm_mday = 6;
    gpsZero.tm_hour = 0;
    gpsZero.tm_min = 0;
    gpsZero.tm_sec = seconds;

    // refresh struct
    std::mktime(&gpsZero);

    // clear fractional date info
    gpsZero.tm_hour = 0;
    gpsZero.tm_min = 0;
    gpsZero.tm_sec = 0;

    return gpsZero;
}

int GpsTimeConvert::weekStartGpsSeconds(std::tm date)
{
    // GPS zero time
    std::tm gpsZero = {};
    gpsZero.tm_year = 80;
    gpsZero.tm_mon = 0;
    gpsZero.tm_mday = 6;
    gpsZero.tm_hour = 0;
    gpsZero.tm_min = 0;
    gpsZero.tm_sec = 0;

    // back up the time to the first day of the week
    date.tm_mday -= date.tm_wday;

    // refresh struct
    std::mktime(&date);

    // seconds from GPS zero to first day of week
    std::chrono::system_clock::time_point durStart, durEnd;
    durStart = std::chrono::system_clock::from_time_t(std::mktime(&gpsZero));
    durEnd = std::chrono::system_clock::from_time_t(std::mktime(&date));

    std::chrono::duration<int> duration;
    duration = std::chrono::duration_cast<std::chrono::duration<int>>(durEnd -
                                                                      durStart);

    return duration.count();
}

int GpsTimeConvert::dayStartGpsSeconds(std::tm date)
{
    // GPS zero time
    std::tm gpsZero = {};
    gpsZero.tm_year = 80;
    gpsZero.tm_mon = 0;
    gpsZero.tm_mday = 6;
    gpsZero.tm_hour = 0;
    gpsZero.tm_min = 0;
    gpsZero.tm_sec = 0;

    // refresh struct
    std::mktime(&date);

    // seconds from GPS zero to first day of week
    std::chrono::system_clock::time_point durStart, durEnd;
    durStart = std::chrono::system_clock::from_time_t(std::mktime(&gpsZero));
    durEnd = std::chrono::system_clock::from_time_t(std::mktime(&date));

    std::chrono::duration<int> duration;
    duration = std::chrono::duration_cast<std::chrono::duration<int>>(durEnd -
                                                                      durStart);

    return duration.count();
}

void GpsTimeConvert::unwrapWeekSeconds(PointRef& point)
{
    // any decrease in time is interpreted as a week rollover
    while (point.getFieldAs<double>(Dimension::Id::GpsTime) < m_lastTime)
    {
        double t = point.getFieldAs<double>(Dimension::Id::GpsTime);
        point.setField(Dimension::Id::GpsTime, t + 604800);
    }
    m_lastTime = point.getFieldAs<double>(Dimension::Id::GpsTime);
}

void GpsTimeConvert::unwrapDaySeconds(PointRef& point)
{
    // any decrease in time is interpreted as a day rollover
    while (point.getFieldAs<double>(Dimension::Id::GpsTime) < m_lastTime)
    {
        double t = point.getFieldAs<double>(Dimension::Id::GpsTime);
        point.setField(Dimension::Id::GpsTime, t + 86400);
    }
    m_lastTime = point.getFieldAs<double>(Dimension::Id::GpsTime);
}

void GpsTimeConvert::wrapWeekSeconds(PointRef& point)
{
    // a time greater than or equal to 604800 indicates a new week has started
    while (point.getFieldAs<double>(Dimension::Id::GpsTime) >= 604800)
    {
        double t = point.getFieldAs<double>(Dimension::Id::GpsTime);
        point.setField(Dimension::Id::GpsTime, t - 604800);
    }
}

void GpsTimeConvert::wrapDaySeconds(PointRef& point)
{
    // a time greater than or equal to 86400 indicates a new day has started
    while (point.getFieldAs<double>(Dimension::Id::GpsTime) >= 86400)
    {
        double t = point.getFieldAs<double>(Dimension::Id::GpsTime);
        point.setField(Dimension::Id::GpsTime, t - 86400);
    }
}

void GpsTimeConvert::weekSeconds2GpsTime(PointRef& point)
{
    // handle wrapped week seconds
    if (m_wrapped)
        unwrapWeekSeconds(point);

    // add to week seconds

    double t = point.getFieldAs<double>(Dimension::Id::GpsTime);
    point.setField(Dimension::Id::GpsTime, t + m_numSeconds);
}

void GpsTimeConvert::daySeconds2GpsTime(PointRef& point)
{
    // handle wrapped week seconds
    if (m_wrapped)
        unwrapDaySeconds(point);

    // add to week seconds
    double t = point.getFieldAs<double>(Dimension::Id::GpsTime);
    point.setField(Dimension::Id::GpsTime, t + m_numSeconds);
}

void GpsTimeConvert::gpsTime2WeekSeconds(PointRef& point)
{

    double t = point.getFieldAs<double>(Dimension::Id::GpsTime);
    point.setField(Dimension::Id::GpsTime, t - m_numSeconds);

    // wrap week seconds
    if (m_wrap)
        wrapWeekSeconds(point);
}

void GpsTimeConvert::gpsTime2DaySeconds(PointRef& point)
{
    // strip off time to day
    double t = point.getFieldAs<double>(Dimension::Id::GpsTime);
    point.setField(Dimension::Id::GpsTime, t - m_numSeconds);

    // wrap week seconds
    if (m_wrap)
        wrapDaySeconds(point);
}

void GpsTimeConvert::gpsTime2GpsTime(PointRef& point)
{
    double tOffset = 1000000000;
    if (m_inTime == "gt" && m_outTime == "gst")
        tOffset *= -1;

    double t = point.getFieldAs<double>(Dimension::Id::GpsTime);
    point.setField(Dimension::Id::GpsTime, t + tOffset);
}

bool GpsTimeConvert::processOne(PointRef& point)
{
    if (m_inTime == "gws")
    {
        if (m_outTime == "gst" || m_outTime == "gt")
            weekSeconds2GpsTime(point);
        else if (m_outTime == "gds")
        {
            weekSeconds2GpsTime(point);
            gpsTime2DaySeconds(point);
        }
    }
    else if (m_inTime == "gds")
    {
        if (m_outTime == "gst" || m_outTime == "gt")
            daySeconds2GpsTime(point);
        if (m_outTime == "gws")
        {
            daySeconds2GpsTime(point);
            gpsTime2WeekSeconds(point);
        }
    }
    else if (m_inTime == "gst" || m_inTime == "gt")
    {
        if (m_first)
        {
            int tOffset = 0;
            if (m_inTime == "gst")
                tOffset = 1000000000;

            // date of first time
            double t =
                point.getFieldAs<double>(Dimension::Id::GpsTime) + tOffset;
            std::tm firstDate = gpsTime2Date((int)t);
            // seconds from GPS zero to day
            if (m_outTime == "gds")
                m_numSeconds = dayStartGpsSeconds(firstDate) - tOffset;
            else if (m_outTime == "gws")
                m_numSeconds = weekStartGpsSeconds(firstDate) - tOffset;
            m_first = false;
        }
        if (m_outTime == "gws")
            gpsTime2WeekSeconds(point);
        else if (m_outTime == "gds")
            gpsTime2DaySeconds(point);
        else if ((m_outTime == "gst" || m_outTime == "gt") &&
                 m_inTime != m_outTime)
            gpsTime2GpsTime(point);
    }

    return true;
}

void GpsTimeConvert::filter(PointView& view)
{
    for (PointRef p : view)
        processOne(p);
}

} // namespace pdal
