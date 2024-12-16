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

#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>

namespace pdal
{

class PDAL_EXPORT GpsTimeConvert : public Filter, public Streamable
{
public:
    GpsTimeConvert() : Filter()
    {}
    std::string getName() const;

private:
    std::string m_conversion;
    std::string m_inTime;
    std::string m_outTime;
    std::string m_strDate;
    std::tm m_tmDate;
    bool m_wrap;
    bool m_wrapped;
    bool m_first;
    double m_lastTime;
    double m_wrappedTolerance;
    int m_numSeconds;

        void weekSeconds2GpsTime(PointRef& point);
    void daySeconds2GpsTime(PointRef& point);

    void gpsTime2WeekSeconds(PointRef& point);
    void gpsTime2DaySeconds(PointRef& point);

    void gpsTime2GpsTime(PointRef& point);

    std::tm gpsTime2Date(int seconds);

    int weekStartGpsSeconds(std::tm date);
    int dayStartGpsSeconds(std::tm date);

    void unwrapWeekSeconds(PointRef& point);
    void wrapWeekSeconds(PointRef& point);
    
    void unwrapDaySeconds(PointRef& point);
    void wrapDaySeconds(PointRef& point);

    void testTimeType(std::string& type);

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual PointViewSet run(PointViewPtr view);
    virtual bool processOne(PointRef& point);
    virtual void filter(PointView& view);

    GpsTimeConvert& operator=(const GpsTimeConvert&); // not implemented
    GpsTimeConvert(const GpsTimeConvert&); // not implemented
};

} // namespace pdal
