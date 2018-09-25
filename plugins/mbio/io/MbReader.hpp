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

class PDAL_DLL MbReader : public Reader, public Streamable
{
    struct BathData
    {
        double m_bathlon;
        double m_bathlat;
        double m_bath;
        double m_amp;
        double m_time;

        BathData(double bathlon, double bathlat, double bath, double amp,
                double time) :
            m_bathlon(bathlon), m_bathlat(bathlat), m_bath(bath), m_amp(amp),
            m_time(time)
        {}
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

    enum class DataType
    {
        Multibeam,
        Sidescan
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
    bool extractMultibeam(int numBath, int numAmp, double time);
    bool extractSidescan(int numSs, double time);

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
    std::queue<BathData> m_bathQueue;
    std::queue<SidescanData> m_ssQueue;
    MbFormat m_format;
    double m_timegap;
    double m_speedmin;
    DataType m_dataType;
};

} // namespace PDAL
