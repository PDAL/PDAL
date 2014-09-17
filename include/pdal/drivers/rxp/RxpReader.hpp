/******************************************************************************
* Copyright (c) 2014, Peter J. Gadomski (pete.gadomski@gmail.com)
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

#include <riegl/scanlib.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/Reader.hpp>


namespace pdal
{
namespace drivers
{
namespace rxp
{


struct Inclination
{
    double time;
    int16_t roll;
    int16_t pitch;
};

struct Point
{
    float x;
    float y;
    float z;
};

typedef std::vector<Inclination> InclinationVector;
typedef std::array< std::array<float, 3>, 3> RotationMatrix;


const bool DEFAULT_SYNC_TO_PPS = true;
const bool DEFAULT_MINIMAL = false;
const bool DEFAULT_INCL_FIX = false;
const InclinationVector::size_type DEFAULT_INCL_FIX_WINDOW = 100;


Dimension::Id::Enum getTimeDimensionId(bool syncToPps);
std::string extractRivlibURI(const Options& options);
Dimension::IdList getRxpDimensions(bool syncToPps, bool minimal);
RotationMatrix makeRotationMatrix(float roll, float pitch);
Point rotatePoint(const Point& point, const RotationMatrix& matrix);
Inclination movingAverage(const InclinationVector& incl,
                          InclinationVector::size_type idx,
                          InclinationVector::size_type halfWindowSize);


class PDAL_DLL RxpPointcloud : public scanlib::pointcloud
{
public:
    RxpPointcloud(const std::string& uri, bool isSyncToPps, bool m_minimal, PointContext ctx);
    virtual ~RxpPointcloud();

    point_count_t read(PointBuffer& buf, point_count_t count);
    virtual point_count_t writeSavedPoints(PointBuffer& buf, point_count_t count);

    inline bool isSyncToPps() const
    {
        return m_syncToPps;
    }

    PointBufferPtr m_buf;
    point_count_t m_idx;

protected:
    void on_echo_transformed(echo_type echo);

private:
    bool m_syncToPps;
    bool m_minimal;
    std::shared_ptr<scanlib::basic_rconnection> m_rc;
    scanlib::decoder_rxpmarker m_dec;
    scanlib::buffer m_rxpbuf;

};


class PDAL_DLL RxpInclFixPointcloud : public RxpPointcloud
{
public:
    RxpInclFixPointcloud(const std::string& uri, bool isSyncToPps, bool minimal,
                         PointContext ctx, InclinationVector::size_type windowSize);
    virtual ~RxpInclFixPointcloud();

    virtual point_count_t writeSavedPoints(PointBuffer& buf, point_count_t count);

protected:
    void on_hk_incl(const scanlib::hk_incl<iterator_type>& arg);

private:
    InclinationVector::size_type m_windowSize;
    InclinationVector m_incl;
    InclinationVector::size_type m_inclIdx;

};


class PDAL_DLL RxpReader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.rxp.reader", "RXP Reader")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.rxp.reader.html")
    SET_STAGE_ENABLED(true)

    RxpReader(const Options& options)
        : pdal::Reader(options)
        , m_uri("")
        , m_syncToPps(DEFAULT_SYNC_TO_PPS)
        , m_inclFix(DEFAULT_INCL_FIX)
        , m_inclFixWindow(DEFAULT_INCL_FIX_WINDOW)
        , m_pointcloud(NULL)
    {}

    static Options getDefaultOptions();
    static Dimension::IdList getDefaultDimensions()
    {
        return getRxpDimensions(DEFAULT_SYNC_TO_PPS, DEFAULT_MINIMAL);
    }

private:
    virtual void processOptions(const Options& options);
    virtual void addDimensions(PointContext ctx);
    virtual void ready(PointContext ctx);
    virtual point_count_t read(PointBuffer& buf, point_count_t count);
    virtual void done(PointContext ctx);

    std::string m_uri;
    bool m_syncToPps;
    bool m_inclFix;
    unsigned int m_inclFixWindow;
    bool m_minimal;
    std::shared_ptr<RxpPointcloud> m_pointcloud;

};


}
}
} // namespace pdal::drivers::rxp
