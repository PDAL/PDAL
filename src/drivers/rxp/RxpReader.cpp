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

#include <pdal/drivers/rxp/RxpReader.hpp>


namespace pdal
{
namespace drivers
{
namespace rxp
{


std::string extractRivlibURI(const Options& options)
{
    if (options.hasOption("filename"))
    {
        if (options.hasOption("rdtp"))
        {
            throw option_not_found("Cannot create URI when both filename and rdtp are provided");
        }
        return "file:" + options.getValueOrThrow<std::string>("filename");
    }
    else
    {
        return "rdtp://" + options.getValueOrThrow<std::string>("rdtp");
    }
}


Dimension::Id::Enum getTimeDimensionId(bool syncToPps)
{
    return syncToPps ? Dimension::Id::GpsTime : Dimension::Id::InternalTime;
}


Dimension::IdList getRxpDimensions(bool syncToPps, bool minimal)
{
    using namespace Dimension;
    Dimension::IdList ids;

    ids.push_back(Id::X);
    ids.push_back(Id::Y);
    ids.push_back(Id::Z);
    ids.push_back(getTimeDimensionId(syncToPps));
    if (!minimal)
    {
        ids.push_back(Id::ReturnNumber);
        ids.push_back(Id::NumberOfReturns);
        ids.push_back(Id::Amplitude);
        ids.push_back(Id::Reflectance);
        ids.push_back(Id::EchoRange);
        ids.push_back(Id::Deviation);
        ids.push_back(Id::BackgroundRadiation);
        ids.push_back(Id::IsPpsLocked);
    }

    return ids;
}


RotationMatrix makeRotationMatrix(float roll, float pitch)
{
    using namespace std;
    return {{
        {cos(pitch), sin(roll) * sin(pitch),  -cos(roll) * sin(pitch)},
        {0,          cos(roll),               sin(roll)},
        {sin(pitch), -sin(roll) * cos(pitch), cos(roll) * cos(pitch)}
    }};
}


Point rotatePoint(const Point& p, const RotationMatrix& m)
{
    return {
        p.x * m[0][0] + p.y * m[0][1] + p.z * m[0][2],
        p.x * m[1][0] + p.y * m[1][1] + p.z * m[1][2],
        p.x * m[2][0] + p.y * m[2][1] + p.z * m[2][2]
    };
}


Options RxpReader::getDefaultOptions()
{
    Options options;
    options.add("sync_to_pps", DEFAULT_SYNC_TO_PPS, "");
    options.add("inclination_fix", DEFAULT_INCL_FIX, "");
    options.add("minimal", DEFAULT_MINIMAL, "");
    return options;
}


void RxpReader::processOptions(const Options& options)
{
    m_uri = extractRivlibURI(options);
    m_syncToPps = options.getValueOrDefault<bool>("sync_to_pps",
                                                  DEFAULT_SYNC_TO_PPS);
    m_inclFix = options.getValueOrDefault<bool>("inclination_fix",
                                                  DEFAULT_INCL_FIX);
    m_inclFixWindow = options.getValueOrDefault<InclinationVector::size_type>("inclination_fix_window",
                                                      DEFAULT_INCL_FIX_WINDOW);
    m_minimal = options.getValueOrDefault<bool>("minimal", DEFAULT_MINIMAL);
}


void RxpReader::addDimensions(PointContext ctx)
{
    ctx.registerDims(getRxpDimensions(m_syncToPps, m_minimal));
}


void RxpReader::ready(PointContext ctx)
{
    if (m_inclFix)
        m_pointcloud = std::make_shared<RxpInclFixPointcloud>(
                m_uri, m_syncToPps, m_minimal, ctx, m_inclFixWindow);
    else
        m_pointcloud = std::make_shared<RxpPointcloud>(m_uri, m_syncToPps, m_minimal, ctx);
}


point_count_t RxpReader::read(PointBuffer& buf, point_count_t count)
{
    point_count_t numRead = m_pointcloud->read(buf, count);
    return numRead;
}


void RxpReader::done(PointContext ctx)
{
    m_pointcloud.reset();
}


RxpPointcloud::RxpPointcloud(const std::string& uri, bool syncToPps, bool minimal, PointContext ctx)
    : scanlib::pointcloud(syncToPps)
    , m_buf(PointBufferPtr(new PointBuffer(ctx)))
    , m_idx(0)
    , m_syncToPps(syncToPps)
    , m_minimal(minimal)
    , m_rc(scanlib::basic_rconnection::create(uri))
    , m_dec(m_rc)
{}


RxpPointcloud::~RxpPointcloud()
{
    m_rc->close();
}


point_count_t RxpPointcloud::read(PointBuffer& buf, point_count_t count)
{
    point_count_t numRead = 0;

    if (m_idx < m_buf->size())
    {
        numRead += writeSavedPoints(buf, count);
        if (numRead == count)
            return numRead;
    }

    for (m_dec.get(m_rxpbuf); !m_dec.eoi(); m_dec.get(m_rxpbuf))
    {
        dispatch(m_rxpbuf.begin(), m_rxpbuf.end());
        if (m_buf->size() - m_idx + numRead >= count) break;
    }

    numRead += writeSavedPoints(buf, count - numRead);

    return numRead;
}


point_count_t RxpPointcloud::writeSavedPoints(PointBuffer& buf, point_count_t count)
{
    point_count_t numRead = 0;
    while (m_idx < m_buf->size() && numRead < count)
    {
        buf.appendPoint(*m_buf, m_idx);
        ++m_idx, ++numRead;
    }
    return numRead;
};


void RxpPointcloud::on_echo_transformed(echo_type echo)
{
    if (!(scanlib::pointcloud::single == echo || scanlib::pointcloud::last == echo))
    {
        // Come back later, when we've got all the echos
        return;
    }

    using namespace Dimension;

    boost::uint32_t idx = m_buf->size();
    unsigned int returnNumber = 1;
    for (auto t : targets)
    {
        m_buf->setField(Id::X, idx, t.vertex[0]);
        m_buf->setField(Id::Y, idx, t.vertex[1]);
        m_buf->setField(Id::Z, idx, t.vertex[2]);
        m_buf->setField(getTimeDimensionId(m_syncToPps), idx, t.time);
        if (!m_minimal)
        {
            m_buf->setField(Id::Amplitude, idx, t.amplitude);
            m_buf->setField(Id::Reflectance, idx, t.reflectance);
            m_buf->setField(Id::ReturnNumber, idx, returnNumber);
            m_buf->setField(Id::NumberOfReturns, idx, targets.size());
            m_buf->setField(Id::EchoRange, idx, t.echo_range);
            m_buf->setField(Id::Deviation, idx, t.deviation);
            m_buf->setField(Id::BackgroundRadiation, idx, t.background_radiation);
            m_buf->setField(Id::IsPpsLocked, idx, t.is_pps_locked);
        }
        ++idx, ++returnNumber;
    }
}


RxpInclFixPointcloud::RxpInclFixPointcloud(const std::string& uri,
                                           bool syncToPps,
                                           bool minimal,
                                           PointContext ctx,
                                           InclinationVector::size_type windowSize)
    : RxpPointcloud(uri, syncToPps, minimal, ctx)
    , m_windowSize(windowSize)
    , m_inclIdx(0)
{}


RxpInclFixPointcloud::~RxpInclFixPointcloud()
{}


point_count_t RxpInclFixPointcloud::writeSavedPoints(PointBuffer& buf, point_count_t count)
{
    using namespace Dimension;

    point_count_t numRead = 0;
    auto timeDim = getTimeDimensionId(isSyncToPps());

    // Skip all inclinations that don't have a point coming right afterwards.
    // This is to prevent wonky inclination readings at the start from throwing
    // off the whole game.
    while (m_buf->getFieldAs<double>(timeDim, m_idx) > m_incl[m_inclIdx + 1].time &&
            m_inclIdx < m_incl.size() - 2)
        ++m_inclIdx;
    auto firstUsableIncl = m_inclIdx;

    // Not enough inclinations to correct anything.
    if (m_incl.size() < (m_windowSize * 2 + firstUsableIncl))
    {
        std::ostringstream ss;
        ss << "Not enough inclination readings around points. Needed "
            << m_windowSize * 2 << ", got " << m_incl.size();
        throw pdal_error(ss.str());
    }

    // Skip forward until we have a full window of inclination readings.
    while (m_inclIdx < m_windowSize + firstUsableIncl - 1)
        ++m_inclIdx;

    // Don't write points that come before the first useful inclination
    // window.
    while (m_buf->getFieldAs<double>(timeDim, m_idx) < m_incl[m_inclIdx].time)
        ++m_idx;

    auto startingIncl = movingAverage(m_incl, m_inclIdx, m_windowSize);
    Inclination movingAverageIncl;
    auto startingIdx = m_inclIdx;
    RotationMatrix mat;

    while (m_idx < m_buf->size() && numRead < count)
    {
        // If the next point is in the next inclination window, move along.
        if (m_buf->getFieldAs<double>(timeDim, m_idx) > m_incl[m_inclIdx + 1].time)
        {
            ++m_inclIdx;
            if (m_inclIdx == m_incl.size() - m_windowSize)
                break;
            movingAverageIncl = movingAverage(m_incl, m_inclIdx, m_windowSize);
            float dRoll = startingIncl.roll * 0.001 -
                movingAverageIncl.roll * 0.001;
            float dPitch = startingIncl.pitch * 0.001 -
                movingAverageIncl.pitch * 0.001;
            mat = makeRotationMatrix(dRoll * M_PI / 180, dPitch * M_PI / 180);
        }
        else
        {
            // Only bother correcting points if they're past the first
            // inclination window
            if (m_inclIdx > startingIdx)
            {
                Point p = {
                    m_buf->getFieldAs<float>(Id::X, m_idx),
                    m_buf->getFieldAs<float>(Id::Y, m_idx),
                    m_buf->getFieldAs<float>(Id::Z, m_idx)
                };
                Point pnew = rotatePoint(p, mat);
                m_buf->setField(Id::X, m_idx, pnew.x);
                m_buf->setField(Id::Y, m_idx, pnew.y);
                m_buf->setField(Id::Z, m_idx, pnew.z);
            }
            buf.appendPoint(*m_buf, m_idx);
            ++m_idx, ++numRead;
        }
    }

    return numRead;
}


void RxpInclFixPointcloud::on_hk_incl(const scanlib::hk_incl<iterator_type>& incl)
{
    m_incl.push_back(Inclination{ time, incl.ROLL, incl.PITCH });
}


Inclination movingAverage(const InclinationVector& incl,
                          InclinationVector::size_type idx,
                          InclinationVector::size_type halfWindowSize)
{
    auto middle = incl.begin() + idx;
    double time = (incl[idx].time + incl[idx + 1].time) / 2;
    long roll(0), pitch(0);
    for (auto it = middle - halfWindowSize + 1;
            it != middle + halfWindowSize + 1;
            ++it)
    {
        roll += it->roll;
        pitch += it->pitch;
    }
    try
    {
        return {
            time,
            boost::numeric_cast<int16_t>(roll
                    / (static_cast<long>(halfWindowSize) * 2)),
            boost::numeric_cast<int16_t>(pitch
                    / (static_cast<long>(halfWindowSize) * 2))
        };
    }
    catch (boost::numeric::bad_numeric_cast& e)
    {
        throw pdal_error("Unable to calculate inclination moving average due "
                " to invalid roll or pitch value.");
    }
}


}
}
} // namespace pdal::drivers::rxp
