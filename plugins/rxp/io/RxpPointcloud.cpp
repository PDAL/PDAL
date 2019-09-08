/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski (pete.gadomski@gmail.com)
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
*
*
* This software has not been developed by RIEGL, and RIEGL will not provide any
* support for this driver. Please do not contact RIEGL with any questions or
* issues regarding this driver. RIEGL is not responsible for damages or other
* issues that arise from use of this driver. This driver has been tested
* against RiVLib version 1.39 on a Ubuntu 14.04 using gcc43.
 ****************************************************************************/

#include "RxpPointcloud.hpp"


namespace pdal
{


Dimension::Id getTimeDimensionId(bool syncToPps)
{
    return syncToPps ? Dimension::Id::GpsTime : Dimension::Id::InternalTime;
}


Point::Point(scanlib::target target, unsigned int returnNumber, unsigned int numberOfReturns, bool edgeOfFlightLine)
    : target(target)
    , returnNumber(returnNumber)
    , numberOfReturns(numberOfReturns)
    , edgeOfFlightLine(edgeOfFlightLine)
{}


RxpPointcloud::RxpPointcloud(
        const std::string& uri,
        bool syncToPps,
        bool reflectanceAsIntensity,
        float minReflectance,
        float maxReflectance,
        PointTableRef table)
    : scanlib::pointcloud(syncToPps)
    , m_syncToPps(syncToPps)
    , m_reflectanceAsIntensity(reflectanceAsIntensity)
    , m_minReflectance(minReflectance)
    , m_maxReflectance(maxReflectance)
    , m_rc(scanlib::basic_rconnection::create(uri))
    , m_dec(m_rc)
    , m_edge(false)
{}


RxpPointcloud::~RxpPointcloud()
{
    m_rc->close();
}


bool RxpPointcloud::readOne(PointRef& point)
{
    if (readSavedPoint(point)) {
        return true;
    }
    savePoints();
    return readSavedPoint(point);
}


bool RxpPointcloud::readSavedPoint(PointRef& point)
{
    if (!m_points.empty()) {
        copyPoint(m_points.front(), point);
        m_points.pop_front();
        return true;
    } else {
        return false;
    }
};


void RxpPointcloud::savePoints() {
    if (endOfInput()) {
        return;
    }
    for (m_dec.get(m_rxpbuf); !m_dec.eoi(); m_dec.get(m_rxpbuf))
    {
        dispatch(m_rxpbuf.begin(), m_rxpbuf.end());
        if (!m_points.empty()) return;
    }
}


void RxpPointcloud::copyPoint(const Point& from, PointRef& to) const {
    using namespace Dimension;
    to.setField(Id::X, from.target.vertex[0]);
    to.setField(Id::Y, from.target.vertex[1]);
    to.setField(Id::Z, from.target.vertex[2]);
    to.setField(getTimeDimensionId(m_syncToPps), from.target.time);
    to.setField(Id::Amplitude, from.target.amplitude);
    to.setField(Id::Reflectance, from.target.reflectance);
    to.setField(Id::ReturnNumber, from.returnNumber);
    to.setField(Id::NumberOfReturns, from.numberOfReturns);
    to.setField(Id::EchoRange, from.target.echo_range);
    to.setField(Id::Deviation, from.target.deviation);
    to.setField(Id::BackgroundRadiation, from.target.background_radiation);
    to.setField(Id::IsPpsLocked, from.target.is_pps_locked);
    to.setField(Id::EdgeOfFlightLine, from.edgeOfFlightLine ? 1 : 0);

    if (m_reflectanceAsIntensity) {
        uint16_t intensity;
        if (from.target.reflectance > m_maxReflectance) {
            intensity = (std::numeric_limits<uint16_t>::max)();
        } else if (from.target.reflectance < m_minReflectance) {
            intensity = 0;
        } else {
            intensity = uint16_t(std::roundf(double((std::numeric_limits<uint16_t>::max)()) *
                        (from.target.reflectance - m_minReflectance) / (m_maxReflectance - m_minReflectance)));
        }
        to.setField(Id::Intensity, intensity);
    }    
}


bool RxpPointcloud::endOfInput() const
{
    return m_dec.eoi();
}


void RxpPointcloud::on_echo_transformed(echo_type echo)
{
    if (!(scanlib::pointcloud::single == echo || scanlib::pointcloud::last == echo))
    {
        // Come back later, when we've got all the echos
        return;
    }

    unsigned int returnNumber = 1;
    for (scanlib::pointcloud::target_count_type i = 0; i < target_count; ++i, ++returnNumber)
    {
        //Only first return is marked as edge of flight line
        m_points.emplace_back(targets[i], returnNumber, target_count, m_edge);
        if (m_edge)
            m_edge = false;
    }
}

void RxpPointcloud::on_line_start_up(const scanlib::line_start_up<iterator_type> & arg)
{
    scanlib::pointcloud::on_line_start_up(arg);
    m_edge = true;
}

void RxpPointcloud::on_line_start_dn(const scanlib::line_start_dn<iterator_type> & arg)
{
    scanlib::pointcloud::on_line_start_dn(arg);
    m_edge = true;
}

} //pdal
