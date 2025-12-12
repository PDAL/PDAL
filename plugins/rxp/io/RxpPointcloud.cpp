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


Point::Point(
        scanlib::target target,
        unsigned int returnNumber,
        unsigned int numberOfReturns,
        bool edgeOfFlightLine,
        double beamOriginX,
        double beamOriginY,
        double beamOriginZ,
        double beamDirectionX,
        double beamDirectionY,
        double beamDirectionZ,
        float roll,
        float pitch,
        double shotTimestamp,
        unsigned int facet,
        unsigned int segment,
        double unambiguousRange)
    : target(target)
    , returnNumber(returnNumber)
    , numberOfReturns(numberOfReturns)
    , edgeOfFlightLine(edgeOfFlightLine)
    , beamOriginX(beamOriginX)
    , beamOriginY(beamOriginY)
    , beamOriginZ(beamOriginZ)
    , beamDirectionX(beamDirectionX)
    , beamDirectionY(beamDirectionY)
    , beamDirectionZ(beamDirectionZ)
    , roll(roll)
    , pitch(pitch)
    , shotTimestamp(shotTimestamp)
    , facet(facet)
    , segment(segment)
    , unambiguousRange(unambiguousRange)
{}


RxpPointcloud::RxpPointcloud(
        const std::string& uri,
        bool syncToPps,
        bool reflectanceAsIntensity,
        bool emitEmptyShots,
        float minReflectance,
        float maxReflectance,
    PointTableRef table)
    : scanlib::pointcloud(syncToPps)
    , m_syncToPps(syncToPps)
    , m_reflectanceAsIntensity(reflectanceAsIntensity)
    , m_emitEmptyShots(emitEmptyShots)
    , m_minReflectance(minReflectance)
    , m_maxReflectance(maxReflectance)
    , m_rc(scanlib::basic_rconnection::create(uri))
    , m_dec(m_rc)
    , m_edge(false)
    , m_lastSegment(0)
    , m_rotationId(0)
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
    to.setField(Id::EchoRange, from.target.echo_range);
    to.setField(Id::Amplitude, from.target.amplitude);
    to.setField(Id::Reflectance, from.target.reflectance);
    to.setField(Id::Deviation, from.target.deviation);
    to.setField(Id::BackgroundRadiation, from.target.background_radiation);
    to.setField(Id::IsPpsLocked, from.target.is_pps_locked);
    to.setField(getTimeDimensionId(m_syncToPps), from.target.time);

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
    
    // Shot metadata (always valid)
    to.setField(Id::ShotTimestamp, from.shotTimestamp);
    to.setField(Id::Segment, from.segment);
    to.setField(Id::Facet, from.facet);
    to.setField(Id::UnambiguousRange, from.unambiguousRange);
    to.setField(Id::ReturnNumber, from.returnNumber);
    to.setField(Id::NumberOfReturns, from.numberOfReturns);
    to.setField(Id::EdgeOfFlightLine, from.edgeOfFlightLine ? 1 : 0);

    to.setField(Id::BeamDirectionX, from.beamDirectionX);
    to.setField(Id::BeamDirectionY, from.beamDirectionY);
    to.setField(Id::BeamDirectionZ, from.beamDirectionZ);
    to.setField(Id::BeamOriginX, from.beamOriginX);
    to.setField(Id::BeamOriginY, from.beamOriginY);
    to.setField(Id::BeamOriginZ, from.beamOriginZ);
    to.setField(Id::Roll, from.roll);
    to.setField(Id::Pitch, from.pitch);

}


bool RxpPointcloud::endOfInput() const
{
    return m_dec.eoi();
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

void RxpPointcloud::on_line_stop(const scanlib::line_stop<iterator_type> & arg)
{
    scanlib::pointcloud::on_line_stop(arg);
    m_edge = true;
}

void RxpPointcloud::on_hk_incl(const scanlib::hk_incl<iterator_type>& arg) {
    scanlib::pointcloud::on_hk_incl(arg);
    m_roll = (float)arg.ROLL * 0.001;
    m_pitch = (float)arg.PITCH * 0.001;
}

void RxpPointcloud::on_shot_end()
{
    if (segment < m_lastSegment)
    {
        m_rotationId++;
        m_edge = true;
    }
    m_lastSegment = segment;
    
    unsigned int returnNumber = 1;
    for (scanlib::pointcloud::target_count_type i = 0; i < target_count; ++i, ++returnNumber)
    {
        m_points.emplace_back(
            targets[i], returnNumber, target_count, m_edge,
            beam_origin[0], beam_origin[1], beam_origin[2],
            beam_direction[0], beam_direction[1], beam_direction[2],
            m_roll, m_pitch,
            time, facet, segment, unambiguous_range);
        if (m_edge)
            m_edge = false;
    }
}

void RxpPointcloud::on_gap()
{
    
    if (segment < m_lastSegment)
    {
        m_rotationId++;
        m_edge = true;
    }
    m_lastSegment = segment;
    
    if (!m_emitEmptyShots)
        return;
    
    // Create empty target with NaN for invalid measurements
    // Keep valid metadata (timestamps, beam geometry) for shot identification
    scanlib::target emptyTarget;
    emptyTarget.vertex[0] = std::numeric_limits<double>::quiet_NaN();
    emptyTarget.vertex[1] = std::numeric_limits<double>::quiet_NaN();
    emptyTarget.vertex[2] = std::numeric_limits<double>::quiet_NaN();
    emptyTarget.echo_range = std::numeric_limits<double>::quiet_NaN();
    emptyTarget.amplitude = std::numeric_limits<float>::quiet_NaN();
    emptyTarget.reflectance = std::numeric_limits<float>::quiet_NaN();
    emptyTarget.deviation = std::numeric_limits<float>::quiet_NaN();
    emptyTarget.background_radiation = std::numeric_limits<float>::quiet_NaN();
    emptyTarget.is_pps_locked = 0;
    emptyTarget.time = time; 
    emptyTarget.time_sorg = time_sorg;
    emptyTarget.segment = segment;
    emptyTarget.facet = facet;
    
    m_points.emplace_back(
        emptyTarget, 0, 0, m_edge,
        beam_origin[0], beam_origin[1], beam_origin[2],
        beam_direction[0], beam_direction[1], beam_direction[2],
        m_roll, m_pitch,
        time, facet, segment, unambiguous_range);
    if (m_edge)
        m_edge = false;
}

} //pdal
