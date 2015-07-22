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


Dimension::Id::Enum getTimeDimensionId(bool syncToPps)
{
    return syncToPps ? Dimension::Id::GpsTime : Dimension::Id::InternalTime;
}


RxpPointcloud::RxpPointcloud(
        const std::string& uri,
        bool syncToPps,
        bool minimal,
        PointTableRef table)
    : scanlib::pointcloud(syncToPps)
    , m_view(new PointView(table))
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


point_count_t RxpPointcloud::read(PointViewPtr view, point_count_t count)
{
    point_count_t numRead = 0;

    if (m_idx < m_view->size())
    {
        numRead += writeSavedPoints(view, count);
        if (numRead == count)
            return numRead;
    }

    for (m_dec.get(m_rxpbuf); !m_dec.eoi(); m_dec.get(m_rxpbuf))
    {
        dispatch(m_rxpbuf.begin(), m_rxpbuf.end());
        if (m_view->size() - m_idx + numRead >= count) break;
    }

    numRead += writeSavedPoints(view, count - numRead);

    return numRead;
}


point_count_t RxpPointcloud::writeSavedPoints(PointViewPtr view, point_count_t count)
{
    point_count_t numRead = 0;
    while (m_idx < m_view->size() && numRead < count)
    {
        view->appendPoint(*m_view, m_idx);
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

    point_count_t idx = m_view->size();
    unsigned int returnNumber = 1;
    Id::Enum timeId = getTimeDimensionId(m_syncToPps);
    for (const auto& t : targets)
    {
        m_view->setField(Id::X, idx, t.vertex[0]);
        m_view->setField(Id::Y, idx, t.vertex[1]);
        m_view->setField(Id::Z, idx, t.vertex[2]);
        m_view->setField(timeId, idx, t.time);
        if (!m_minimal)
        {
            m_view->setField(Id::Amplitude, idx, t.amplitude);
            m_view->setField(Id::Reflectance, idx, t.reflectance);
            m_view->setField(Id::ReturnNumber, idx, returnNumber);
            m_view->setField(Id::NumberOfReturns, idx, targets.size());
            m_view->setField(Id::EchoRange, idx, t.echo_range);
            m_view->setField(Id::Deviation, idx, t.deviation);
            m_view->setField(Id::BackgroundRadiation, idx, t.background_radiation);
            m_view->setField(Id::IsPpsLocked, idx, t.is_pps_locked);
        }
        ++idx, ++returnNumber;
    }
}


}
