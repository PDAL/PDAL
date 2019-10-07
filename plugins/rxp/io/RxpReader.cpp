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
*
* This software has not been developed by RIEGL, and RIEGL will not provide any
* support for this driver. Please do not contact RIEGL with any questions or
* issues regarding this driver. RIEGL is not responsible for damages or other
* issues that arise from use of this driver. This driver has been tested
* against RiVLib version 1.39 on a Ubuntu 14.04 using gcc43.
****************************************************************************/

#include "RxpReader.hpp"

#include <pdal/util/ProgramArgs.hpp>
#include <pdal/PDALUtils.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "readers.rxp",
    "RXP Reader",
    "http://pdal.io/stages/readers.rxp.html"
};

CREATE_SHARED_STAGE(RxpReader, s_info)

std::string RxpReader::getName() const { return s_info.name; }

Dimension::IdList getRxpDimensions(bool syncToPps, bool reflectanceAsIntensity)
{
    using namespace Dimension;
    Dimension::IdList ids;

    ids.push_back(Id::X);
    ids.push_back(Id::Y);
    ids.push_back(Id::Z);
    ids.push_back(getTimeDimensionId(syncToPps));
    ids.push_back(Id::ReturnNumber);
    ids.push_back(Id::NumberOfReturns);
    ids.push_back(Id::Amplitude);
    ids.push_back(Id::Reflectance);
    ids.push_back(Id::EchoRange);
    ids.push_back(Id::Deviation);
    ids.push_back(Id::BackgroundRadiation);
    ids.push_back(Id::IsPpsLocked);
    ids.push_back(Id::EdgeOfFlightLine);
    if (reflectanceAsIntensity) {
        ids.push_back(Id::Intensity);
    }

    return ids;
}


void RxpReader::addArgs(ProgramArgs& args)
{
    args.add("rdtp", "", m_isRdtp, DEFAULT_IS_RDTP);
    args.add("sync_to_pps", "Sync to PPS", m_syncToPps, DEFAULT_SYNC_TO_PPS);
    args.add("reflectance_as_intensity", "Reflectance as intensity", m_reflectanceAsIntensity, DEFAULT_REFLECTANCE_AS_INTENSITY);
    args.add("min_reflectance", "Minimum reflectance", m_minReflectance, DEFAULT_MIN_REFLECTANCE);
    args.add("max_reflectance", "Maximum reflectance", m_maxReflectance, DEFAULT_MAX_REFLECTANCE);
}

void RxpReader::initialize()
{

    if (pdal::Utils::isRemote(m_filename))
        m_filename = pdal::Utils::fetchRemote(m_filename);

    m_uri = m_isRdtp ? "rdtp://" + m_filename : "file:" + m_filename;
}


void RxpReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims(getRxpDimensions(m_syncToPps, m_reflectanceAsIntensity));
}


void RxpReader::ready(PointTableRef table)
{
    m_pointcloud.reset(new RxpPointcloud(m_uri, m_syncToPps, m_reflectanceAsIntensity, m_minReflectance, m_maxReflectance, table));
}


point_count_t RxpReader::read(PointViewPtr view, point_count_t num)
{
    point_count_t numRead = 0;
    PointRef point(view->point(0));
    while (numRead < num && !m_pointcloud->endOfInput()) {
        point.setPointId(numRead);
        processOne(point);
        ++numRead;
    }
    return numRead;
}


bool RxpReader::processOne(PointRef& point)
{
    return m_pointcloud->readOne(point);
}


void RxpReader::done(PointTableRef table)
{
    m_pointcloud.reset();
}


} // namespace pdal
