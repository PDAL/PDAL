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

#include <pdal/StageFactory.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.rxp",
    "RXP Reader",
    "http://pdal.io/stages/readers.rxp.html" );

CREATE_SHARED_PLUGIN(1, 0, RxpReader, Reader, s_info)

std::string RxpReader::getName() const { return s_info.name; }

std::string extractRivlibURI(const Options& options)
{
    if (options.hasOption("filename"))
    {
        if (options.hasOption("rdtp"))
        {
            throw pdal_error("Cannot create URI when both filename "
                "and rdtp are provided");
        }
        return "file:" + options.getValueOrThrow<std::string>("filename");
    }
    else
    {
        return "rdtp://" + options.getValueOrThrow<std::string>("rdtp");
    }
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


Options RxpReader::getDefaultOptions()
{
    Options options;
    options.add("sync_to_pps", DEFAULT_SYNC_TO_PPS, "");
    options.add("minimal", DEFAULT_MINIMAL, "");
    return options;
}


void RxpReader::processOptions(const Options& options)
{
    m_uri = extractRivlibURI(options);
    m_syncToPps = options.getValueOrDefault<bool>("sync_to_pps",
                                                  DEFAULT_SYNC_TO_PPS);
}


void RxpReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims(getRxpDimensions(m_syncToPps, m_minimal));
}


void RxpReader::ready(PointTableRef table)
{
    m_pointcloud.reset(new RxpPointcloud(m_uri, m_syncToPps, m_minimal, table));
}


point_count_t RxpReader::read(PointViewPtr view, point_count_t count)
{
    point_count_t numRead = m_pointcloud->read(view, count);
    return numRead;
}


void RxpReader::done(PointTableRef table)
{
    m_pointcloud.reset();
}


} // namespace pdal
