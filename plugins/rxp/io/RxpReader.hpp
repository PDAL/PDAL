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
*
* This software has not been developed by RIEGL, and RIEGL will not provide any
* support for this driver. Please do not contact RIEGL with any questions or
* issues regarding this driver. RIEGL is not responsible for damages or other
* issues that arise from use of this driver. This driver has been tested
* against RiVLib version 1.39 on a Ubuntu 14.04 using gcc43.
 ****************************************************************************/

#pragma once

#include <memory>

#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include "RxpPointcloud.hpp"

namespace pdal
{


const bool DEFAULT_SYNC_TO_PPS = true;
const bool DEFAULT_IS_RDTP = false;
const bool DEFAULT_MINIMAL = false;
const bool DEFAULT_REFLECTANCE_AS_INTENSITY = true;
const float DEFAULT_MIN_REFLECTANCE = -25.0;
const float DEFAULT_MAX_REFLECTANCE = 5.0;

std::string extractRivlibURI(const Options& options);
Dimension::IdList getRxpDimensions(bool syncToPps);

class PDAL_EXPORT RxpReader : public pdal::Reader, public pdal::Streamable
{
public:
    RxpReader()
        : pdal::Reader()
        , pdal::Streamable()
        , m_syncToPps(DEFAULT_SYNC_TO_PPS)
        , m_reflectanceAsIntensity(DEFAULT_REFLECTANCE_AS_INTENSITY)
        , m_minReflectance(DEFAULT_MIN_REFLECTANCE)
        , m_maxReflectance(DEFAULT_MAX_REFLECTANCE)
        , m_pointcloud()
    {}

    std::string getName() const;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual point_count_t read(PointViewPtr view, point_count_t num);
    virtual bool processOne(PointRef& point);
    virtual void done(PointTableRef table);

    std::string m_uri;
    bool m_syncToPps;
    bool m_isRdtp;
    bool m_reflectanceAsIntensity;
    float m_minReflectance;
    float m_maxReflectance;
    std::unique_ptr<RxpPointcloud> m_pointcloud;
};


} // namespace pdal
