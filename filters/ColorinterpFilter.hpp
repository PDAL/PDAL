/******************************************************************************
* Copyright (c) 2016, Howard Butler, howard@hobu.co
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
#include <filters/StatsFilter.hpp>

#include <map>

namespace pdal
{

namespace gdal { class Raster; }

// Interpolates color ramp into Red, Green, and Blue dimensions
// for a given dimension
// specified dimensions. It also supports scaling the data by a multiplier
// on a per-dimension basis.
class PDAL_DLL ColorinterpFilter : public Filter, public Streamable
{
public:

    ColorinterpFilter()
        : m_interpDim(Dimension::Id::Z)
        , m_interpDimString("Z")
        , m_min(0.0)
        , m_max(0.0)
        , m_clamp(false)
        , m_rampFilename("/vsimem/colorramp.png")
        , m_invertRamp(false)
        , m_stdDevThreshold(0.0)
        , m_useMAD(false)
        , m_madMultiplier(1.4862)
    {}
    ColorinterpFilter& operator=(const ColorinterpFilter&) = delete;
    ColorinterpFilter(const ColorinterpFilter&) = delete;
    std::string getName() const;

    virtual bool pipelineStreamable() const;
private:
    virtual void addArgs(ProgramArgs& args);
    virtual void filter(PointView& view);
    virtual void prepared(PointTableRef table);
    virtual void ready(PointTableRef table);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual bool processOne(PointRef& point);

    Dimension::Id m_interpDim;
    std::string m_interpDimString;
    double m_min;
    double m_max;
    bool m_clamp;
    std::string m_colorramp;
    std::shared_ptr<gdal::Raster> m_raster;
    std::string m_rampFilename;
    std::vector<uint8_t> m_redBand;
    std::vector<uint8_t> m_greenBand;
    std::vector<uint8_t> m_blueBand;
    bool m_invertRamp;
    double m_stdDevThreshold;
    bool m_useMAD;
    double m_madMultiplier;
};

} // namespace pdal
