/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
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

#include <boost/array.hpp>

#include <gdal.h>
#include <ogr_spatialref.h>
#include <pdal/GDALUtils.hpp>

#include <map>

extern "C" int32_t ColorizationFilter_ExitFunc();
extern "C" PF_ExitFunc ColorizationFilter_InitPlugin();

namespace pdal
{

// Provides GDAL-based raster overlay that places output data in
// specified dimensions. It also supports scaling the data by a multiplier
// on a per-dimension basis.
class PDAL_DLL ColorizationFilter : public Filter
{
public:
    struct BandInfo
    {
        BandInfo(const std::string& name, uint32_t band, double scale) :
            m_name(name), m_band(band), m_scale(scale),
            m_dim(Dimension::Id::Unknown)
        {}

        BandInfo() : m_band(0), m_scale(1.0), m_dim(Dimension::Id::Unknown)
        {}

        std::string m_name;
        uint32_t m_band;
        double m_scale;
        Dimension::Id::Enum m_dim;
    };


    ColorizationFilter()
    {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();

private:
    virtual void initialize();
    virtual void processOptions(const Options&);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual void filter(PointView& view);

    std::string m_rasterFilename;
    std::vector<BandInfo> m_bands;

    std::unique_ptr<gdal::Raster> m_raster;

    ColorizationFilter& operator=(const ColorizationFilter&); // not implemented
    ColorizationFilter(const ColorizationFilter&); // not implemented
};

} // namespace pdal
