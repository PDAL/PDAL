/******************************************************************************
* Copyright (c) 2020, Julian Fell (hi@jtfell.com)
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

#include <cstdint>
#include <memory>
#include <string>

namespace pdal
{

namespace gdal { class Raster; }
class Options;
class PointLayout;
class PointView;

class PDAL_DLL HagDemFilter : public Filter, public Streamable
{
public:
    HagDemFilter();
    HagDemFilter& operator=(const HagDemFilter&) = delete;
    HagDemFilter(const HagDemFilter&) = delete;

    std::string getName() const;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void prepared(PointTableRef table);
    virtual void ready(PointTableRef table);
    virtual void filter(PointView& view);
    virtual bool processOne(PointRef& point);

    std::unique_ptr<gdal::Raster> m_raster;
    std::string m_rasterName;
    bool m_zeroGround;
    int32_t m_band;
};

} // namespace pdal
