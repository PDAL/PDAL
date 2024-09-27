/******************************************************************************
* Copyright (c) 2013, Andrew Bell (andrew.bell.ia@gmail.com)
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
#include <pdal/util/ProgramArgs.hpp>

namespace hexer
{
    class BaseGrid;
};

namespace pdal
{

class PDAL_DLL HexBin : public Filter, public Streamable
{
public:
    HexBin();
    ~HexBin();

    HexBin& operator=(const HexBin&) = delete;
    HexBin(const HexBin&) = delete;

    std::string getName() const;
    hexer::BaseGrid* grid() const;

private:
    std::unique_ptr<hexer::BaseGrid> m_grid;
    uint32_t m_precision;
    uint32_t m_sampleSize;
    double m_cullArea;
    Arg *m_cullArg;
    int32_t m_density;
    double m_edgeLength;
    bool m_outputTesselation;
    bool m_doSmooth;
    point_count_t m_count;
    bool m_preserve_topology;
    std::string m_DensityOutput;
    std::string m_boundaryOutput;
    bool m_isH3;
    int m_h3Res;
    SpatialReference m_srs;
    std::string m_driver;

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void ready(PointTableRef table);
    virtual void filter(PointView& view);
    virtual bool processOne(PointRef& point);
    virtual void spatialReferenceChanged(const SpatialReference& srs);
    virtual void done(PointTableRef table);
};

} // namespace pdal
