/******************************************************************************
* Copyright (c) 2017, Hobu Inc. <hobu.inc@gmail.com>
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
#include <pdal/KDIndex.hpp>

namespace pdal
{

struct DimRange;

class PDAL_DLL NeighborClassifierFilter : public Filter
{
public:
    NeighborClassifierFilter();
    NeighborClassifierFilter& operator=(const NeighborClassifierFilter&) = delete;
    NeighborClassifierFilter(const NeighborClassifierFilter&) = delete;
    ~NeighborClassifierFilter();

    std::string getName() const { return "filters.neighborclassifier"; }

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void prepared(PointTableRef table);
    virtual void filter(PointView& view);
    virtual void ready(PointTableRef table);

    bool doOne(PointRef& point, PointRef& temp, KD3Index &kdi);
    void doOneNoDomain(PointRef &point, PointRef& temp, KD3Index &kdi);
    PointViewPtr loadSet(const std::string &candFileName, PointTableRef table);

    std::vector<DimRange> m_domain;
    int m_k;
    std::map<PointId, int> m_newClass;
    std::string m_candidateFile;
};

} // namespace pdal
