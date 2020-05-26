/******************************************************************************
 * Copyright (c) 2016-2017, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <memory>
#include <string>

namespace pdal
{

struct SMRArgs;

class PDAL_DLL SMRFilter : public Filter
{
public:
    SMRFilter();
    ~SMRFilter();
    SMRFilter& operator=(const SMRFilter&) = delete;
    SMRFilter(const SMRFilter&) = delete;

    std::string getName() const;

private:
    int m_rows;
    int m_cols;
    BOX2D m_bounds;
    SpatialReference m_srs;
    std::unique_ptr<SMRArgs> m_args;

    virtual void addArgs(ProgramArgs& args);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void prepared(PointTableRef table);
    virtual void ready(PointTableRef table);
    virtual PointViewSet run(PointViewPtr view);

    void classifyGround(PointViewPtr, std::vector<double>&);
    std::vector<int> createLowMask(std::vector<double> const&);
    std::vector<int> createNetMask();
    std::vector<int> createObjMask(std::vector<double> const&);
    std::vector<double> createZImin(PointViewPtr view);
    std::vector<double> createZInet(std::vector<double> const&,
                                    std::vector<int> const&);
    std::vector<double> createZIpro(PointViewPtr, std::vector<double> const&,
                                    std::vector<int> const&,
                                    std::vector<int> const&,
                                    std::vector<int> const&);
    void knnfill(PointViewPtr, std::vector<double>&);
    std::vector<int> progressiveFilter(std::vector<double> const&, double,
                                       double);
};

} // namespace pdal
