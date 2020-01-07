/******************************************************************************
 * Copyright (c) 2016, 2017, 2020 Bradley J Chambers (brad.chambers@gmail.com)
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
#include <pdal/util/ProgramArgs.hpp>

#include <memory>
#include <string>

namespace pdal
{

class Options;
class PointLayout;
class PointView;
struct NormalArgs;

struct Edge
{
    PointId m_v0;
    PointId m_v1;
    double m_weight;

    Edge(PointId i, PointId j, double weight)
        : m_v0(i), m_v1(j), m_weight(weight)
    {
    }
};

struct CompareEdgeWeight
{
    bool operator()(Edge const& lhs, Edge const& rhs)
    {
        return lhs.m_weight > rhs.m_weight;
    }
};

typedef std::vector<Edge> EdgeList;

class PDAL_DLL NormalFilter : public Filter
{
public:
    NormalFilter();
    ~NormalFilter();

    NormalFilter& operator=(const NormalFilter&) = delete;
    NormalFilter(const NormalFilter&) = delete;

    void doFilter(PointView& view, int knn = 8);

    std::string getName() const;

private:
    std::unique_ptr<NormalArgs> m_args;
    point_count_t m_count;
    Arg* m_viewpointArg;

    void compute(PointView& view, KD3Index& kdi);
    void refine(PointView& view, KD3Index& kdi);
    void
    update(PointView& view, KD3Index& kdi, std::vector<bool> inMST,
           std::priority_queue<Edge, EdgeList, CompareEdgeWeight> edge_queue,
           PointId updateIdx);

    virtual void addArgs(ProgramArgs& args);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void prepared(PointTableRef table);
    virtual void filter(PointView& view);
};

} // namespace pdal
