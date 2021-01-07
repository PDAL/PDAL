/******************************************************************************
* Copyright (c) 2016, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "EstimateRankFilter.hpp"

#include <string>

#include <pdal/KDIndex.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/private/MathUtils.hpp>

namespace pdal
{

using namespace Dimension;

static StaticPluginInfo const s_info
{
    "filters.estimaterank",
    "Computes the rank of a neighborhood of points.",
    "http://pdal.io/stages/filters.estimaterank.html"
};

CREATE_STATIC_STAGE(EstimateRankFilter, s_info)

std::string EstimateRankFilter::getName() const
{
    return s_info.name;
}


void EstimateRankFilter::addArgs(ProgramArgs& args)
{
    args.add("knn", "k-Nearest Neighbors", m_knn, 8);
    args.add("thresh", "Threshold", m_thresh, 0.01);
}


void EstimateRankFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Id::Rank);
}

void EstimateRankFilter::filter(PointView& view)
{
    const KD3Index& kdi = view.build3dIndex();

    for (PointRef p : view)
    {
        PointIdList ids = kdi.neighbors(p, m_knn);
        p.setField(Id::Rank, math::computeRank(view, ids, m_thresh));
    }
}

} // namespace pdal
