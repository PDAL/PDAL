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

#include "KDistanceFilter.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/pdal_macros.hpp>

#include <string>
#include <vector>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.kdistance", "K-Distance Filter",
               "http://pdal.io/stages/filters.kdistance.html");

CREATE_STATIC_PLUGIN(1, 0, KDistanceFilter, Filter, s_info)

std::string KDistanceFilter::getName() const
{
    return s_info.name;
}

void KDistanceFilter::addArgs(ProgramArgs& args)
{
    args.add("k", "k neighbors", m_k, 10);
}

void KDistanceFilter::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;
    m_kdist = layout->registerOrAssignDim("KDistance", Type::Double);
}

void KDistanceFilter::filter(PointView& view)
{
    using namespace Dimension;
    
    // Build the 3D KD-tree.
    log()->get(LogLevel::Debug) << "Building 3D KD-tree...\n";
    KD3Index index(view);
    index.build();
    
    // Increment the minimum number of points, as knnSearch will be returning
    // the neighbors along with the query point.
    m_k++;
  
    // Compute the k-distance for each point. The k-distance is the Euclidean
    // distance to k-th nearest neighbor.
    log()->get(LogLevel::Debug) << "Computing k-distances...\n";
    for (PointId i = 0; i < view.size(); ++i)
    {
        std::vector<PointId> indices(m_k);
        std::vector<double> sqr_dists(m_k);
        index.knnSearch(i, m_k, &indices, &sqr_dists);
        view.setField(m_kdist, i, std::sqrt(sqr_dists[m_k-1]));
    }
}

} // namespace pdal
