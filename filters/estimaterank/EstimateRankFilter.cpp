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

#include <pdal/Eigen.hpp>
#include <pdal/KDIndex.hpp>
#include <pdal/pdal_macros.hpp>

#include <string>
#include <vector>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.estimaterank", "EstimateRank Filter", 
               "http://pdal.io/stages/filters.estimaterank.html");

CREATE_STATIC_PLUGIN(1, 0, EstimateRankFilter, Filter, s_info)

std::string EstimateRankFilter::getName() const
{
    return s_info.name;
}

Options EstimateRankFilter::getDefaultOptions()
{
    Options options;
    options.add("knn", 8, "k-Nearest Neighbors");
    options.add("thresh", 0.01, "Threshold");
    return options;
}

void EstimateRankFilter::processOptions(const Options& options)
{
    m_knn = options.getValueOrDefault<int>("knn", 8);
    m_thresh = options.getValueOrDefault<double>("thresh", 0.01);
}

void EstimateRankFilter::addDimensions(PointLayoutPtr layout)
{
    m_rank = layout->registerOrAssignDim("Rank", Dimension::Type::Unsigned8);
}

void EstimateRankFilter::filter(PointView& view)
{
    KD3Index kdi(view);
    kdi.build();

    for (PointId i = 0; i < view.size(); ++i)
    {
        // find the k-nearest neighbors
        double x = view.getFieldAs<double>(Dimension::Id::X, i);
        double y = view.getFieldAs<double>(Dimension::Id::Y, i);
        double z = view.getFieldAs<double>(Dimension::Id::Z, i);
        auto ids = kdi.neighbors(x, y, z, m_knn);

        view.setField(m_rank, i, computeRank(view, ids, m_thresh));
    }
}

} // namespace pdal
