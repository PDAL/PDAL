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

#include "SampleFilter.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <string>
#include <vector>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.sample", "Subsampling filter",
               "http://pdal.io/stages/filters.sample.html");

CREATE_STATIC_PLUGIN(1, 0, SampleFilter, Filter, s_info)

std::string SampleFilter::getName() const
{
    return s_info.name;
}


void SampleFilter::addArgs(ProgramArgs& args)
{
    args.add("radius", "Radius", m_radius, 1.0);
}


void SampleFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::Classification);
}


PointViewSet SampleFilter::run(PointViewPtr inView)
{
    point_count_t np = inView->size();

    // Return empty PointViewSet if the input PointView has no points.
    // Otherwise, make a new output PointView.
    PointViewSet viewSet;
    if (!np)
        return viewSet;
    PointViewPtr outView = inView->makeNew();

    // Build the 3D KD-tree.
    KD3Index index(*inView);
    index.build();

    // The result looks much better if we take some time to shuffle the indices.
    std::srand(std::time(NULL));
    std::vector<PointId> indices(np);
    for (PointId i = 0; i < np; ++i)
        indices[i] = i;
    std::random_shuffle(indices.begin(), indices.end());

    // All points are marked as kept (1) by default. As they are masked by
    // neighbors within the user-specified radius, their value is changed to 0.
    std::vector<int> keep(np, 1);

    // We are able to subsample in a single pass over the shufflled indices.
    for (auto const& i : indices)
    {
        // If a point is masked, it is forever masked, and cannot be part of the
        // sampled cloud. Otherwise, the current index is appended to the output
        // PointView.
        if (keep[i] == 0)
            continue;
        outView->appendPoint(*inView, i);

        // We now proceed to mask all neighbors within m_radius of the kept
        // point.
        auto ids = index.radius(i, m_radius);
        for (PointId j = 1; j < ids.size(); ++j)
            keep[ids[j]] = 0;
    }

    // Simply calculate the percentage of retained points.
    double frac = (double)outView->size() / (double)inView->size();
    log()->get(LogLevel::Debug2) << "Retaining "
                                 << outView->size() << " of "
                                 << inView->size() << " points ("
                                 << 100*frac << "%)\n";

    viewSet.insert(outView);
    return viewSet;
}

} // namespace pdal
