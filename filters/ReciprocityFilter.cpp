/******************************************************************************
 * Copyright (c) 2019, Bradley J Chambers (brad.chambers@gmail.com)
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

// PDAL implementation of the nearest-neighbor reciprocity criterion presented
// in T. Weyrich, M. Pauly, R. Keiser, S. Heinzle, S. Scandella, and M. Gross,
// “Post-processing of Scanned 3D Surface Data,” Proc. Eurographics Symp.
// Point-Based Graph. 2004, pp. 85–94, 2004.

#include "ReciprocityFilter.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <string>
#include <thread>
#include <vector>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.reciprocity",
    "Returns the percentage of neighbors that do NOT have the query point as a "
    "neighbor",
    "http://pdal.io/stages/filters.reciprocity.html"
};

CREATE_STATIC_STAGE(ReciprocityFilter, s_info)

std::string ReciprocityFilter::getName() const
{
    return s_info.name;
}

void ReciprocityFilter::addArgs(ProgramArgs& args)
{
    args.add("knn", "k-Nearest neighbors", m_knn, 8);
    args.add("threads", "Number of threads used to run this filter", m_threads,
             1);
}

void ReciprocityFilter::addDimensions(PointLayoutPtr layout)
{
    m_reciprocity =
        layout->registerOrAssignDim("Reciprocity", Dimension::Type::Double);
}

void ReciprocityFilter::filter(PointView& view)
{
    KD3Index& kdi = view.build3dIndex();

    point_count_t nloops = view.size();
    std::vector<std::thread> threadPool(m_threads);
    for (int t = 0; t < m_threads; t++)
    {
        threadPool[t] = std::thread(std::bind(
            [&](const PointId start, const PointId end, const PointId t) {
                for (PointId i = start; i < end; i++)
                    setReciprocity(view, i, kdi);
            },
            t * nloops / m_threads,
            (t + 1) == m_threads ? nloops : (t + 1) * nloops / m_threads, t));
    }
    for (auto& t : threadPool)
        t.join();
}

void ReciprocityFilter::setReciprocity(PointView& view, const PointId& i,
                                       const KD3Index& kdi)
{
    // Find k-nearest neighbors of i.
    auto ni = kdi.neighbors(i, m_knn + 1);

    // Initialize number of unidirectional neighbors to 0.
    point_count_t uni(0);

    // Visit each neighbor of i, finding its k-nearest neighbors. If i is
    // not a nearest neighbor of one of its neighbors, increment uni.
    for (auto const& j : ni)
    {
        // The query point itself will always show up as a neighbor and can
        // be skipped.
        if (j == i)
            continue;

        // Find k-nearest neighbors of j.
        auto nj = kdi.neighbors(j, m_knn + 1);

        // If i is not a neighbor of j, increment uni.
        if (std::find(nj.begin(), nj.end(), i) == nj.end())
            ++uni;
    }

    // Compute reciprocity as percentage of neighbors that do NOT contain
    // id as a neighbor.
    double reciprocity = 100.0 * uni / m_knn;
    view.setField(m_reciprocity, i, reciprocity);
}

} // namespace pdal
