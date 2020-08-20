/******************************************************************************
 * Copyright (c) 2020, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "RelaxationDartThrowing.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/Utils.hpp>

#include <chrono>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace pdal
{

static StaticPluginInfo const s_info{
    "filters.relaxationdartthrowing", "Subsampling filter",
    "http://pdal.io/stages/filters.relaxationdartthrowing.html"};

CREATE_STATIC_STAGE(RelaxationDartThrowing, s_info)

std::string RelaxationDartThrowing::getName() const
{
    return s_info.name;
}

void RelaxationDartThrowing::addArgs(ProgramArgs& args)
{
    args.add("decay", "Decay rate", m_decay, 0.9);
    args.add("radius", "Minimum radius (initial)", m_startRadius, 1.0);
    args.add("count", "Target number of points after sampling", m_maxSize,
             (point_count_t)1000);
    args.add("shuffle", "Shuffle points prior to sampling?", m_shuffle, true);
    m_seedArg = &args.add("seed", "Random number generator seed", m_seed);
}

PointViewSet RelaxationDartThrowing::run(PointViewPtr inView)
{
    point_count_t np = inView->size();

    // Return empty PointViewSet if the input PointView has no points.
    // Otherwise, make a new output PointView.
    PointViewSet viewSet;
    if (!np)
        return viewSet;
    PointViewPtr outView = inView->makeNew();

    // Build the 3D KD-tree.
    KD3Index& index = inView->build3dIndex();

    PointIdList finalIds;

    double radius(m_startRadius);

    PointIdList shuffledIds(np);
    std::iota(shuffledIds.begin(), shuffledIds.end(), 0);
    if (m_shuffle)
    {
        if (!m_seedArg->set())
            m_seed =
                std::chrono::system_clock::now().time_since_epoch().count();
        std::shuffle(shuffledIds.begin(), shuffledIds.end(),
                     std::mt19937(m_seed));
    }

    while (finalIds.size() < m_maxSize)
    {
        // All points are marked as kept (1) by default. As they are masked by
        // neighbors within the user-specified radius, their value is changed to
        // 0.
        std::vector<int> keep(np, 1);

        // Start by masking all points within radius of finalIds.
        for (PointId i : finalIds)
        {
            PointIdList ids = index.radius(i, radius);
            for (PointId const& id : ids)
                keep[id] = 0;
        }

        // We are able to subsample in a single pass over the shuffled indices.
        for (PointId const& i : shuffledIds)
        {
            // If a point is masked, it is forever masked, and cannot be part of
            // the sampled cloud. Otherwise, the current index is appended to
            // the output PointView.
            if (keep[i] == 0)
                continue;
            finalIds.push_back(i);

            if (finalIds.size() == m_maxSize)
                break;

            // We now proceed to mask all neighbors within radius of the
            // kept point.
            PointIdList ids = index.radius(i, radius);
            for (PointId const& id : ids)
                keep[id] = 0;
        }

        if (finalIds.size() < m_maxSize)
        {
            radius = m_decay * radius;
            log()->get(LogLevel::Debug)
                << "Currently have " << finalIds.size()
                << " ids, reducing radius to " << radius << std::endl;
        }
    }

    for (PointId i : finalIds)
        outView->appendPoint(*inView, i);

    // Simply calculate the percentage of retained points.
    double frac = (double)outView->size() / (double)inView->size();
    log()->get(LogLevel::Debug2)
        << "Retaining " << outView->size() << " of " << inView->size()
        << " points (" << 100 * frac << "%)\n";

    viewSet.insert(outView);
    return viewSet;
}

} // namespace pdal
