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

#include "SkewnessBalancingFilter.hpp"

#include <pdal/PointViewIter.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info{
    "filters.skewnessbalancing", "Bartels & Wei Skewness Balancing",
    "http://pdal.io/stages/filters.skewnessbalancing.html"};

CREATE_STATIC_STAGE(SkewnessBalancingFilter, s_info)

std::string SkewnessBalancingFilter::getName() const
{
    return s_info.name;
}

void SkewnessBalancingFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::Classification);
}

std::set<PointId> SkewnessBalancingFilter::processGround(PointViewPtr view)
{
    auto cmp = [](const PointIdxRef& p1, const PointIdxRef& p2) {
        return p1.compare(Dimension::Id::Z, p2);
    };
    std::sort(view->begin(), view->end(), cmp);

    point_count_t n(0);
    point_count_t n1(0);
    double delta, delta_n, term1, M1, M2, M3;
    M1 = M2 = M3 = 0.0;
    std::vector<double> skewness(view->size());
    for (PointId i = 0; i < view->size(); ++i)
    {
        double z = view->getFieldAs<double>(Dimension::Id::Z, i);
        n1 = n;
        n++;
        delta = z - M1;
        delta_n = delta / n;
        term1 = delta * delta_n * n1;
        M1 += delta_n;
        M3 += term1 * delta_n * (n - 2) - 3 * delta_n * M2;
        M2 += term1;
        skewness[i] = std::sqrt(n) * M3 / std::pow(M2, 1.5);
    }

    PointId j(0);
    PointId i(view->size());
    do
    {
        if (skewness[i] <= 0)
        {
            j = i;
            break;
        }
    } while (--i != 0);

    std::set<PointId> groundIdx;
    for (PointId i = 0; i <= j; ++i)
        groundIdx.insert(i);

    log()->get(LogLevel::Debug)
        << "Stopped with " << groundIdx.size()
        << " ground returns and skewness of " << skewness[j] << std::endl;

    return groundIdx;
}

PointViewSet SkewnessBalancingFilter::run(PointViewPtr input)
{
    PointViewSet viewSet;
    if (!input->size())
        return viewSet;

    bool logOutput = log()->getLevel() > LogLevel::Debug1;
    if (logOutput)
        log()->floatPrecision(8);

    auto idx = processGround(input);

    if (!idx.empty())
    {
        // set the classification label of ground returns as 2
        // (corresponding to ASPRS LAS specification)
        for (const auto& i : idx)
            input->setField(Dimension::Id::Classification, i, 2);

        viewSet.insert(input);
    }
    else
    {
        if (idx.empty())
            log()->get(LogLevel::Debug2)
                << "Filtered cloud has no ground returns!\n";

        // return the input buffer unchanged
        viewSet.insert(input);
    }

    return viewSet;
}

} // namespace pdal
