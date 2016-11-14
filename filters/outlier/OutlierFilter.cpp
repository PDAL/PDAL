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

#include "OutlierFilter.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <string>
#include <vector>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.outlier", "Outlier removal",
               "http://pdal.io/stages/filters.outlier.html");

CREATE_STATIC_PLUGIN(1, 0, OutlierFilter, Filter, s_info)

std::string OutlierFilter::getName() const
{
    return s_info.name;
}


void OutlierFilter::addArgs(ProgramArgs& args)
{
    args.add("method", "Method [default: statistical]", m_method,
        "statistical");
    args.add("min_k", "Minimum number of neighbors in radius", m_minK, 2);
    args.add("radius", "Radius", m_radius, 1.0);
    args.add("mean_k", "Mean number of neighbors", m_meanK, 8);
    args.add("multiplier", "Standard deviation threshold", m_multiplier, 2.0);
    args.add("classify", "Apply classification labels?", m_classify, true);
    args.add("extract", "Extract ground returns?", m_extract);
}


void OutlierFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::Classification);
}


Indices OutlierFilter::processRadius(PointViewPtr inView)
{
    KD3Index index(*inView);
    index.build();

    point_count_t np = inView->size();

    std::vector<PointId> inliers, outliers;

    for (PointId i = 0; i < np; ++i)
    {
        auto ids = index.radius(i, m_radius);
        if (ids.size() > size_t(m_minK))
            inliers.push_back(i);
        else
            outliers.push_back(i);
    }

    return Indices{inliers, outliers};
}


Indices OutlierFilter::processStatistical(PointViewPtr inView)
{
    KD3Index index(*inView);
    index.build();

    point_count_t np = inView->size();

    std::vector<PointId> inliers, outliers;

    std::vector<double> distances(np);
    for (PointId i = 0; i < np; ++i)
    {
        // we increase the count by one because the query point itself will
        // be included with a distance of 0
        point_count_t count = m_meanK + 1;

        std::vector<PointId> indices(count);
        std::vector<double> sqr_dists(count);
        index.knnSearch(i, count, &indices, &sqr_dists);

        double dist_sum = 0.0;
        for (auto const& d : sqr_dists)
            dist_sum += sqrt(d);
        distances[i] = dist_sum / m_meanK;
    }

    double sum = 0.0, sq_sum = 0.0;
    for (auto const& d : distances)
    {
        sum += d;
        sq_sum += d * d;
    }
    double mean = sum / np;
    double variance = (sq_sum - sum * sum / np) / (np - 1);
    double stdev = sqrt(variance);
    double threshold = mean + m_multiplier * stdev;

    for (PointId i = 0; i < np; ++i)
    {
        if (distances[i] < threshold)
            inliers.push_back(i);
        else
            outliers.push_back(i);
    }

    return Indices{inliers, outliers};
}


PointViewSet OutlierFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;
    if (!inView->size())
        return viewSet;

    Indices indices;
    if (Utils::iequals(m_method, "statistical"))
    {
        indices = processStatistical(inView);
    }
    else if (Utils::iequals(m_method, "radius"))
    {
        indices = processRadius(inView);
    }
    else
    {
        log()->get(LogLevel::Warning) << "Requested method is unrecognized. "
                                      << "Please choose from \"statistical\" " << "or \"radius\".\n";
        viewSet.insert(inView);
        return viewSet;
    }

    if (indices.inliers.empty())
    {
        log()->get(LogLevel::Warning) << "Requested filter would remove all "
                                      << "points. Try a larger radius/smaller " << "minimum neighbors.\n";
        viewSet.insert(inView);
        return viewSet;
    }

    if (!indices.outliers.empty() && (m_classify || m_extract))
    {
        if (m_classify)
        {
            log()->get(LogLevel::Debug2) << "Labeled "
                                         << indices.outliers.size()
                                         << " outliers as noise!\n";

            // set the classification label of outlier returns as 18
            // (corresponding to ASPRS LAS specification for high noise)
            for (const auto& i : indices.outliers)
                inView->setField(Dimension::Id::Classification, i, 18);

            viewSet.insert(inView);
        }

        if (m_extract)
        {
            log()->get(LogLevel::Debug2) << "Extracted "
                                         << indices.inliers.size()
                                         << " inliers!\n";

            // create new PointView containing only outliers
            PointViewPtr output = inView->makeNew();
            for (const auto& i : indices.inliers)
                output->appendPoint(*inView, i);

            viewSet.erase(inView);
            viewSet.insert(output);
        }
    }
    else
    {
        if (indices.outliers.empty())
            log()->get(LogLevel::Warning) << "Filtered cloud has no "
                                          << "outliers!\n";

        if (!(m_classify || m_extract))
            log()->get(LogLevel::Warning) << "Must choose --classify or "
                                          << "--extract\n";

        // return the input buffer unchanged
        viewSet.insert(inView);
    }

    return viewSet;
}

} // namespace pdal
