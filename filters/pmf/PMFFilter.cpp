/******************************************************************************
* Copyright (c) 2015, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "PMFFilter.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/pdal_macros.hpp>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.pmf", "Progressive morphological filter",
               "http://pdal.io/stages/filters.pmf.html");

CREATE_STATIC_PLUGIN(1, 0, PMFFilter, Filter, s_info)

std::string PMFFilter::getName() const
{
    return s_info.name;
}

Options PMFFilter::getDefaultOptions()
{
    Options options;
    options.add("max_window_size", 33, "Maximum window size");
    options.add("slope", 1, "Slope");
    options.add("max_distance", 2.5, "Maximum distance");
    options.add("initial_distance", 0.15, "Initial distance");
    options.add("cell_size", 1, "Cell Size");
    options.add("classify", true, "Apply classification labels?");
    options.add("extract", false, "Extract ground returns?");
    options.add("approximate", false, "Use approximate algorithm?");
    return options;
}

void PMFFilter::processOptions(const Options& options)
{
    m_maxWindowSize = options.getValueOrDefault<double>("max_window_size", 33);
    m_slope = options.getValueOrDefault<double>("slope", 1);
    m_maxDistance = options.getValueOrDefault<double>("max_distance", 2.5);
    m_initialDistance = options.getValueOrDefault<double>("initial_distance", 0.15);
    m_cellSize = options.getValueOrDefault<double>("cell_size", 1);
    m_classify = options.getValueOrDefault<bool>("classify", true);
    m_extract = options.getValueOrDefault<bool>("extract", false);
    m_approximate = options.getValueOrDefault<bool>("approximate", false);
}

void PMFFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::Classification);
}

std::vector<double> PMFFilter::morphOpen(PointViewPtr view, float radius)
{
    point_count_t np(view->size());

    KD2Index index(*view);
    index.build();

    std::vector<double> minZ(np), maxZ(np);
    typedef std::vector<PointId> PointIdVec;
    std::map<PointId, PointIdVec> neighborMap;

    // erode
    for (PointId i = 0; i < np; ++i)
    {
        double x = view->getFieldAs<double>(Dimension::Id::X, i);
        double y = view->getFieldAs<double>(Dimension::Id::Y, i);
        auto ids = index.radius(x, y, radius);

        // neighborMap.insert(std::pair<PointId, std::vector<PointId>(i, ids));
        neighborMap[i] = ids;
        double localMin(std::numeric_limits<double>::max());
        for (auto const& j : ids)
        {
            double z = view->getFieldAs<double>(Dimension::Id::Z, j);
            if (z < localMin)
                localMin = z;
        }
        minZ[i] = localMin;
    }

    // dilate
    for (PointId i = 0; i < np; ++i)
    {
        auto ids = neighborMap[i];
        double localMax(std::numeric_limits<double>::lowest());
        for (auto const& j : ids)
        {
            double z = minZ[j];
            if (z > localMax)
                localMax = z;
        }
        maxZ[i] = localMax;
    }

    return maxZ;
}

std::vector<PointId> PMFFilter::processGround(PointViewPtr view)
{
    point_count_t np(view->size());

    // Compute the series of window sizes and height thresholds
    std::vector<float> height_thresholds;
    std::vector<float> window_sizes;
    int iteration = 0;
    float window_size = 0.0f;
    float height_threshold = 0.0f;

    while (window_size < m_maxWindowSize)
    {
        // Determine the initial window size.
        if (1) // exponential
            window_size = m_cellSize * (2.0f * std::pow(2, iteration) + 1.0f);
        else
            window_size = m_cellSize * (2.0f * (iteration+1) * 2 + 1.0f);

        // Calculate the height threshold to be used in the next iteration.
        if (iteration == 0)
            height_threshold = m_initialDistance;
        else
            height_threshold = m_slope * (window_size - window_sizes[iteration-1]) * m_cellSize + m_initialDistance;

        // Enforce max distance on height threshold
        if (height_threshold > m_maxDistance)
            height_threshold = m_maxDistance;

        window_sizes.push_back(window_size);
        height_thresholds.push_back(height_threshold);

        iteration++;
    }

    std::vector<PointId> groundIdx;
    for (PointId i = 0; i < np; ++i)
        groundIdx.push_back(i);

    // Progressively filter ground returns using morphological open
    for (size_t j = 0; j < window_sizes.size(); ++j)
    {
        // Limit filtering to those points currently considered ground returns
        PointViewPtr ground = view->makeNew();
        for (PointId i = 0; i < groundIdx.size(); ++i)
            ground->appendPoint(*view, groundIdx[i]);

        printf("      Iteration %ld (height threshold = %f, window size = %f)...",
               j, height_thresholds[j], window_sizes[j]);

        // Create new cloud to hold the filtered results. Apply the morphological
        // opening operation at the current window size.
        auto maxZ = morphOpen(ground, window_sizes[j]*0.5);

        // Find indices of the points whose difference between the source and
        // filtered point clouds is less than the current height threshold.
        std::vector<PointId> pt_indices;
        for (PointId i = 0; i < ground->size(); ++i)
        {
            double z0 = ground->getFieldAs<double>(Dimension::Id::Z, i);
            double z1 = maxZ[i];
            float diff = z0 - z1;
            if (diff < height_thresholds[j])
                pt_indices.push_back(groundIdx[i]);
        }
        groundIdx.swap(pt_indices);
    }

    return groundIdx;
}

PointViewSet PMFFilter::run(PointViewPtr input)
{
    bool logOutput = log()->getLevel() > LogLevel::Debug1;
    if (logOutput)
        log()->floatPrecision(8);
    log()->get(LogLevel::Debug2) << "Process PMFFilter...\n";

    auto idx = processGround(input);

    PointViewSet viewSet;
    if (!idx.empty() && (m_classify || m_extract))
    {

        if (m_classify)
        {
            log()->get(LogLevel::Debug2) << "Labeled " << idx.size() << " ground returns!\n";

            // set the classification label of ground returns as 2
            // (corresponding to ASPRS LAS specification)
            for (const auto& i : idx)
            {
                input->setField(Dimension::Id::Classification, i, 2);
            }

            viewSet.insert(input);
        }

        if (m_extract)
        {
            log()->get(LogLevel::Debug2) << "Extracted " << idx.size() << " ground returns!\n";

            // create new PointView containing only ground returns
            PointViewPtr output = input->makeNew();
            for (const auto& i : idx)
            {
                output->appendPoint(*input, i);
            }

            viewSet.erase(input);
            viewSet.insert(output);
        }
    }
    else
    {
        if (idx.empty())
            log()->get(LogLevel::Debug2) << "Filtered cloud has no ground returns!\n";

        if (!(m_classify || m_extract))
            log()->get(LogLevel::Debug2) << "Must choose --classify or --extract\n";

        // return the input buffer unchanged
        viewSet.insert(input);
    }

    return viewSet;
}

} // namespace pdal
