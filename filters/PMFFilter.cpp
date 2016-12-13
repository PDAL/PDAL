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

#include <pdal/EigenUtils.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/QuadIndex.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <Eigen/Dense>

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


void PMFFilter::addArgs(ProgramArgs& args)
{
    args.add("max_window_size", "Maximum window size", m_maxWindowSize, 33.0);
    args.add("slope", "Slope", m_slope, 1.0);
    args.add("max_distance", "Maximum distance", m_maxDistance, 2.5);
    args.add("initial_distance", "Initial distance", m_initialDistance, 0.15);
    args.add("cell_size", "Cell size", m_cellSize, 1.0);
    args.add("classify", "Apply classification labels?", m_classify, true);
    args.add("extract", "Extract ground returns?", m_extract);
    args.add("approximate", "Use approximate algorithm?", m_approximate);
}


void PMFFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::Classification);
}

std::vector<double> PMFFilter::morphOpen(PointViewPtr view, float radius)
{
    point_count_t np(view->size());

    QuadIndex idx(*view);

    std::vector<double> minZ(np), maxZ(np);

    // erode
    for (PointId i = 0; i < np; ++i)
    {
        double x = view->getFieldAs<double>(Dimension::Id::X, i);
        double y = view->getFieldAs<double>(Dimension::Id::Y, i);

        std::vector<PointId> ids = idx.getPoints(x-radius, y-radius, x+radius, y+radius);

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
        double x = view->getFieldAs<double>(Dimension::Id::X, i);
        double y = view->getFieldAs<double>(Dimension::Id::Y, i);

        std::vector<PointId> ids = idx.getPoints(x-radius, y-radius, x+radius, y+radius);

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
    // Compute the series of window sizes and height thresholds
    std::vector<float> htvec;
    std::vector<float> wsvec;
    int iter = 0;
    float ws = 0.0f;
    float ht = 0.0f;

    while (ws < m_maxWindowSize)
    {
        // Determine the initial window size.
        if (1) // exponential
            ws = m_cellSize * (2.0f * std::pow(2, iter) + 1.0f);
        else
            ws = m_cellSize * (2.0f * (iter+1) * 2 + 1.0f);

        // Calculate the height threshold to be used in the next iteration.
        if (iter == 0)
            ht = m_initialDistance;
        else
            ht = m_slope * (ws - wsvec[iter-1]) * m_cellSize + m_initialDistance;

        // Enforce max distance on height threshold
        if (ht > m_maxDistance)
            ht = m_maxDistance;

        wsvec.push_back(ws);
        htvec.push_back(ht);

        iter++;
    }

    std::vector<PointId> groundIdx;
    for (PointId i = 0; i < view->size(); ++i)
        groundIdx.push_back(i);

    // Progressively filter ground returns using morphological open
    for (size_t j = 0; j < wsvec.size(); ++j)
    {
        // Limit filtering to those points currently considered ground returns
        PointViewPtr ground = view->makeNew();
        for (auto const& i : groundIdx)
            ground->appendPoint(*view, i);

        log()->get(LogLevel::Debug) <<  "Iteration " << j
                                    << " (height threshold = " << htvec[j]
                                    << ", window size = " << wsvec[j]
                                    << ")...\n";

        // Create new cloud to hold the filtered results. Apply the
        // morphological opening operation at the current window size.
        auto maxZ = morphOpen(ground, wsvec[j]*0.5);

        // Find indices of the points whose difference between the source and
        // filtered point clouds is less than the current height threshold.
        std::vector<PointId> groundNewIdx;
        for (PointId i = 0; i < ground->size(); ++i)
        {
            double z0 = ground->getFieldAs<double>(Dimension::Id::Z, i);
            float diff = z0 - maxZ[i];
            if (diff < htvec[j])
                groundNewIdx.push_back(groundIdx[i]);
        }

        groundIdx.swap(groundNewIdx);

        log()->get(LogLevel::Debug) << "Ground now has " << groundIdx.size()
                                    << " points.\n";
    }

    return groundIdx;
}

std::vector<PointId> PMFFilter::processGroundApprox(PointViewPtr view)
{
    using namespace Eigen;

    BOX2D bounds;
    view->calculateBounds(bounds);

    double extent_x = floor(bounds.maxx) - ceil(bounds.minx);
    double extent_y = floor(bounds.maxy) - ceil(bounds.miny);

    int cols = static_cast<int>(ceil(extent_x/m_cellSize)) + 1;
    int rows = static_cast<int>(ceil(extent_y/m_cellSize)) + 1;

    // Compute the series of window sizes and height thresholds
    std::vector<float> htvec;
    std::vector<float> wsvec;
    int iter = 0;
    float ws = 0.0f;
    float ht = 0.0f;

    while (ws < m_maxWindowSize)
    {
        // Determine the initial window size.
        if (1) // exponential
            ws = m_cellSize * (2.0f * std::pow(2, iter) + 1.0f);
        else
            ws = m_cellSize * (2.0f * (iter+1) * 2 + 1.0f);

        // Calculate the height threshold to be used in the next iteration.
        if (iter == 0)
            ht = m_initialDistance;
        else
            ht = m_slope * (ws - wsvec[iter-1]) * m_cellSize + m_initialDistance;

        // Enforce max distance on height threshold
        if (ht > m_maxDistance)
            ht = m_maxDistance;

        wsvec.push_back(ws);
        htvec.push_back(ht);

        iter++;
    }

    std::vector<PointId> groundIdx;
    for (PointId i = 0; i < view->size(); ++i)
        groundIdx.push_back(i);

    MatrixXd ZImin = eigen::createMinMatrix(*view.get(), rows, cols, m_cellSize,
                                            bounds);

    // Progressively filter ground returns using morphological open
    for (size_t j = 0; j < wsvec.size(); ++j)
    {
        log()->get(LogLevel::Debug) <<  "Iteration " << j
                                    << " (height threshold = " << htvec[j]
                                    << ", window size = " << wsvec[j]
                                    << ")...\n";

        MatrixXd mo = eigen::matrixOpen(ZImin, 0.5*(wsvec[j]-1));

        std::vector<PointId> groundNewIdx;
        for (auto p_idx : groundIdx)
        {
            double x = view->getFieldAs<double>(Dimension::Id::X, p_idx);
            double y = view->getFieldAs<double>(Dimension::Id::Y, p_idx);
            double z = view->getFieldAs<double>(Dimension::Id::Z, p_idx);

            int r = static_cast<int>(std::floor((y-bounds.miny) / m_cellSize));
            int c = static_cast<int>(std::floor((x-bounds.minx) / m_cellSize));

            float diff = z - mo(r, c);
            if (diff < htvec[j])
                groundNewIdx.push_back(p_idx);
        }

        ZImin.swap(mo);
        groundIdx.swap(groundNewIdx);

        log()->get(LogLevel::Debug) << "Ground now has " << groundIdx.size()
                                    << " points.\n";
    }

    return groundIdx;
}

PointViewSet PMFFilter::run(PointViewPtr input)
{
    bool logOutput = log()->getLevel() > LogLevel::Debug1;
    if (logOutput)
        log()->floatPrecision(8);
    log()->get(LogLevel::Debug2) << "Process PMFFilter...\n";

    std::vector<PointId> idx;
    if (m_approximate)
        idx = processGroundApprox(input);
    else
        idx = processGround(input);

    PointViewSet viewSet;
    if (!idx.empty() && (m_classify || m_extract))
    {

        if (m_classify)
        {
            log()->get(LogLevel::Debug2) << "Labeled " << idx.size()
                                         << " ground returns!\n";

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
            log()->get(LogLevel::Debug2) << "Extracted " << idx.size()
                                         << " ground returns!\n";

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
