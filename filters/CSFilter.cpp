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

// PDAL implementation of T. J. Pingel, K. C. Clarke, and W. A. McBride, “An
// improved simple morphological filter for the terrain classification of
// airborne LIDAR data,” ISPRS J. Photogramm. Remote Sens., vol. 77, pp. 21–30,
// 2013.

#include "CSFilter.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "private/DimRange.hpp"
#include "private/Segmentation.hpp"
#include "private/csf/CSF.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace pdal
{

using namespace Dimension;
using namespace Eigen;

static StaticPluginInfo const s_info{
    "filters.csf", "Cloth Simulation Filter (Zhang et al., 2016)",
    "http://pdal.io/stages/filters.csf.html"};

CREATE_STATIC_STAGE(CSFilter, s_info)

struct CSArgs
{
    bool m_smooth;
    double m_step;
    double m_threshold;
    double m_resolution;
    int m_rigid;
    int m_iterations;
    std::vector<DimRange> m_ignored;
    StringList m_returns;
};

CSFilter::CSFilter() : m_args(new CSArgs)
{
}

std::string CSFilter::getName() const
{
    return s_info.name;
}

void CSFilter::addArgs(ProgramArgs& args)
{
    args.add("smooth", "Slope postprocessing?", m_args->m_smooth, true);
    args.add("step", "Time step", m_args->m_step, 0.65);
    args.add("threshold", "Classification threshold", m_args->m_threshold, 0.5);
    args.add("resolution", "Cloth resolution", m_args->m_resolution, 1.0);
    args.add("rigidness", "Rigidness", m_args->m_rigid, 3);
    args.add("iterations", "Max iterations", m_args->m_iterations, 500);
    args.add("ignore", "Ignore values", m_args->m_ignored);
    args.add("returns", "Include last returns?", m_args->m_returns,
             {"last", "only"});
}

void CSFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Id::Classification);
}

void CSFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());

    for (auto& r : m_args->m_ignored)
    {
        r.m_id = layout->findDim(r.m_name);
        if (r.m_id == Id::Unknown)
            throwError("Invalid dimension name in 'ignored' option: '" +
                       r.m_name + "'.");
    }
    if (m_args->m_returns.size())
    {
        for (auto& r : m_args->m_returns)
        {
            Utils::trim(r);
            if ((r != "first") && (r != "intermediate") && (r != "last") &&
                (r != "only"))
            {
                throwError("Unrecognized 'returns' value: '" + r + "'.");
            }
        }

        if (!layout->hasDim(Id::ReturnNumber) ||
            !layout->hasDim(Id::NumberOfReturns))
        {
            log()->get(LogLevel::Warning) << "Could not find ReturnNumber and "
                                             "NumberOfReturns. Skipping "
                                             "segmentation of last returns and "
                                             "proceeding with all returns.\n";
            m_args->m_returns.clear();
        }
    }
}

PointViewSet CSFilter::run(PointViewPtr view)
{
    PointViewSet viewSet{view};
    if (!view->size())
        return viewSet;

    // Segment input view into ignored/kept views.
    PointViewPtr ignoredView = view->makeNew();
    PointViewPtr keptView = view->makeNew();
    if (m_args->m_ignored.empty())
        keptView->append(*view);
    else
        Segmentation::ignoreDimRanges(m_args->m_ignored, view, keptView,
                                      ignoredView);

    // Check for 0's in ReturnNumber and NumberOfReturns
    bool nrOneZero(false);
    bool rnOneZero(false);
    bool nrAllZero(true);
    bool rnAllZero(true);
    for (PointRef p : *keptView)
    {
        uint8_t nr = p.getFieldAs<uint8_t>(Id::NumberOfReturns);
        uint8_t rn = p.getFieldAs<uint8_t>(Id::ReturnNumber);
        if ((nr == 0) && !nrOneZero)
            nrOneZero = true;
        if ((rn == 0) && !rnOneZero)
            rnOneZero = true;
        if (nr != 0)
            nrAllZero = false;
        if (rn != 0)
            rnAllZero = false;
    }

    if ((nrOneZero || rnOneZero) && !(nrAllZero && rnAllZero))
        throwError("Some NumberOfReturns or ReturnNumber values were 0, but "
                   "not all. Check that all values in the input file are >= "
                   "1.");

    // Segment kept view into two views
    PointViewPtr firstView = keptView->makeNew();
    PointViewPtr secondView = keptView->makeNew();

    if (nrAllZero && rnAllZero)
    {
        log()->get(LogLevel::Warning)
            << "Both NumberOfReturns and ReturnNumber are filled with 0's. "
               "Proceeding without any further return filtering.\n";
        firstView->append(*keptView);
    }
    else
    {
        Segmentation::segmentReturns(keptView, firstView, secondView,
                                     m_args->m_returns);
    }

    if (!firstView->size())
        throwError("No returns to process.");

    csf::PointCloud csfPC;
    for (PointRef point : *firstView)
    {
        csf::Point p;
        p.x = point.getFieldAs<double>(Id::X);
        p.y = point.getFieldAs<double>(Id::Y);
        p.z = point.getFieldAs<double>(Id::Z);
        csfPC.push_back(p);
    }

    CSF c(0);
    c.params.bSloopSmooth = m_args->m_smooth;
    c.params.time_step = m_args->m_step;
    c.params.class_threshold = m_args->m_threshold;
    c.params.cloth_resolution = m_args->m_resolution;
    c.params.rigidness = m_args->m_rigid;
    c.params.interations = m_args->m_iterations;
    std::vector<int> groundIdx, offGroundIdx;
    c.setLog(log());
    c.setPointCloud(csfPC);
    try
    {
        c.do_filtering(groundIdx, offGroundIdx, true);
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }

    for (auto const& i : groundIdx)
        firstView->setField(Id::Classification, i, 2);
    for (auto const& i : offGroundIdx)
        firstView->setField(Id::Classification, i, 1);

    return viewSet;
}

} // namespace pdal
