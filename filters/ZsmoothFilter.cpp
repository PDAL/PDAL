/******************************************************************************
 * Copyright (c) 2020, University Nevada, Reno
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

#include <pdal/KDIndex.hpp>

#include "ZsmoothFilter.hpp"

namespace pdal
{

static PluginInfo const ptstatInfo
{
    "filters.zsmooth",
    "Zsmooth Filter",
    "http://pdal.io/stages/filters.zsmooth.html"
};

struct ZsmoothFilter::Private
{
    double radius;
    double pos;
    std::string dimName;
    Dimension::Id statDim;
};

CREATE_SHARED_STAGE(ZsmoothFilter, ptstatInfo)

std::string ZsmoothFilter::getName() const
{
    return ptstatInfo.name;
}

ZsmoothFilter::ZsmoothFilter() : m_p(new Private)
{}


ZsmoothFilter::~ZsmoothFilter()
{}


void ZsmoothFilter::addArgs(ProgramArgs& args)
{
    args.add("radius", "Radius in X/Y plane in which to find neighboring points", m_p->radius, 1.0);
    args.add("medianpercent", "Location (percent) in neighbor list at which to find "
        "neighbor Z value (min == 0, max == 100, median == 50, etc.)", m_p->pos, 50.0);
    args.add("dim", "Name of dimension in which to store statistic", m_p->dimName).setPositional();
}


void ZsmoothFilter::addDimensions(PointLayoutPtr layout)
{
    m_p->statDim = layout->registerOrAssignDim(m_p->dimName, Dimension::Type::Double);
    if (m_p->statDim == Dimension::Id::Z)
        throwError("Can't use 'Z' as output dimension.");
}


void ZsmoothFilter::prepared(PointTableRef)
{
    if (m_p->pos < 0.0 || m_p->pos > 100.0)
        throwError("'medicanpercent' value must be in the range [0, 100]");
    m_p->pos /= 100.0;
}


void ZsmoothFilter::filter(PointView& view)
{
    const KD2Index& kdi = view.build2dIndex();

    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        double d = view.getFieldAs<double>(Dimension::Id::Z, idx);

        std::vector<double> valList;
        PointIdList nears = kdi.radius(idx, m_p->radius);
        for (PointId n = 1; n < nears.size(); ++n)
        {
            double z = view.getFieldAs<double>(Dimension::Id::Z, nears[n]);
            valList.push_back(z);
        }
        std::sort(valList.begin(), valList.end());

        double val;
        if (valList.empty())
            val = view.getFieldAs<double>(Dimension::Id::Z, idx);
        else if (valList.size() == 1)
            val = valList[0];
        else if (m_p->pos == 0.0)
            val = valList[0];
        else if (m_p->pos == 1.0)
            val = valList[valList.size() - 1];
        else
        {
            double pos = m_p->pos * (valList.size() - 1);
            size_t low = (size_t)std::floor(pos);
            size_t high = low + 1;
            double highfrac = pos - low;
            double lowfrac = 1 - highfrac;
            val = valList[low] * lowfrac + valList[high] * highfrac;

        }
        view.setField(m_p->statDim, idx, val);
    }
}

} // namespace pdal
