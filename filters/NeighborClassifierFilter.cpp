/******************************************************************************
* Copyright (c) 2017, Hobu Inc., info@hobu.co
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

#include "NeighborClassifierFilter.hpp"

#include <unordered_map>

#include <pdal/KDIndex.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "private/DimRange.hpp"

namespace pdal
{

static PluginInfo const NeighborClassifierFilter_info
{
    "filters.neighborclassifier",
    "Re-assign some point attributes based KNN voting",
    "https://pdal.org/stages/filters.neighborclassifier.html"
};

struct NeighborClassifierFilter::Private
{
    DimRangeList domain;
    int k;
    Dimension::Id dimId;
    std::string dimName;
    std::string candidateFile;
    std::unordered_map<PointId, int> newClass;
};

CREATE_STATIC_STAGE(NeighborClassifierFilter, NeighborClassifierFilter_info)

NeighborClassifierFilter::NeighborClassifierFilter() : m_p(new Private())
{}


std::string NeighborClassifierFilter::getName() const
{
    return NeighborClassifierFilter_info.name;
}

void NeighborClassifierFilter::addArgs(ProgramArgs& args)
{
    args.add("domain", "Selects which points will be subject to "
        "KNN-based assignment", m_p->domain);
    args.add("k", "Number of nearest neighbors to consult",
        m_p->k).setPositional();
    args.add("candidate", "candidate file name", m_p->candidateFile);
    args.add("dimension", "Dimension on which votes are calculated (treated as an integer).",
        m_p->dimName, "Classification");
}

void NeighborClassifierFilter::initialize()
{
    if (m_p->k < 1)
        throwError("Invalid 'k' option: " + std::to_string(m_p->k) + ", must be > 0");
}


void NeighborClassifierFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());

    m_p->dimId = layout->findDim(m_p->dimName);
    if (m_p->dimId == Dimension::Id::Unknown)
        throwError("Dimension '" + m_p->dimName + "' does not exist.");

    Utils::StatusWithReason status = m_p->domain.prepare(layout);
    if (!status)
        throwError("Invalid dimension name in 'domain': " + status.what());
}


void NeighborClassifierFilter::ready(PointTableRef)
{
    m_p->newClass.clear();
}


void NeighborClassifierFilter::doOneNoDomain(PointRef &point, PointRef &temp,
    KD3Index &kdi)
{
    PointIdList iSrc = kdi.neighbors(point, m_p->k);
    double thresh = iSrc.size()/2.0;

    // vote NNs
    using CountMap = std::map<int, unsigned int>;
    CountMap counts;
    for (PointId id : iSrc)
    {
        temp.setPointId(id);
        counts[temp.getFieldAs<int>(m_p->dimId)]++;
    }

    // pick winner of the vote
    auto pr = *std::max_element(counts.begin(), counts.end(),
        [](CountMap::const_reference p1, CountMap::const_reference p2)
        { return p1.second < p2.second; });

    // update point
    auto oldclass = point.getFieldAs<int>(m_p->dimId);
    auto newclass = pr.first;
    if (pr.second > thresh && oldclass != newclass)
        m_p->newClass[point.pointId()] = newclass;
}

// update point.  kdi and temp both reference the NN point cloud
bool NeighborClassifierFilter::doOne(PointRef& point, PointRef &temp,
    KD3Index &kdi)
{
    if (m_p->domain.empty())  // No domain, process all points
        doOneNoDomain(point, temp, kdi);

    for (const DimRange& r : m_p->domain.ranges())
    {   // process only points that satisfy a domain condition
        if (r.valuePasses(point.getFieldAs<double>(r.m_id)))
        {
            doOneNoDomain(point, temp, kdi);
            break;
        }
    }
    return true;
}


PointViewPtr NeighborClassifierFilter::loadSet(const std::string& filename,
    PointTableRef table)
{
    PipelineManager mgr;

    Stage& reader = mgr.makeReader(filename, "");
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    assert(viewSet.size() == 1);
    return *viewSet.begin();
}

void NeighborClassifierFilter::filter(PointView& view)
{
    PointRef point_src(view, 0);
    if (m_p->candidateFile.empty())
    {   // No candidate file so NN comes from src file
        KD3Index& kdiSrc = view.build3dIndex();
        PointRef point_nn(view, 0);
        for (PointId id = 0; id < view.size(); ++id)
        {
            point_src.setPointId(id);
            doOne(point_src, point_nn, kdiSrc);
        }
    }
    else
    {   // NN comes from candidate file
        ColumnPointTable candTable;
        PointViewPtr candView = loadSet(m_p->candidateFile, candTable);
        KD3Index& kdiCand = candView->build3dIndex();
        PointRef point_nn(*candView, 0);
        for (PointId id = 0; id < view.size(); ++id)
        {
            point_src.setPointId(id);
            doOne(point_src, point_nn, kdiCand);
        }
    }

    for (auto& [pointId, classification] : m_p->newClass)
        view.setField(m_p->dimId, pointId, classification);
}

} // namespace pdal
