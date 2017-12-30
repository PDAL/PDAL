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

#include "KNNFilter.hpp"

#include <pdal/pdal_macros.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "private/DimRange.hpp"

#include <iostream>
#include <utility>
namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.knn",
    "Re-assign some point attributes based KNN voting",
    "http://pdal.io/stages/filters.knn.html" );

CREATE_STATIC_PLUGIN(1, 0, KNNFilter, Filter, s_info)

KNNFilter::KNNFilter()
{}


KNNFilter::~KNNFilter()
{}


void KNNFilter::addArgs(ProgramArgs& args)
{
    args.add("domain", "Selects which points will be subject to KNN-based update",
        m_domain);
    args.add("k", "Number of nearest neighbors to consult",
        m_k).setPositional();
    args.add("dimension", "Dimension on to be updated", m_dimName).setPositional();
}

void KNNFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());

    for (auto& r : m_domain)
    {
        r.m_id = layout->findDim(r.m_name);
        if (r.m_id == Dimension::Id::Unknown)
            throwError("Invalid dimension name in 'assignment' option: '" +
                r.m_name + "'.");
    }

    m_dim = layout->findDim(m_dimName);
    if (m_dim == Dimension::Id::Unknown)
        throwError("Dimension '" + m_dimName + "' not found.");
}

std::ostream& operator<<(std::ostream& out, const Dimension::Id& r)
{
    out << (const Dimension::Id&)r;
    out << "=" << (int)r;
    return out;
}

bool KNNFilter::processOne(PointRef& point, PointRef &temp, KD3Index &kdi)
{
    for (DimRange& r : m_domain)
    {
        if (r.valuePasses(point.getFieldAs<double>(r.m_id)))
        {
            std::vector<PointId> iSrc = kdi.neighbors(point, m_k);
            double thresh = iSrc.size()/2.0;
            std::map<double, unsigned int> counts;
            for (PointId id : iSrc)
            {
                temp.setPointId(id);
                double votefor = temp.getFieldAs<double>(m_dim);
                counts[votefor]++;
            }
            auto pr = *std::max_element(counts.begin(), counts.end());
            auto oldclass = point.getFieldAs<double>(m_dim);
            auto newclass = pr.first;
            if (pr.second > thresh && oldclass != newclass)
            {    
                point.setField(m_dim, newclass); 
            }
            break;
        }
    }
    return true;
}

PointViewPtr KNNFilter::loadSet(const std::string& filename,
    PointTable& table)
{
    PipelineManager mgr;

    Stage& reader = mgr.makeReader(filename, "readers.las");
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    assert(viewSet.size() == 1);
    return *viewSet.begin();
}

void KNNFilter::filter(PointView& view)
{
    KD3Index kdi(view);
    kdi.build();
    PointRef point(view, 0);
    PointRef temp(view, 0);
    for (PointId id = 0; id < view.size(); ++id)
    {
        point.setPointId(id);
        processOne(point, temp, kdi);
    }
}

} // namespace pdal

