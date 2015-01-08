/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/Utils.hpp>

#include "StatsFilter.hpp"

namespace pdal
{
namespace stats
{

void Summary::extractMetadata(MetadataNode &m) const
{
    uint32_t cnt = static_cast<uint32_t>(count());
    m.add("count", cnt, "count");
    m.add("minimum", minimum(), "minimum");
    m.add("maximum", maximum(), "maximum");
    m.add("average", average(), "average");
    m.add("name", m_name, "name");
}

} // namespace stats


void StatsFilter::filter(PointBuffer& buffer)
{
    for (PointId idx = 0; idx < buffer.size(); ++idx)
    {
        for (auto p = m_stats.begin(); p != m_stats.end(); ++p)
        {
            Dimension::Id::Enum d = p->first;
            SummaryPtr c = p->second;
            c->insert(buffer.getFieldAs<double>(d, idx));
        }
    }
}


void StatsFilter::done(PointContext ctx)
{
    extractMetadata(ctx);
}

void StatsFilter::processOptions(const Options& options)
{
    m_dimNames = m_options.getValueOrDefault<std::string>("dimensions", "");
}


void StatsFilter::ready(PointContext ctx)
{
    using namespace std;

    std::vector<Dimension::Id::Enum> dims;
    std::vector<std::string> dimNames;
    if (m_dimNames.empty())
    {
        dims = ctx.dims();
        for (auto di = dims.begin(); di != dims.end(); ++di)
            dimNames.push_back(ctx.dimName(*di));
    }
    else
    {
        auto splits = [](char c)
            { return c == ' ' || c == ','; };
        dimNames = Utils::split2(m_dimNames, splits);
        for (auto di = dimNames.begin(); di != dimNames.end(); ++di)
        {
            auto dim = ctx.findDim(*di);
            if (dim != Dimension::Id::Unknown)
                dims.push_back(dim);
        }
    }

    auto ni = dimNames.begin();
    for (auto di = dims.begin(); di != dims.end(); ++di, ++ni)
        m_stats[*di] = SummaryPtr(new stats::Summary(*ni));
}


void StatsFilter::extractMetadata(PointContext ctx)
{
    uint32_t position(0);
    
    for (auto di = m_stats.begin(); di != m_stats.end(); ++di)
    {
        Dimension::Id::Enum d = di->first;
        const SummaryPtr s = di->second;

        MetadataNode t = m_metadata.addList("statistic");
        t.add("position", position++);
        s->extractMetadata(t);
    }
}


stats::Summary const& StatsFilter::getStats(Dimension::Id::Enum dim) const
{
    for (auto di = m_stats.begin(); di != m_stats.end(); ++di)
    {
        Dimension::Id::Enum d = di->first;
        if (d == dim)
            return *(di->second);
    }
    throw pdal_error("Dimension not found");
}

} // namespace pdal
