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

#include "StatsFilter.hpp"

#include <unordered_map>

#include <pdal/pdal_export.hpp>
#include <pdal/Options.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/util/Utils.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.stats",
    "Compute statistics about each dimension (mean, min, max, etc.)",
    "http://pdal.io/stages/filters.stats.html" );

CREATE_STATIC_PLUGIN(1, 0, StatsFilter, Filter, s_info)

std::string StatsFilter::getName() const { return s_info.name; }

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
    if (m_enumerate == Enumerate)
        for (auto& v : m_values)
            m.addList("values", v.first);
    else if (m_enumerate == Count)
        for (auto& v : m_values)
        {
            std::string val =
                std::to_string(v.first) + "/" + std::to_string(v.second);
            m.addList("counts", val);
        }
}

} // namespace stats

using namespace stats;

void StatsFilter::filter(PointView& view)
{
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        for (auto p = m_stats.begin(); p != m_stats.end(); ++p)
        {
            Dimension::Id::Enum d = p->first;
            Summary& c = p->second;
            c.insert(view.getFieldAs<double>(d, idx));
        }
    }
}


void StatsFilter::done(PointTableRef table)
{
    extractMetadata();
}


void StatsFilter::processOptions(const Options& options)
{
    m_dimNames = options.getValueOrDefault<StringList>("dimensions");
    m_enums = options.getValueOrDefault<StringList>("enumerate");
    m_counts = options.getValueOrDefault<StringList>("count");
}


void StatsFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());

    std::unordered_map<std::string, Summary::EnumType> dims;

    // Add dimensions to the list.
    if (m_dimNames.empty())
    {
        for (auto id : layout->dims())
            dims[layout->dimName(id)] = Summary::NoEnum;
    }
    else
    {
        for (auto& s : m_dimNames)
        {
            if (layout->findDim(s) == Dimension::Id::Unknown)
            {
                std::ostringstream out;
                out << "Dimension '" << s << "' listed in --dimensions option "
                   "does not exist.  Ignoring.";
                Utils::printError(out.str());
            }
            else
                dims[s] = Summary::NoEnum;
        }
    }

    // Set the enumeration flag for those dimensions specified.
    for (auto& s : m_enums)
    {
        if (dims.find(s) == dims.end())
        {
            std::ostringstream out;
            out << "Dimension '" << s << "' listed in --enumerate option "
                "does not exist.  Ignoring.";
            Utils::printError(out.str());
        }
        else
            dims[s] = Summary::Enumerate;
    }

    // Set the enumeration flag for those dimensions specified.
    for (auto& s : m_counts)
    {
        if (dims.find(s) == dims.end())
        {
            std::ostringstream out;
            out << "Dimension '" << s << "' listed in --count option "
                "does not exist.  Ignoring.";
            Utils::printError(out.str());
        }
        else
            dims[s] = Summary::Count;
    }

    // Create the summary objects.
    for (auto& dv : dims)
        m_stats.insert(std::make_pair(layout->findDim(dv.first),
            Summary(dv.first, dv.second)));
}
    

void StatsFilter::extractMetadata()
{
    uint32_t position(0);

    for (auto di = m_stats.begin(); di != m_stats.end(); ++di)
    {
        const Summary& s = di->second;

        MetadataNode t = m_metadata.addList("statistic");
        t.add("position", position++);
        s.extractMetadata(t);
    }
}


const Summary& StatsFilter::getStats(Dimension::Id::Enum dim) const
{
    for (auto di = m_stats.begin(); di != m_stats.end(); ++di)
    {
        Dimension::Id::Enum d = di->first;
        if (d == dim)
            return di->second;
    }
    throw pdal_error("Dimension not found");
}

} // namespace pdal
