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

#include "RangeFilter.hpp"

#include <cmath>
#include <limits>
#include <map>
#include <regex>
#include <string>
#include <utility>
#include <vector>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.range", "Pass only points given a dimension/range.",
               "http://pdal.io/stages/filters.range.html");

CREATE_STATIC_PLUGIN(1, 0, RangeFilter, Filter, s_info)

std::string RangeFilter::getName() const
{
    return s_info.name;
}

void RangeFilter::processOptions(const Options& options)
{
    StringList range_string = options.getValueOrDefault<StringList>("limits");
    auto parseRanges = [range_string]()
    {
        std::vector<Range> range_list;

        for (auto const& r : range_string)
        {
            std::regex rgx("(\\w+)([\\(\\[])([-]{0,1}[\\d]*[\\.]{0,1}[\\d]*):([-]{0,1}[\\d]*[\\.]{0,1}[\\d]*)([\\)\\]])");
            int submatches[] = { 1, 2, 3, 4, 5 };
            std::sregex_token_iterator it(r.begin(), r.end(), rgx, submatches);

            std::string name;
            bool ilb = true;
            bool iub = true;
            double lb = -std::numeric_limits<double>::max();
            double ub = std::numeric_limits<double>::max();

            name = *it++;
            if (*it++ == "(")
                ilb = false;
            std::string lbtmp = *it++;
            if (lbtmp != "")
                lb = std::atof(lbtmp.c_str());
            std::string uptmp = *it++;
            if (uptmp != "")
                ub = std::atof(uptmp.c_str());
            if (*it++ == ")")
                iub = false;

            Range range(name, lb, ub, ilb, iub);
            range_list.push_back(range);
        }

        return range_list;
    };
    m_range_list = parseRanges();

    if (m_range_list.size() == 0)
        throw pdal_error("No ranges given");
}

void RangeFilter::ready(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    for (auto const& d : m_range_list)
    {
        m_dimensions_map.insert(
            std::make_pair(
                layout->findDim(d.m_name),
                d));
    }
}

PointViewSet RangeFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;
    if (!inView->size())
        return viewSet;

    PointViewPtr outView = inView->makeNew();

    for (PointId i = 0; i < inView->size(); ++i)
    {
        bool keep_point = true;
        for (auto const& d : m_dimensions_map)
        {
            double v = inView->getFieldAs<double>(d.first, i);
            if (d.second.m_inclusive_lower_bound)
            {
                if (v < d.second.m_lower_bound)
                    keep_point = false;
            }
            else
            {
                if (v <= d.second.m_lower_bound)
                    keep_point = false;
            }
            if (d.second.m_inclusive_upper_bound)
            {
                if (v > d.second.m_upper_bound)
                    keep_point = false;
            }
            else
            {
                if (v >= d.second.m_upper_bound)
                    keep_point = false;
            }

            if (keep_point)
                break;
        }
        if (keep_point)
            outView->appendPoint(*inView, i);
    }

    viewSet.insert(outView);

    return viewSet;
}

} // pdal
