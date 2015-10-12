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

#include <pdal/util/Utils.hpp>

#include <cctype>
#include <limits>
#include <map>
#include <string>
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

namespace
{

RangeFilter::Range parseRange(const std::string& r)
{ 
    std::string::size_type pos, count;
    bool ilb = true;
    bool iub = true;
    const char *start;
    char *end;
    std::string name;
    double ub, lb;

    try
    {
        pos = 0;
        // Skip leading whitespace.
        count = Utils::extract(r, pos, (int(*)(int))std::isspace);
        pos += count;

        count = Utils::extract(r, pos, (int(*)(int))std::isalpha);
        if (count == 0)
           throw std::string("No dimension name.");
        name = r.substr(pos, count);
        pos += count;

        if (r[pos] == '(')
            ilb = false;
        else if (r[pos] != '[')
            throw std::string("Missing '(' or '['.");
        pos++;

        // Extract lower bound.
        start = r.data() + pos;
        lb = std::strtod(start, &end);
        if (start == end)
            lb = std::numeric_limits<double>::min();
        pos += (end - start);

        count = Utils::extract(r, pos, (int(*)(int))std::isspace);
        pos += count;

        if (r[pos] != ':')
            throw std::string("Missing ':' limit separator.");
        pos++;

        start = r.data() + pos;
        ub = std::strtod(start, &end);
        if (start == end)
            ub = std::numeric_limits<double>::max();
        pos += (end - start);

        count = Utils::extract(r, pos, (int(*)(int))std::isspace);
        pos += count;

        if (r[pos] == ')')
            iub = false;
        else if (r[pos] != ']')
            throw std::string("Missing ')' or ']'.");
        pos++;

        count = Utils::extract(r, pos, (int(*)(int))std::isspace);
        pos += count;

        if (pos != r.size())
            throw std::string("Invalid characters following valid range.");
    }
    catch (std::string s)
    {
        std::ostringstream oss;
        oss << "filters.range: invalid 'limits' option: '" << r << "': " << s;
        throw pdal_error(oss.str());
    }
    return RangeFilter::Range(name, lb, ub, ilb, iub);
}

}


void RangeFilter::processOptions(const Options& options)
{
    StringList rangeString = options.getValueOrDefault<StringList>("limits");

    if (rangeString.empty())
        throw pdal_error("filters.range missing required 'limits' option.");

    for (auto const& r : rangeString)
        m_range_list.push_back(parseRange(r));
}


void RangeFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());

    for (auto const& d : m_range_list)
    {
        Dimension::Id::Enum id = layout->findDim(d.m_name);
        if (id == Dimension::Id::Unknown)
        {
            std::ostringstream oss;
            oss << "Invalid dimension name in filters.range 'limits' "
                "option: '" << d.m_name << "'.";
            throw pdal_error(oss.str());
        }
        m_dimensions_map[id] = d;
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
