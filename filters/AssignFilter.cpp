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

#include "AssignFilter.hpp"

#include <pdal/pdal_macros.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "private/DimRange.hpp"

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.assign",
    "Assign values for a dimension using a specified value.",
    "http://pdal.io/stages/filters.assign.html" );

CREATE_STATIC_PLUGIN(1, 0, AssignFilter, Filter, s_info)

struct AssignRange : public DimRange
{
    void parse(const std::string& r);
    double m_value;
};

void AssignRange::parse(const std::string& r)
{
    std::string::size_type pos, count;
    const char *start;
    char *end;

    pos = subParse(r);
    count = Utils::extract(r, pos, (int(*)(int))std::isspace);
    pos += count;

    if (r[pos] != '=')
        throw error("Missing '=' assignment separator.");
    pos++;

    count = Utils::extract(r, pos, (int(*)(int))std::isspace);
    pos += count;

    // Extract value
    start = r.data() + pos;
    m_value = std::strtod(start, &end);
    if (start == end)
        throw error("Missing value to assign following '='.");
    pos += (end - start);

    if (pos != r.size())
        throw error("Invalid characters following valid range.");
}


std::istream& operator>>(std::istream& in, AssignRange& r)
{
    std::string s;

    std::getline(in, s);
    r.parse(s);
    return in;
}


std::ostream& operator<<(std::ostream& out, const AssignRange& r)
{
    out << (const DimRange&)r;
    out << "=" << r.m_name;
    return out;
}


AssignFilter::AssignFilter()
{}


AssignFilter::~AssignFilter()
{}


void AssignFilter::addArgs(ProgramArgs& args)
{
    args.add("assignment", "Values to assign to dimensions based on range.",
        m_assignments);
}


void AssignFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());

    for (auto& r : m_assignments)
    {
        r.m_id = layout->findDim(r.m_name);
        if (r.m_id == Dimension::Id::Unknown)
            throwError("Invalid dimension name in 'values' option: '" +
                r.m_name + "'.");
    }
}


bool AssignFilter::processOne(PointRef& point)
{
    for (AssignRange& r : m_assignments)
        if (r.valuePasses(point.getFieldAs<double>(r.m_id)))
            point.setField(r.m_id, r.m_value);
    return true;
}


void AssignFilter::filter(PointView& view)
{
    PointRef point(view, 0);
    for (PointId id = 0; id < view.size(); ++id)
    {
        point.setPointId(id);
        processOne(point);
    }
}

} // namespace pdal

