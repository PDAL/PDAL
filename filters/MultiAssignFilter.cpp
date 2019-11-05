/******************************************************************************
 * Copyright (c) 2019, Helix Re Inc. <pravin@helix.re>
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
 *     * Neither the name of Helix Re, Inc. nor the
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

#include "MultiAssignFilter.hpp"

#include <pdal/StageFactory.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include "boost/algorithm/string/split.hpp"
#include "boost/algorithm/string/classification.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.multiassign",
    "Assign values for a dimension range to a specified value.",
    "http://pdal.io/stages/filters.multiassign.html"};


CREATE_STATIC_STAGE(MultiAssignFilter, s_info)

void MultiAssignFilter::addArgs(ProgramArgs& args)
{
    args.add("args", "Assignment JSON", m_json).setPositional();
}

void MultiAssignFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());
    auto assigns = m_json.at("assignments");
    for (auto a : assigns)
    {
        std::vector<std::string> res;
        std::string assign = a.at("assign");
        pdalboost::split(res, assign, pdalboost::is_any_of(","));
        AssignArgs assignArgs;
        for (auto s : res)
        {
            AssignRange assn;
            assn.parse(s);
            assn.m_id = layout->findDim(assn.m_name);
            if (assn.m_id == Dimension::Id::Unknown)
                throwError("Invalid dimension name in 'assignment' option: '" +
                           assn.m_name + "'.");
            assignArgs.m_assignments.push_back(assn);
        }
        if (a.contains("condition"))
        {
            std::string condition = a.at("condition");
            DimRange range;
            range.parse(condition);

            range.m_id = layout->findDim(range.m_name);
            if (range.m_id == Dimension::Id::Unknown)
                throwError("Invalid dimension name in 'condition' option: '" +
                           range.m_name + "'.");
            assignArgs.m_condition = range;
        }
        m_assignments.push_back(assignArgs);
    }

    return;
}

bool MultiAssignFilter::processOne(PointRef& point)
{
    for (auto& a : m_assignments)
    {
        if (a.m_condition.m_id != Dimension::Id::Unknown)
        {
            double condVal = point.getFieldAs<double>(a.m_condition.m_id);
            if (!a.m_condition.valuePasses(condVal))
                continue;
        }
        for (auto& r : a.m_assignments)
            if (r.valuePasses(point.getFieldAs<double>(r.m_id)))
                point.setField(r.m_id, r.m_value);
    }
    return true;
}

void MultiAssignFilter::filter(PointView& view)
{
    PointRef point(view, 0);
    for (PointId id = 0; id < view.size(); ++id)
    {
        point.setPointId(id);
        processOne(point);
    }
}

} // namespace pdal
