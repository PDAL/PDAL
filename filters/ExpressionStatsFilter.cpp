/******************************************************************************
 * Copyright (c) 2024, Howard Butler (info@hobu.co)
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

#include "ExpressionStatsFilter.hpp"

#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/Utils.hpp>
#include "./private/expr/ConditionalExpression.hpp"



#include <cctype>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.expressionstats",
    "Accumulate count statistics for a given dimension for an array of expressions",
    "http://pdal.org/stages/filters.expressionstats.html"
};

CREATE_STATIC_STAGE(ExpressionStatsFilter, s_info)

std::string ExpressionStatsFilter::getName() const
{
    return s_info.name;
}

struct ExpressionStatsFilter::Args
{
    std::vector<expr::ConditionalExpression> m_expressions;
    std::string m_dimName;
    Arg *m_whereArg;
};


ExpressionStatsFilter::ExpressionStatsFilter() : m_args(new Args)
{}


ExpressionStatsFilter::~ExpressionStatsFilter()
{}


void ExpressionStatsFilter::addArgs(ProgramArgs& args)
{
    m_args->m_whereArg = &args.add("expressions",
        "Conditional expressions describing points to be passed to this filter",
        m_args->m_expressions).setPositional();
    args.add("dimension", "Dimension on which apply expression to calculate statistics",
        m_dimName).setPositional();

}


void ExpressionStatsFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());

    m_dimId = layout->findDim(m_dimName);
    if (m_dimId == Dimension::Id::Unknown)
        throwError("Invalid dimension name in 'dimension' option: '" + m_dimName + "'.");

    for (auto& expression: m_args->m_expressions)
    {
        if (!expression.valid())
        {
            std::stringstream oss;
            oss << "The expression '" <<  expression
                << "' is invalid";
            throwError(oss.str());
        }

        auto status = expression.prepare(table.layout());
        if (!status)
            throwError("Invalid expression: " + status.what());
    }
}


bool ExpressionStatsFilter::processOne(PointRef& point)
{
    double value = point.getFieldAs<double>(m_dimId);

    for(const auto& expr: m_args->m_expressions)
    {
        bool status = expr.eval(point);
        auto& stat = m_stats[expr.print()];
        if (status)
            stat[value]++;
    }
    return true;
}


void ExpressionStatsFilter::filter(PointView& view)
{
    PointRef point(view, 0);
    for (PointId id = 0; id < view.size(); ++id)
    {
        point.setPointId(id);
        processOne(point);
    }
}

void ExpressionStatsFilter::done(PointTableRef table)
{
    extractMetadata(table);
}

void ExpressionStatsFilter::extractMetadata(PointTableRef table)
{
    uint32_t position(0);


    MetadataNode c = m_metadata.add("dimension", table.layout()->dimName(m_dimId));
    for (auto& stat: m_stats)
    {
        auto& expression_str = stat.first;
        auto& bin_map = stat.second;

        MetadataNode t = m_metadata.addList("statistic");
        t.add("expression", expression_str);
        t.add("position", position);

        for (auto& bin: bin_map)
        {

            auto& value = bin.first;
            auto& count = bin.second;

            MetadataNode n = t.addList("bins");
            n.add("count", count);
            n.add("value", value);
        }

        position++;
    }

}



} // namespace pdal
