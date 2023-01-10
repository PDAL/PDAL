/******************************************************************************
 * Copyright (c) 2023, Howard Butler (info@hobu.co)
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

#include "ExpressionFilter.hpp"

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
    "filters.expression",
    "Pass only points given an expression",
    "http://pdal.io/stages/filters.expression.html"
};

CREATE_STATIC_STAGE(ExpressionFilter, s_info)

std::string ExpressionFilter::getName() const
{
    return s_info.name;
}

struct ExpressionFilter::Args
{
    expr::ConditionalExpression m_expression;
    Arg *m_whereArg;
};


ExpressionFilter::ExpressionFilter() : m_args(new Args)
{}


ExpressionFilter::~ExpressionFilter()
{}


void ExpressionFilter::addArgs(ProgramArgs& args)
{
    m_args->m_whereArg = &args.add("expression",
        "Conditional expression describing points to be passed to this filter",
        m_args->m_expression).setPositional();
}


void ExpressionFilter::prepared(PointTableRef table)
{
     if (!m_args->m_expression.valid())
     {
         std::stringstream oss;
         oss << "The expression '" <<  m_args->m_expression
             << "' is invalid";
         throwError(oss.str());
     }

     auto status = m_args->m_expression.prepare(table.layout());
        if (!status)
            throwError("Invalid 'where': " + status.what());
}


bool ExpressionFilter::processOne(PointRef& point)
{
    bool status = m_args->m_expression.eval(point);
    return status;
}


PointViewSet ExpressionFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;
    PointViewPtr outView = inView->makeNew();

    if (!inView->size())
        return viewSet;

    for (PointRef p : *inView)
    {
        if (processOne(p))
            outView->appendPoint(*inView, p.pointId());
    }

    viewSet.insert(outView);
    return viewSet;
}

} // namespace pdal
