/******************************************************************************
* Copyright (c) 2020, Hobu Inc.
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

#include "Filter.hpp"
#include "../filters/private/expr/ConditionalExpression.hpp"

namespace pdal
{

struct Filter::Args
{
    expr::ConditionalExpression m_where;
    Arg *m_whereArg;
    Filter::WhereMergeMode m_whereMerge;
    Arg *m_whereMergeArg;
};

Filter::Filter() : m_args(new Args)
{}

Filter::~Filter()
{}

PointViewSet Filter::run(PointViewPtr view)
{
    PointViewSet viewSet;
    filter(*view);
    viewSet.insert(view);
    return viewSet;
}

void Filter::l_initialize(PointTableRef table)
{
    Stage::l_initialize(table);
}

void Filter::l_addArgs(ProgramArgs& args)
{
    Stage::l_addArgs(args);
    m_args->m_whereArg = &args.add("where",
        "Expression describing points to be passed to this "
        "filter", m_args->m_where);
    m_args->m_whereMergeArg = &args.add("where_merge", "If 'where' option is set, describes "
        "how skipped points should be merged with kept points in standard mode.",
        m_args->m_whereMerge, WhereMergeMode::Auto);
}

void Filter::l_prepared(PointTableRef table)
{
    Stage::l_prepared(table);
    auto status = m_args->m_where.prepare(table.layout());
    if (!status)
        throwError("Invalid 'where': " + status.what());
    if (m_args->m_whereMergeArg->set() && !m_args->m_whereArg->set())
        throwError("Can't set 'where_merge' options without also setting 'where' option.");
}


const expr::ConditionalExpression *Filter::whereExpr() const
{
    if (!m_args->m_where.valid())
        return nullptr;
    return &(m_args->m_where);
}


Stage::WhereMergeMode Filter::mergeMode() const
{
    return m_args->m_whereMerge;
}


} // namespace pdal

