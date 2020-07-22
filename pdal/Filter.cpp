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
#include "../filters/private/expr/Expression.hpp"

namespace pdal
{

struct Filter::Args
{
    expr::Expression m_where;
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

void Filter::splitView(const PointViewPtr& view, PointViewPtr& keep, PointViewPtr& skip)
{
    if (m_args->m_whereArg->set())
    {
        PointView *k = keep.get();
        PointView *s = skip.get();
        for (PointRef p : *view)
        {
            PointView *active = m_args->m_where.eval(p) ? k : s;
            active->appendPoint(*view, p.pointId());
        }
    }
    else
        keep = view;
}

Filter::WhereMergeMode Filter::mergeMode() const
{
    return m_args->m_whereMerge;
}

bool Filter::eval(PointRef& p) const
{
    if (!m_args->m_whereArg->set())
        return true;
    return m_args->m_where.eval(p);
}

std::istream& operator>>(std::istream& in, Filter::WhereMergeMode& mode)
{
    std::string s;
    in >> s;

    s = Utils::tolower(s);
    if (s == "auto")
        mode = Filter::WhereMergeMode::Auto;
    else if (s == "true")
        mode = Filter::WhereMergeMode::True;
    else if (s == "false")
        mode = Filter::WhereMergeMode::False;
    else
        in.setstate(std::ios_base::failbit);
    return in;
}

std::ostream& operator<<(std::ostream& out, const Filter::WhereMergeMode& mode)
{
    switch (mode)
    {
    case Filter::WhereMergeMode::Auto:
        out << "auto";
        break;
    case Filter::WhereMergeMode::True:
        out << "true";
        break;
    case Filter::WhereMergeMode::False:
        out << "false";
        break;
    }

    return out;
}

} // namespace pdal

