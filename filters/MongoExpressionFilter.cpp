/******************************************************************************
 * Copyright (c) 2018, Connor Manning (connor@hobu.co)
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

#include <nlohmann/json.hpp>

#include "MongoExpressionFilter.hpp"

#include "private/mongoexpression/Expression.hpp"

namespace pdal
{

static const StaticPluginInfo s_info
{
    "filters.mongo",
    "Pass only points that pass a logic filter.",
    "http://pdal.io/stages/filters.mongo.html"
};

CREATE_STATIC_STAGE(MongoExpressionFilter, s_info);

std::string MongoExpressionFilter::getName() const
{
    return s_info.name;
}

MongoExpressionFilter::MongoExpressionFilter()
{}

MongoExpressionFilter::~MongoExpressionFilter()
{}

void MongoExpressionFilter::addArgs(ProgramArgs& args)
{
    args.add("expression", "Logical query expression", m_json).setPositional();
}

void MongoExpressionFilter::prepared(PointTableRef table)
{
    log()->get(LogLevel::Debug) << "Building expression from: " << m_json <<
        std::endl;

    m_expression = makeUnique<Expression>(*table.layout(), m_json);

    log()->get(LogLevel::Debug) << "Built expression: " << *m_expression <<
        std::endl;
}

PointViewSet MongoExpressionFilter::run(PointViewPtr inView)
{
    PointViewSet views;
    PointViewPtr view(inView->makeNew());

    for (PointId i(0); i < inView->size(); ++i)
    {
        PointRef pr(inView->point(i));
        if (processOne(pr))
        {
            view->appendPoint(*inView, i);
        }
    }

    views.insert(view);
    return views;
}

bool MongoExpressionFilter::processOne(PointRef& pr)
{
    return m_expression->check(pr);
}

} // namespace pdal

