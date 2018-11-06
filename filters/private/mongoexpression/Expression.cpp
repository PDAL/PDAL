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

#include "Expression.hpp"

namespace pdal
{

void Expression::build(LogicGate& gate, const Json::Value& json)
{
    if (json.isArray())
    {
        for (const Json::Value& val : json) build(gate, val);
        return;
    }

    if (!json.isObject())
    {
        throw pdal_error("Unexpected expression: " + json.toStyledString());
    }

    LogicGate* active(&gate);

    std::unique_ptr<LogicGate> outer;

    if (json.size() > 1)
    {
        outer = LogicGate::create(LogicalOperator::lAnd);
        active = outer.get();
    }

    for (const std::string key : json.getMemberNames())
    {
        const Json::Value& val(json[key]);

        if (isLogicalOperator(key))
        {
            auto inner(LogicGate::create(key));
            if (inner->type() != LogicalOperator::lNot && !val.isArray())
            {
                throw pdal_error("Logical operator expressions must be arrays");
            }

            build(*inner, val);
            active->push(std::move(inner));
        }
        else if (!val.isObject() || val.size() == 1)
        {
            // A comparison object.
            active->push(Comparison::create(m_layout, key, val));
        }
        else
        {
            // key is the name of a dimension, val is an object of
            // multiple comparison key/val pairs, for example:
            //
            // key: "Red"
            // val: { "$gt": 100, "$lt": 200 }
            //
            // There cannot be any further nested logical operators
            // within val, since we've already selected a dimension.
            for (const std::string& innerKey : val.getMemberNames())
            {
                Json::Value nest;
                nest[innerKey] = val[innerKey];
                active->push(Comparison::create(m_layout, key, nest));
            }
        }
    }

    if (outer) gate.push(std::move(outer));
}

} // namespace pdal

