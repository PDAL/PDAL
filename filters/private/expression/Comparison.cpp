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

#include "Comparison.hpp"

namespace pdal
{

namespace
{

template<typename O>
std::unique_ptr<ComparisonSingle<O>> createSingle(
        ComparisonType type,
        O op,
        double d)
{
    return makeUnique<ComparisonSingle<O>>(type, op, d);
}

} // unnamed namespace

std::unique_ptr<ComparisonOperator> ComparisonOperator::create(
        const Json::Value& json)
{
    if (!json.isObject())
    {
        // If it's a value specified without the $eq operator, convert it.
        Json::Value converted;
        converted["$eq"] = json;
        return create(converted);
    }

    if (json.size() != 1)
    {
        throw pdal_error("Invalid comparison object: " + json.toStyledString());
    }

    const auto key(json.getMemberNames().at(0));
    const ComparisonType co(toComparisonType(key));
    const auto& val(json[key]);

    if (isSingle(co))
    {
        if (!val.isConvertibleTo(Json::ValueType::realValue))
        {
            throw pdal_error("Invalid comparison operand: " +
                    val.toStyledString());
        }

        const double d(val.asDouble());

        switch (co)
        {
        case ComparisonType::eq:
            return createSingle(co, std::equal_to<double>(), d);
        case ComparisonType::gt:
            return createSingle(co, std::greater<double>(), d);
        case ComparisonType::gte:
            return createSingle(co, std::greater_equal<double>(), d);
        case ComparisonType::lt:
            return createSingle(co, std::less<double>(), d);
        case ComparisonType::lte:
            return createSingle(co, std::less_equal<double>(), d);
        case ComparisonType::ne:
            return createSingle(co, std::not_equal_to<double>(), d);
        default:
            throw pdal_error("Invalid single comparison operator");
        }
    }
    else
    {
        if (!val.isArray())
        {
            throw pdal_error("Invalid comparison list: " +
                    val.toStyledString());
        }

        std::vector<double> vals;

        for (const Json::Value& single : val)
        {
            if (!single.isConvertibleTo(Json::ValueType::realValue))
            {
                throw pdal_error("Invalid multi comparison operand: " +
                        val.toStyledString());
            }

            vals.push_back(single.asDouble());
        }

        switch (co)
        {
        case ComparisonType::in:
            return makeUnique<ComparisonAny>(vals);
        case ComparisonType::nin:
            return makeUnique<ComparisonNone>(vals);
        default:
            throw pdal_error("Invalid multi comparison operator");
        }
    }
}

} // namespace pdal

