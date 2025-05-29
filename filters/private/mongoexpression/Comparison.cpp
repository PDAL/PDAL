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

std::unique_ptr<Comparison> Comparison::create(const PointLayout& layout,
        const std::string dimName, const nlohmann::json& json)
{
    if (!json.is_object())
    {
        // If it's a value specified without the $eq operator, convert it.
        nlohmann::json converted;
        converted["$eq"] = json;
        return create(layout, dimName, converted);
    }

    if (json.size() != 1)
    {
        throw pdal_error("Invalid comparison object: " +
            json.get<std::string>());
    }

    auto it = json.begin();

    //const auto key(json.getMemberNames().at(0));
    const ComparisonType co(toComparisonType(it.key()));
    const nlohmann::json& val(it.value());

    const Dimension::Id dimId(layout.findDim(dimName));
    if (dimId == pdal::Dimension::Id::Unknown)
    {
        throw pdal_error("Unknown dimension: " + dimName);
    }

    if (isSingle(co))
    {
        Operand op(layout, val);
        switch (co)
        {
        case ComparisonType::eq:
            return makeUnique<ComparisonEqual>(dimId, op);
        case ComparisonType::gt:
            return makeUnique<ComparisonGreater>(dimId, op);
        case ComparisonType::gte:
            return makeUnique<ComparisonGreaterEqual>(dimId, op);
        case ComparisonType::lt:
            return makeUnique<ComparisonLess>(dimId, op);
        case ComparisonType::lte:
            return makeUnique<ComparisonLessEqual>(dimId, op);
        case ComparisonType::ne:
            return makeUnique<ComparisonNotEqual>(dimId, op);
        default:
            throw pdal_error("Invalid single comparison operator");
        }
    }
    else
    {
        if (!val.is_array())
            throw pdal_error("Invalid comparisons: " + val.dump());

        Operands ops;
        for (auto& op : val)
        {
            ops.emplace_back(layout, op);
        }
        switch (co)
        {
        case ComparisonType::in:
            return makeUnique<ComparisonAny>(dimId, ops);
        case ComparisonType::nin:
            return makeUnique<ComparisonNone>(dimId, ops);
        default:
            throw pdal_error("Invalid multi comparison operator");
        }
    }
}

} // namespace pdal

