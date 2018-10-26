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

#pragma once

#include "Support.hpp"

namespace pdal
{

enum class ComparisonType
{
    eq,
    gt,
    gte,
    lt,
    lte,
    ne,
    in,
    nin
};

inline bool isComparisonType(const std::string& s)
{
    return s == "$eq" || s == "$gt" || s == "$gte" || s == "$lt" ||
        s == "$lte" || s == "$ne" || s == "$in" || s == "$nin";
}

inline ComparisonType toComparisonType(const std::string& s)
{
    if (s == "$eq")         return ComparisonType::eq;
    else if (s == "$gt")    return ComparisonType::gt;
    else if (s == "$gte")   return ComparisonType::gte;
    else if (s == "$lt")    return ComparisonType::lt;
    else if (s == "$lte")   return ComparisonType::lte;
    else if (s == "$ne")    return ComparisonType::ne;
    else if (s == "$in")    return ComparisonType::in;
    else if (s == "$nin")   return ComparisonType::nin;
    else throw pdal_error("Invalid comparison type: " + s);
}

inline std::string typeToString(ComparisonType c)
{
    switch (c)
    {
        case ComparisonType::eq: return "$eq";
        case ComparisonType::gt: return "$gt";
        case ComparisonType::gte: return "$gte";
        case ComparisonType::lt: return "$lt";
        case ComparisonType::lte: return "$lte";
        case ComparisonType::ne: return "$ne";
        case ComparisonType::in: return "$in";
        case ComparisonType::nin: return "$nin";
        default: throw pdal_error("Invalid comparison type enum");
    }
}

inline bool isSingle(ComparisonType co)
{
    return co != ComparisonType::in && co != ComparisonType::nin;
}

inline bool isMultiple(ComparisonType co)
{
    return !isSingle(co);
}

class ComparisonOperator : public Comparable
{
public:
    ComparisonOperator(ComparisonType type) : m_type(type) { }

    virtual ~ComparisonOperator() { }

    // Accepts a JSON value of the form:
    // { "$<op>": <val> }       // E.g. { "$eq": 42 }
    //
    // or the special case for the "$eq" operator:
    //      <val>               // Equivalent to the above.
    //
    //
    // Returns a pointer to a functor that performs the requested comparison.
    static std::unique_ptr<ComparisonOperator> create(const Json::Value& json);

    ComparisonType type() const { return m_type; }

protected:
    ComparisonType m_type;
};

template<typename Op>
class ComparisonSingle : public ComparisonOperator
{
public:
    ComparisonSingle(ComparisonType type, Op op, double val)
        : ComparisonOperator(type)
        , m_op(op)
        , m_val(val)
    { }

    virtual bool operator()(double in) const override
    {
        return m_op(in, m_val);
    }

    virtual std::string toString(std::string pre) const override
    {
        std::ostringstream ss;
        ss << pre << typeToString(m_type) << " " << m_val << std::endl;
        return ss.str();
    }

protected:
    Op m_op;
    double m_val;
};

class ComparisonMulti : public ComparisonOperator
{
public:
    ComparisonMulti(ComparisonType type, const std::vector<double>& vals)
        : ComparisonOperator(type)
        , m_vals(vals)
    { }

    virtual std::string toString(std::string pre) const override
    {
        std::ostringstream ss;
        ss << pre << typeToString(m_type) << " ";
        for (const double d : m_vals) ss << d << " ";
        ss << std::endl;
        return ss.str();
    }

protected:
    std::vector<double> m_vals;
};

class ComparisonAny : public ComparisonMulti
{
public:
    ComparisonAny(const std::vector<double>& vals)
        : ComparisonMulti(ComparisonType::in, vals)
    { }

    virtual bool operator()(double in) const override
    {
        return std::any_of(m_vals.begin(), m_vals.end(), [in](double val)
        {
            return in == val;
        });
    }
};

class ComparisonNone : public ComparisonMulti
{
public:
    ComparisonNone(const std::vector<double>& vals)
        : ComparisonMulti(ComparisonType::nin, vals)
    { }

    virtual bool operator()(double in) const override
    {
        return std::none_of(m_vals.begin(), m_vals.end(), [in](double val)
        {
            return in == val;
        });
    }
};

class Comparison : public Filterable
{
public:
    Comparison(const PointLayout& layout, std::string dimName, Json::Value json)
        : m_dimName(dimName)
        , m_dimId(layout.findDim(dimName))
        , m_op(ComparisonOperator::create(json))
    {
        if (m_dimId == pdal::Dimension::Id::Unknown)
        {
            throw pdal_error("Unknown dimension: " + dimName);
        }
    }

    bool operator()(const pdal::PointRef& pointRef) const override
    {
        return (*m_op)(pointRef.getFieldAs<double>(m_dimId));
    }

    virtual std::string toString(std::string pre) const override
    {
        std::ostringstream ss;
        ss << pre << m_dimName << " ";
        ss << m_op->toString("");
        return ss.str();
    }

protected:
    std::string m_dimName;
    pdal::Dimension::Id m_dimId;
    std::unique_ptr<ComparisonOperator> m_op;
};

} // namespace pdal

