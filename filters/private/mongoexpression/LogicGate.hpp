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

enum class LogicalOperator
{
    lAnd,
    lNot,
    lOr,
    lNor
};

inline bool isLogicalOperator(std::string s)
{
    return s == "$and" || s == "$not" || s == "$or" || s == "$nor";
}

inline LogicalOperator toLogicalOperator(std::string s)
{
    if (s == "$and")    return LogicalOperator::lAnd;
    if (s == "$not")    return LogicalOperator::lNot;
    if (s == "$or")     return LogicalOperator::lOr;
    if (s == "$nor")    return LogicalOperator::lNor;
    throw pdal_error("Invalid logical operator: " + s);
}

inline std::string opToString(LogicalOperator o)
{
    switch (o)
    {
    case LogicalOperator::lAnd: return "$and";
    case LogicalOperator::lNot: return "$not";
    case LogicalOperator::lOr:  return "$or";
    case LogicalOperator::lNor: return "$nor";
    default: throw pdal_error("Invalid logical operator");
    }
}

class LogicGate : public Filterable
{
public:
    LogicGate() = default;
    LogicGate(LogicGate&&) = default;
    LogicGate& operator=(LogicGate&&) = default;
    virtual ~LogicGate() = default;

    static std::unique_ptr<LogicGate> create(std::string s)
    {
        return create(toLogicalOperator(s));
    }

    static std::unique_ptr<LogicGate> create(LogicalOperator type);

    virtual void push(std::unique_ptr<Filterable> f)
    {
        m_filters.push_back(std::move(f));
    }

    virtual std::string toString(std::string pre) const override
    {
        std::ostringstream ss;
        if (m_filters.size()) ss << pre << opToString(type()) << std::endl;
        for (const auto& c : m_filters) ss << c->toString(pre + "  ");
        return ss.str();
    }

    virtual LogicalOperator type() const = 0;

protected:
    std::vector<std::unique_ptr<Filterable>> m_filters;
};

class LogicalAnd : public LogicGate
{
public:
    virtual bool operator()(const pdal::PointRef& pr) const override
    {
        for (const auto& f : m_filters)
        {
            if (!(*f)(pr)) return false;
        }

        return true;
    }

protected:
    virtual LogicalOperator type() const override
    {
        return LogicalOperator::lAnd;
    }
};

class LogicalNot : public LogicGate
{
public:
    using LogicGate::push;
    virtual void push(std::unique_ptr<Filterable> f) override
    {
        if (!m_filters.empty())
            throw pdal_error("Cannot push onto a logical NOT");

        LogicGate::push(std::move(f));
    }

    virtual bool operator()(const pdal::PointRef& pr) const override
    {
        return !(*m_filters.at(0))(pr);
    }

private:
    virtual LogicalOperator type() const override
    {
        return LogicalOperator::lNot;
    }
};

class LogicalOr : public LogicGate
{
public:
    virtual bool operator()(const pdal::PointRef& pr) const override
    {
        for (const auto& f : m_filters)
        {
            if ((*f)(pr)) return true;
        }

        return false;
    }

protected:
    virtual LogicalOperator type() const override
    {
        return LogicalOperator::lOr;
    }
};

class LogicalNor : public LogicalOr
{
public:
    using LogicalOr::operator();
    virtual bool operator()(const pdal::PointRef& pr) const override
    {
        return !LogicalOr::operator()(pr);
    }

protected:
    virtual LogicalOperator type() const override
    {
        return LogicalOperator::lNor;
    }
};

} // namespace pdal

