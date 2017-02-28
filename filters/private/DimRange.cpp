/******************************************************************************
 * Copyright (c) 2015, Brad Chambers (brad.chambers@gmail.com)
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

#include "DimRange.hpp"

#include <pdal/util/Utils.hpp>

namespace pdal
{

std::string::size_type DimRange::subParse(const std::string& r)
{
    bool& ilb(m_inclusive_lower_bound);
    bool& iub(m_inclusive_upper_bound);
    bool& negate(m_negate);
    double& ub(m_upper_bound);
    double& lb(m_lower_bound);
    std::string& name(m_name);

    std::string::size_type pos, count;
    const char *start;
    char *end;

    ilb = true;
    iub = true;
    negate = false;
    pos = 0;
    // Skip leading whitespace.
    count = Utils::extract(r, pos, (int(*)(int))std::isspace);
    pos += count;

    count = Utils::extract(r, pos, (int(*)(int))std::isalpha);
    if (count == 0)
        throw error("No dimension name.");
    name = r.substr(pos, count);
    pos += count;

    if (r[pos] == '!')
    {
        negate = true;
        pos++;
    }

    if (r[pos] == '(')
        ilb = false;
    else if (r[pos] != '[')
        throw error("Missing '(' or '['.");
    pos++;

    // Extract lower bound.
    start = r.data() + pos;
    lb = std::strtod(start, &end);
    if (start == end)
        lb = std::numeric_limits<double>::lowest();
    pos += (end - start);

    count = Utils::extract(r, pos, (int(*)(int))std::isspace);
    pos += count;

    if (r[pos] != ':')
        throw error("Missing ':' limit separator.");
    pos++;

    start = r.data() + pos;
    ub = std::strtod(start, &end);
    if (start == end)
        ub = std::numeric_limits<double>::max();
    pos += (end - start);

    count = Utils::extract(r, pos, (int(*)(int))std::isspace);
    pos += count;

    if (r[pos] == ')')
        iub = false;
    else if (r[pos] != ']')
        throw error("Missing ')' or ']'.");
    pos++;

    count = Utils::extract(r, pos, (int(*)(int))std::isspace);
    pos += count;
    return pos;
}


bool DimRange::valuePasses(double v) const
{
    // Determine if a point passes a range.
    bool fail = ((m_inclusive_lower_bound && v < m_lower_bound) ||
        (!m_inclusive_lower_bound && v <= m_lower_bound) ||
        (m_inclusive_upper_bound && v > m_upper_bound) ||
        (!m_inclusive_upper_bound && v >= m_upper_bound));
    if (m_negate)
        fail = !fail;
    return !fail;
}


void DimRange::parse(const std::string& r)
{
    std::string::size_type pos = subParse(r);
    if (pos != r.size())
        throw error("Invalid characters following valid range.");
}


bool operator < (const DimRange& r1, const DimRange& r2)
{
    return (r1.m_name < r2.m_name ? true :
        r1.m_name > r2.m_name ? false :
        &r1 < &r2);
}


std::istream& operator>>(std::istream& in, DimRange& r)
{
    std::string s;

    std::getline(in, s);
    r.parse(s);
    return in;
}


std::ostream& operator<<(std::ostream& out, const DimRange& r)
{
    out << (r.m_inclusive_lower_bound ? '[' : '(');
    if (r.m_lower_bound != std::numeric_limits<double>::lowest())
        out << r.m_lower_bound;
    out << ':';
    if (r.m_upper_bound != std::numeric_limits<double>::max())
        out << r.m_upper_bound;
    out << (r.m_inclusive_upper_bound ? ']' : ')');
    return out;
}

} // namespace pdal

