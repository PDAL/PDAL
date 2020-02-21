/******************************************************************************
 * Copyright (c) 2014, Hobu Inc. (hobu@hobu.co)
 * Copyright (c) 2015, Bradley J Chambers, brad.chambers@gmail.com
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
 *     * Neither the name of the Andrew Bell or libLAS nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#include "SortFilter.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.sort",
    "Sort data based on a given dimension.",
    "http://pdal.io/stages/filters.sort.html"
};

CREATE_STATIC_STAGE(SortFilter, s_info)

std::string SortFilter::getName() const { return s_info.name; }

void SortFilter::addArgs(ProgramArgs& args)
{
    args.add("dimension", "Dimension on which to sort", m_dimName).
        setPositional();
    args.add("order", "Sort order ASC(ending) or DESC(ending)", m_order,
        SortOrder::ASC);
}

void SortFilter::prepared(PointTableRef table)
{
    m_dim = table.layout()->findDim(m_dimName);
    if (m_dim == Dimension::Id::Unknown)
        throwError("Dimension '" + m_dimName + "' not found.");
}

void SortFilter::filter(PointView& view)
{
    auto cmp = [this](const PointRef& p1, const PointRef& p2)
    {
        bool result = p1.compare(m_dim, p2);
        return (m_order == SortOrder::ASC) ? result : !result;
    };

    std::stable_sort(view.begin(), view.end(), cmp);
}

std::istream& operator >> (std::istream& in, SortOrder& order)
{
    std::string s;

    in >> s;
    s = Utils::toupper(s);
    if (s == "ASC")
        order = SortOrder::ASC;
    else if (s == "DESC")
        order = SortOrder::DESC;
    else
        in.setstate(std::ios::failbit);
    return in;
}

std::ostream& operator<<(std::ostream& out, const SortOrder& order)
{
    switch (order)
    {
    case SortOrder::ASC:
        out << "ASC";
    case SortOrder::DESC:
        out << "DESC";
    }
    return out;
}

} // namespace pdal

