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
    args.add("dimensions", "Dimensions and ordering on which to sort", m_dimNames).
        setPositional();

    args.addSynonym("dimensions", "dimension");

    args.add("order", "Sort order ASC(ending) or DESC(ending)", m_order,
        SortOrder::ASC);

    args.add("algorithm", "NORMAL (default) or STABLE", m_algorithm,
        SortAlgorithm::Normal);
}

void SortFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());
    for (auto& s : m_dimNames)
    {
        Dimension::Id dimId = layout->findDim(s);
        if (layout->findDim(s) == Dimension::Id::Unknown)
            throwError("Cannot sort because dimension '" + s + "' was not found.");
        m_dims.push_back(dimId);
    }

    if (!m_dimNames.size())
        throwError("At least one valid dimension name must be provided!");
}

void SortFilter::filter(PointView& view)
{
    for (Dimension::IdList::size_type i=0; i < m_dims.size(); ++i)
    {
        PointView::Compare cmp;
        cmp = [&view, dim=m_dims[i]](PointId id1, PointId id2)
        {
            return view.compare(dim, id1, id2);
        };
        if (m_order == SortOrder::DESC)
            cmp = [&view, dim=m_dims[i]](PointId id1, PointId id2)
            {
                return view.compare(dim, id2, id1);
            };

        if (m_dims.size() > 1)
        {
            // If we have multiple dimensions, sort the first one
            // and then stable_sort the rest
            if (i == 0)
                view.sort(cmp);
            else
                view.stableSort(cmp);
        }
        else
        {
            if (m_algorithm == SortAlgorithm::Stable)
                view.stableSort(cmp);
            else if (m_algorithm == SortAlgorithm::Normal)
                view.sort(cmp);
        }
    }
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

std::istream& operator >> (std::istream& in, SortAlgorithm& order)
{
    std::string s;

    in >> s;
    s = Utils::toupper(s);
    if (s == "NORMAL")
        order = SortAlgorithm::Normal;
    else if (s == "STABLE")
        order = SortAlgorithm::Stable;
    else
        in.setstate(std::ios::failbit);
    return in;
}

std::ostream& operator<<(std::ostream& out, const SortAlgorithm& order)
{
    switch (order)
    {
    case SortAlgorithm::Normal:
        out << "NORMAL";
    case SortAlgorithm::Stable:
        out << "STABLE";
    }
    return out;
}

} // namespace pdal

