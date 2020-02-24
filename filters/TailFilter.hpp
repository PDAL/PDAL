/******************************************************************************
 * Copyright (c) 2017, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/Filter.hpp>

namespace pdal
{

class PDAL_DLL TailFilter : public Filter
{
public:
    TailFilter()
    {}
    TailFilter& operator=(const TailFilter&) = delete;
    TailFilter(const TailFilter&) = delete;

    std::string getName() const;

private:
    point_count_t m_count;
    bool m_invert;

    void addArgs(ProgramArgs& args)
    {
        args.add("count", "Number of points to return from end. "
            "If 'invert' is true, number of points to drop from the end.",
            m_count, point_count_t(10));
        args.add("invert", "If true, 'count' specifies the number of points "
            "at the end to drop.", m_invert);
    }

    PointViewSet run(PointViewPtr view)
    {
        if (m_count > view->size())
            log()->get(LogLevel::Warning)
                << "Requested number of points (count=" << m_count
                << ") exceeds number of available points.\n";
        PointViewSet viewSet;
        PointViewPtr outView = view->makeNew();
        PointId start;
        PointId end;
        if (m_invert)
        {
            start = 0;
            end = view->size() - (std::min)(m_count, view->size());
        }
        else
        {
            start = view->size() - (std::min)(m_count, view->size());
            end = view->size();
        }

        for (PointId i = start; i < end; ++i)
            outView->appendPoint(*view, i);
        viewSet.insert(outView);
        return viewSet;
    }
};

} // namespace pdal
