/******************************************************************************
 * Copyright (c) 2015, Hobu Inc. (info@hobu.co)
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
 *     * Neither the name of Hobu, Inc. nor the names of its contributors
 *       may be used to endorse or promote products derived from this
 *       software without specific prior written permission.
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

#include "DividerFilter.hpp"

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.divider",
    "Divide points into approximately equal sized groups based on a simple "
      "scheme",
    "http://pdal.io/stages/filters.divider.html" );

CREATE_STATIC_PLUGIN(1, 0, DividerFilter, Filter, s_info)

std::string DividerFilter::getName() const { return s_info.name; }

void DividerFilter::processOptions(const Options& options)
{
    std::string mode = options.getValueOrDefault<std::string>("mode");
    mode = Utils::tolower(mode);
    if (mode.empty() || mode == "partition")
        m_mode = Mode::Partition;
    else if (mode == "round_robin")
        m_mode = Mode::RoundRobin;
    else
    {
        std::ostringstream oss;
        oss << getName() << ": Invalid 'mode' option '" << mode << "'. "
            "Valid options are 'partition' and 'round_robin'";
        throw pdal_error(oss.str());
    }
    if (options.hasOption("count") && options.hasOption("capacity"))
    {
        std::ostringstream oss;
        oss << getName() << ": Can't specify both option 'count' and "
            "option 'capacity.";
        throw pdal_error(oss.str());
    }
    if (!options.hasOption("count") && !options.hasOption("capacity"))
    {
        std::ostringstream oss;
        oss << getName() << ": Must specify either option 'count' or "
            "option 'capacity.";
        throw pdal_error(oss.str());
    }
    if (options.hasOption("count"))
    {
        m_size = options.getValueOrThrow<int>("count");
        m_sizeMode = SizeMode::Count;
        if (m_size < 2 || m_size > 1000)
        {
            std::ostringstream oss;
            oss << getName() << ": Option 'count' must be in the range "
                "[2, 1000].";
            throw pdal_error(oss.str());
        }
    }
    if (options.hasOption("capacity"))
    {
        m_size = options.getValueOrThrow<point_count_t>("capacity");
        m_sizeMode = SizeMode::Capacity;
    }
}


PointViewSet DividerFilter::run(PointViewPtr inView)
{
    if (m_sizeMode == SizeMode::Capacity)
        m_size = ((inView->size() - 1) / m_size) + 1;

    PointViewSet result;
    std::vector<PointViewPtr> views;
    for (point_count_t i = 0; i < m_size; ++i)
    {
        PointViewPtr v(inView->makeNew());
        views.push_back(v);
        result.insert(v);
    }

    if (m_mode == Mode::Partition)
    {
        point_count_t limit = ((inView->size() - 1) / m_size) + 1;
        unsigned viewNum = 0;
        for (PointId i = 0; i < inView->size();)
        {
            views[viewNum]->appendPoint(*inView, i++);
            if (i % limit == 0)
                viewNum++;
        }
    }
    else // RoundRobin
    {
        unsigned viewNum = 0;
        for (PointId i = 0; i < inView->size(); ++i)
        {
            views[viewNum]->appendPoint(*inView, i);
            viewNum++;
            if (viewNum == m_size)
                viewNum = 0;
        }
    }
    return result;
}

} // pdal
