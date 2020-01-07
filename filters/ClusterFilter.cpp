/******************************************************************************
 * Copyright (c) 2016, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "ClusterFilter.hpp"

#include "private/Segmentation.hpp"

#include <string>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.cluster",
    "Extract and label clusters using Euclidean distance.",
    "http://pdal.io/stages/filters.cluster.html"
};

CREATE_STATIC_STAGE(ClusterFilter, s_info)

std::string ClusterFilter::getName() const
{
    return s_info.name;
}

void ClusterFilter::addArgs(ProgramArgs& args)
{
    args.add("min_points", "Min points per cluster", m_minPoints,
        static_cast<uint64_t>(1));
    args.add("max_points", "Max points per cluster", m_maxPoints,
        (std::numeric_limits<uint64_t>::max)());
    args.add("tolerance", "Radius", m_tolerance, 1.0);
}

void ClusterFilter::filter(PointView& view)
{
    auto clusters = Segmentation::extractClusters(view, m_minPoints,
        m_maxPoints, m_tolerance);

    uint64_t id = 1;
    for (auto const& c : clusters)
    {
        for (auto const& i : c)
            view.setField(Dimension::Id::ClusterID, i, id);
        id++;
    }
}

} // namespace pdal
