/******************************************************************************
* Copyright (c) 2024, Howard Butler (howard@hobu.co)
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

#include "LabelDuplicatesFilter.hpp"

#include <string>
#include <vector>

#include <pdal/KDIndex.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "filters.label_duplicates",
    "Label duplicate points",
    "http://pdal.io/stages/filters.label_duplicates.html"
};

CREATE_STATIC_STAGE(LabelDuplicatesFilter, s_info)

std::string LabelDuplicatesFilter::getName() const
{
    return s_info.name;
}


LabelDuplicatesFilter::LabelDuplicatesFilter() : Filter()
{}




void LabelDuplicatesFilter::addArgs(ProgramArgs& args)
{
    args.add("dimensions", "Dimensions to use to declare points as duplicate", m_dimNames);
}


void LabelDuplicatesFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::Duplicate);
}

void LabelDuplicatesFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout = table.layout();
    for (std::string const& dimName : m_dimNames)
    {
        Dimension::Id dimId = layout->findDim(dimName);
        if (dimId == Dimension::Id::Unknown)
            throwError("Dimension '" + dimName + "' specified in "
                "'dimensions' option not found in layout.");
        m_dims.push_back(dimId);
    }
}

void LabelDuplicatesFilter::filter(PointView& view)
{
    log()->get(LogLevel::Debug) << "Finding duplicates...\n";

    // No duplicates if we have less than 2 points
    if (view.size() < 2)
        return;

    auto isDuplicatePoint = [&view, this](auto idx)
    {
        assert (idx > 0);

        for(auto dimId: m_dims)
        {
            double current = view.getFieldAs<double>(dimId, idx);
            double previous = view.getFieldAs<double>(dimId, idx - 1);
            if (current != previous)
                return false;
        }

        return true;
    };

    for (PointId idx = 1; idx < view.size(); ++idx)
    {
        view.setField(Dimension::Id::Duplicate, idx, (uint8_t)(isDuplicatePoint(idx)));
    }
}

} // namespace pdal
