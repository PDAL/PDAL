/******************************************************************************
 * Copyright (c) 2024, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "SparseSurfaceFilter.hpp"

#include <pdal/KDIndex.hpp>

namespace pdal
{

using namespace Dimension;

static PluginInfo const s_info{"filters.sparsesurface", "Sparse Surface Filter",
                               "http://pdal.org/stages/filters.sparsesurface.html"};

CREATE_STATIC_STAGE(SparseSurfaceFilter, s_info)

std::string SparseSurfaceFilter::getName() const
{
    return s_info.name;
}

void SparseSurfaceFilter::addArgs(ProgramArgs& args)
{
    args.add("radius", "Mask neighbor points as low noise",
             m_radius, 1.0);
}

void SparseSurfaceFilter::filter(PointView& view)
{
    // Step 1: Relabel all points as unclassified
    for (auto point : view)
        point.setField(Id::Classification, ClassLabel::Unclassified);

    // Step 2: Generate a sorted Z index
    PointIdList zIndex(view.size());
    std::iota(zIndex.begin(), zIndex.end(), 0);

    auto zComparator = [&view](size_t a, size_t b) {
        return view.getFieldAs<double>(Id::Z, a) < 
                view.getFieldAs<double>(Id::Z, b);
    };

    std::sort(zIndex.begin(), zIndex.end(), zComparator);

    // Create a KD-tree index for the point view
    pdal::KD2Index& index = view.build2dIndex();

    // Steps 3-5: Ground labeling and neighbor processing
    for (PointId idx : zIndex) {
        // Check if the point is already labeled
        uint8_t classification = view.getFieldAs<uint8_t>(Id::Classification, idx);
        if (classification != ClassLabel::Unclassified)
            continue;

        // Label the point as ground
        view.setField(Id::Classification, idx, ClassLabel::Ground);

        // Find neighbors within radius R and label them as low noise
        PointIdList neighbors = index.radius(idx, m_radius);
        for (PointId neighborIdx : neighbors) {
            uint8_t neighborClass = view.getFieldAs<uint8_t>(Id::Classification, neighborIdx);
            if (neighborClass == ClassLabel::Unclassified) {
                view.setField(Id::Classification, neighborIdx, ClassLabel::LowPoint);
            }
        }
    }
}

}
