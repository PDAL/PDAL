/******************************************************************************
 * Copyright (c) 2016-2017, Bradley J. Chambers (brad.chambers@gmail.com)
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

#include <pdal/PDALUtils.hpp>

#include <pdal/KDIndex.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Stage.hpp>
#include <pdal/pdal_types.hpp>

#include "DimRange.hpp"
#include "Segmentation.hpp"

#include <vector>

namespace pdal
{

namespace Segmentation
{

std::vector<PointIdList> extractClusters(PointView& view, uint64_t min_points,
                                         uint64_t max_points, double tolerance)
{
    // Index the incoming PointView for subsequent radius searches.
    KD3Index kdi(view);
    kdi.build();

    // Create variables to track PointIds that have already been added to
    // clusters and to build the list of cluster indices.
    PointIdList processed(view.size(), 0);
    std::vector<PointIdList> clusters;

    for (PointId i = 0; i < view.size(); ++i)
    {
        // Points can only belong to a single cluster.
        if (processed[i])
            continue;

        // Initialize list of indices belonging to current cluster, marking the
        // seed point as processed.
        PointIdList seed_queue;
        size_t sq_idx = 0;
        seed_queue.push_back(i);
        processed[i] = 1;

        // Check each point in the cluster for additional neighbors within the
        // given tolerance, remembering that the list can grow if we add points
        // to the cluster.
        while (sq_idx < seed_queue.size())
        {
            // Find neighbors of the next cluster point.
            PointId j = seed_queue[sq_idx];
            PointIdList ids = kdi.radius(j, tolerance);

            // The case where the only neighbor is the query point.
            if (ids.size() == 1)
            {
                sq_idx++;
                continue;
            }

            // Skip neighbors that already belong to a cluster and add the rest
            // to this cluster.
            for (auto const& k : ids)
            {
                if (processed[k])
                    continue;
                seed_queue.push_back(k);
                processed[k] = 1;
            }

            sq_idx++;
        }

        // Keep clusters that are within the min/max number of points.
        if (seed_queue.size() >= min_points && seed_queue.size() <= max_points)
            clusters.push_back(seed_queue);
    }

    return clusters;
}

void ignoreDimRange(DimRange dr, PointViewPtr input, PointViewPtr keep,
                    PointViewPtr ignore)
{
    PointRef point(*input, 0);
    for (PointId i = 0; i < input->size(); ++i)
    {
        point.setPointId(i);
        if (dr.valuePasses(point.getFieldAs<double>(dr.m_id)))
            ignore->appendPoint(*input, i);
        else
            keep->appendPoint(*input, i);
    }
}

void ignoreDimRanges(std::vector<DimRange>& ranges, PointViewPtr input,
    PointViewPtr keep, PointViewPtr ignore)
{
    std::sort(ranges.begin(), ranges.end());
    PointRef point(*input, 0);
    for (PointId i = 0; i < input->size(); ++i)
    {
        point.setPointId(i);
        if (DimRange::pointPasses(ranges, point))
            ignore->appendPoint(*input, i);
        else
            keep->appendPoint(*input, i);
    }
}

void ignoreClassBits(PointViewPtr input, PointViewPtr keep,
                     PointViewPtr ignore, StringList classbits)
{
    using namespace Dimension;

    bool ignoreSynthetic = false;
    bool ignoreKeypoint = false;
    bool ignoreWithheld = false;

    if (!classbits.size())
    {
        keep->append(*input);
    }
    else
    {
        for (auto& b : classbits)
        {
            Utils::trim(b);
            if (b == "synthetic")
                ignoreSynthetic = true;
            else if (b == "keypoint")
                ignoreKeypoint = true;
            else if (b == "withheld")
                ignoreWithheld = true;
        }

        for (PointId i = 0; i < input->size(); ++i)
        {
            uint8_t c = input->getFieldAs<uint8_t>(Id::Classification, i);
	    if (((c & ClassLabel::Synthetic) && ignoreSynthetic) ||
                ((c & ClassLabel::Keypoint) && ignoreKeypoint) ||
		((c & ClassLabel::Withheld) && ignoreWithheld))
            {
                ignore->appendPoint(*input, i);
            }
	    else
            {
                keep->appendPoint(*input, i);
            }
        }
    }
}

void segmentLastReturns(PointViewPtr input, PointViewPtr last,
                        PointViewPtr other)
{
    using namespace Dimension;

    for (PointId i = 0; i < input->size(); ++i)
    {
        uint8_t rn = input->getFieldAs<uint8_t>(Id::ReturnNumber, i);
        uint8_t nr = input->getFieldAs<uint8_t>(Id::NumberOfReturns, i);
        if ((rn == nr) && (nr > 1))
            last->appendPoint(*input, i);
        else
            other->appendPoint(*input, i);
    }
}

void segmentReturns(PointViewPtr input, PointViewPtr first,
                    PointViewPtr second, StringList returns)
{
    using namespace Dimension;

    bool returnFirst = false;
    bool returnIntermediate = false;
    bool returnLast = false;
    bool returnOnly = false;

    if (!returns.size())
    {
        first->append(*input);
    }
    else
    {
        for (auto& r : returns)
        {
            Utils::trim(r);
            if (r == "first")
                returnFirst = true;
            else if (r == "intermediate")
                returnIntermediate = true;
            else if (r == "last")
                returnLast = true;
            else if (r == "only")
                returnOnly = true;
        }

        for (PointId i = 0; i < input->size(); ++i)
        {
            uint8_t rn = input->getFieldAs<uint8_t>(Id::ReturnNumber, i);
            uint8_t nr = input->getFieldAs<uint8_t>(Id::NumberOfReturns, i);

            if ((((rn == 1) && (nr > 1)) && returnFirst) ||
                (((rn > 1) && (rn < nr)) && returnIntermediate) ||
                (((rn == nr) && (nr > 1)) && returnLast) ||
                ((nr == 1) && returnOnly))
            {
                first->appendPoint(*input.get(), i);
            }
            else
            {
                second->appendPoint(*input.get(), i);
            }
        }
    }
}

} // namespace Segmentation
} // namespace pdal
