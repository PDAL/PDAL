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

std::istream& operator>>(std::istream& in, PointClasses& classes)
{
    std::string s;

    classes.m_classes = 0;

    in >> s;
    s = Utils::tolower(s);
    StringList sl = Utils::split(s, ',');
    for (const std::string& c : sl)
    {
        if (c == "keypoint")
            classes.m_classes |= ClassLabel::Keypoint;
        else if (c == "synthetic")
            classes.m_classes |= ClassLabel::Synthetic;
        else if (c == "withheld")
            classes.m_classes |= ClassLabel::Withheld;
        else
            in.setstate(std::ios::failbit);
    }
    return in;
}

std::ostream& operator<<(std::ostream& out, const PointClasses& classes)
{
    std::string s;
    if (classes.m_classes & ClassLabel::Keypoint)
        s += "keypoint,";
    if (classes.m_classes & ClassLabel::Synthetic)
        s += "synthetic,";
    if (classes.m_classes & ClassLabel::Withheld)
        s += "withheld,";
    if (Utils::endsWith(s, ","))
        s.resize(s.size() - 1);
    out << s;
    return out;
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
                     PointViewPtr ignore, PointClasses classbits)
{
    using namespace Dimension;

    if (classbits.isNone())
    {
        keep->append(*input);
    }
    else
    {
        for (PointId i = 0; i < input->size(); ++i)
        {
            uint8_t c = input->getFieldAs<uint8_t>(Id::Classification, i);
            if (classbits.bits() & c)
                ignore->appendPoint(*input, i);
            else
                keep->appendPoint(*input, i);
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

PointIdList farthestPointSampling(PointView& view, point_count_t count)
{
    // Construct a KD-tree of the input view.
    KD3Index& kdi = view.build3dIndex();

    // Seed the output view with the first point in the current sorting.
    PointId seedId(0);
    PointIdList ids(count);
    ids[0] = seedId;

    // Compute distances from seedId to all other points.
    PointIdList indices(view.size());
    std::vector<double> sqr_dists(view.size());
    kdi.knnSearch(seedId, view.size(), &indices, &sqr_dists);

    // Sort distances by PointId.
    std::vector<double> min_dists(view.size());
    for (PointId i = 0; i < view.size(); ++i)
        min_dists[indices[i]] = sqr_dists[i];

    // Proceed until we have m_count points in the output PointView.
    for (PointId i = 1; i < count; ++i)
    {
        // Find the max distance in min_dists, this is the farthest point from
        // any point currently in the output PointView.
        auto it = std::max_element(min_dists.begin(), min_dists.end());

        // Record the PointId of the farthest point and add it to the output
        // PointView.
        PointId idx(it - min_dists.begin());
        ids[i] = idx;

        // Compute distances from idx to all other points.
        kdi.knnSearch(idx, view.size(), &indices, &sqr_dists);

        // Update distances.
        for (PointId j = 0; j < view.size(); ++j)
        {
            if (sqr_dists[j] < min_dists[indices[j]])
                min_dists[indices[j]] = sqr_dists[j];
        }
    }

    return ids;
}

} // namespace Segmentation
} // namespace pdal
