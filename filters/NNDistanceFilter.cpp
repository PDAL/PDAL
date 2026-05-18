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

#include "NNDistanceFilter.hpp"

#include <string>
#include <vector>

#include <pdal/KDIndex.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "filters.nndistance",
    "NN-Distance Filter",
    "https://pdal.org/stages/filters.nndistance.html"
};

CREATE_STATIC_STAGE(NNDistanceFilter, s_info)

std::string NNDistanceFilter::getName() const
{
    return s_info.name;
}


NNDistanceFilter::NNDistanceFilter() : Filter()
{}


std::istream& operator>>(std::istream& in, NNDistanceFilter::Mode& mode)
{
    std::string s;
    in >> s;

    s = Utils::tolower(s);
    if (s == "kth")
        mode = NNDistanceFilter::Mode::Kth;
    else if (s == "avg")
        mode = NNDistanceFilter::Mode::Average;
    else
        in.setstate(std::ios_base::failbit);
    return in;
}


std::ostream& operator<<(std::ostream& out, const NNDistanceFilter::Mode& mode)
{
    switch (mode)
    {
    case NNDistanceFilter::Mode::Kth:
        out << "kth";
    case NNDistanceFilter::Mode::Average:
        out << "avg";
    }
    return out;
}


void NNDistanceFilter::addArgs(ProgramArgs& args)
{
    args.add("mode", "Distance computation mode (kth, avg)", m_mode, Mode::Kth);
    args.add("k", "k neighbors", m_k, size_t(10));
}


void NNDistanceFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::NNDistance);
}


void NNDistanceFilter::filter(PointView& view)
{
    using namespace Dimension;

    // Build the 3D KD-tree.
    KD3Index& index = view.build3dIndex();

    // Increment the minimum number of points, as knnSearch will be returning
    // the query point along with the neighbors.
    size_t k = m_k + 1;

    // Compute the k-distance for each point. The k-distance is the Euclidean
    // distance to k-th nearest neighbor.
    log()->get(LogLevel::Debug) << "Computing k-distances...\n";
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        PointIdList indices(k);
        std::vector<double> sqr_dists(k);
        index.knnSearch(idx, k, &indices, &sqr_dists);
        double val;
        if (m_mode == Mode::Kth)
            val = std::sqrt(sqr_dists[k - 1]);
        else // m_mode == Mode::Average
        {
            val = 0;

            // We start at 1 since index 0 is the test point.
            for (size_t i = 1; i < k; ++i)
                val += std::sqrt(sqr_dists[i]);
            val /= (k - 1);
        }
        view.setField(Dimension::Id::NNDistance, idx, val);
    }
}

} // namespace pdal
