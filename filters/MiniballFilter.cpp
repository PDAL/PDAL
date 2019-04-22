/******************************************************************************
 * Copyright (c) 2019, Bradley J Chambers (brad.chambers@gmail.com)
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

// PDAL implementation of the miniball criterion presented in T. Weyrich, M.
// Pauly, R. Keiser, S. Heinzle, S. Scandella, and M. Gross, “Post-processing
// of Scanned 3D Surface Data,” Proc. Eurographics Symp.  Point-Based Graph.
// 2004, pp. 85–94, 2004.

#include "MiniballFilter.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "private/miniball/Seb.h"

#include <cmath>
#include <string>
#include <thread>
#include <vector>

namespace pdal
{

using namespace Dimension;

static StaticPluginInfo const s_info
{
    "filters.miniball",
    "Miniball (Kutz et al., 2003)",
    "http://pdal.io/stages/filters.miniball.html"
};

CREATE_STATIC_STAGE(MiniballFilter, s_info)

std::string MiniballFilter::getName() const
{
    return s_info.name;
}

void MiniballFilter::addArgs(ProgramArgs& args)
{
    args.add("knn", "k-Nearest neighbors", m_knn, 8);
    args.add("threads", "Number of threads used to run this filter", m_threads,
             1);
}

void MiniballFilter::addDimensions(PointLayoutPtr layout)
{
    m_miniball =
        layout->registerOrAssignDim("Miniball", Dimension::Type::Double);
}

void MiniballFilter::filter(PointView& view)
{
    KD3Index& kdi = view.build3dIndex();

    point_count_t nloops = view.size();
    std::vector<std::thread> threadPool(m_threads);
    for (int t = 0; t < m_threads; t++)
    {
        threadPool[t] = std::thread(std::bind(
            [&](const PointId start, const PointId end, const PointId t) {
                for (PointId i = start; i < end; i++)
                    setMiniball(view, i, kdi);
            },
            t * nloops / m_threads,
            (t + 1) == m_threads ? nloops : (t + 1) * nloops / m_threads, t));
    }
    for (auto& t : threadPool)
        t.join();
}

void MiniballFilter::setMiniball(PointView& view, const PointId& i,
                                 const KD3Index& kdi)
{
    typedef double FT;
    typedef Seb::Point<FT> Point;
    typedef std::vector<Point> PointVector;
    typedef Seb::Smallest_enclosing_ball<FT> Miniball;

    double X = view.getFieldAs<double>(Dimension::Id::X, i);
    double Y = view.getFieldAs<double>(Dimension::Id::Y, i);
    double Z = view.getFieldAs<double>(Dimension::Id::Z, i);

    // Find k-nearest neighbors of i.
    auto ni = kdi.neighbors(i, m_knn + 1);

    PointVector S;
    std::vector<double> coords(3);
    for (auto const& j : ni)
    {
        if (j == i)
            continue;
        coords[0] = view.getFieldAs<double>(Dimension::Id::X, j);
        coords[1] = view.getFieldAs<double>(Dimension::Id::Y, j);
        coords[2] = view.getFieldAs<double>(Dimension::Id::Z, j);
        S.push_back(Point(3, coords.begin()));
    }

    // add neighbors to Miniball mb(3, S)
    Miniball mb(3, S);

    // obtain radius r = mb.radius();
    FT radius = mb.radius();

    // obtain center = mb.center_begin()
    Miniball::Coordinate_iterator center_it = mb.center_begin();
    double x = center_it[0];
    double y = center_it[1];
    double z = center_it[2];

    // compute distance d from p to center
    double d =
        std::sqrt((X - x) * (X - x) + (Y - y) * (Y - y) + (Z - z) * (Z - z));

    double miniball = d / (d + 2 * radius / (std::sqrt(3)));
    view.setField(m_miniball, i, miniball);
}

} // namespace pdal
