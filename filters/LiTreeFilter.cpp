/******************************************************************************
 * Copyright (c) 2020, Bradley J Chambers (brad.chambers@gmail.com)
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

// PDAL implementation of W. Li, Q. Guo, M. K. Jakubowski, and M. Kelly, “A new
// method for segmenting individual trees from the lidar point cloud,”
// Photogramm. Eng. Remote Sensing, vol. 78, no. 1, pp. 75–84, 2012.

#include "LiTreeFilter.hpp"

#include <pdal/KDIndex.hpp>

#include <numeric>

namespace pdal
{

using namespace Dimension;

static PluginInfo const s_info{"filters.litree", "Li Tree Filter",
                               "http://pdal.io/stages/filters.litree.html"};

CREATE_STATIC_STAGE(LiTreeFilter, s_info)

std::string LiTreeFilter::getName() const
{
    return s_info.name;
}

void LiTreeFilter::addArgs(ProgramArgs& args)
{
    args.add("min_points", "Minimum number of points in a tree cluster",
             m_minSize, point_count_t(10));
    args.add("min_height",
             "Minimum height above ground to start a tree cluster", m_minHag,
             3.0);
    args.add("radius", "Dummy point located outside this approximate radius",
             m_dummyRadius, 100.0);
}

void LiTreeFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Id::ClusterID);
}

void LiTreeFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    if (!layout->hasDim(Id::HeightAboveGround))
        throwError("Missing HeightAboveGround dimension in input PointView.");
}

PointId LiTreeFilter::locateHighestPoint(PointView& view, PointIdList const& Ui)
{
    // In each iteration of the algorithm, we start by locating the point with
    // the highest HeightAboveGround among those still under consideration
    // (indexed by Ui).
    if (!view.size() || !Ui.size())
        throwError("Empty PointView or PointIdList.");
    PointId t0(Ui[0]);
    double vmax(view.getFieldAs<double>(Id::HeightAboveGround, Ui[0]));
    for (PointId const& i : Ui)
    {
        double val = view.getFieldAs<double>(Id::HeightAboveGround, i);
        if (val > vmax)
        {
            vmax = val;
            t0 = i;
        }
    }
    log()->get(LogLevel::Debug2) << "Max HAG value of " << vmax << std::endl;

    return t0;
}

PointId LiTreeFilter::locateDummyPoint(PointView& view, PointIdList const& Ui,
                                       PointId t0)
{
    PointViewPtr Uview = view.makeNew();
    for (PointId const& p : Ui)
        Uview->appendPoint(view, p);
    KD2Index& Utree = Uview->build2dIndex();

    PointRef p = view.point(t0);
    PointIdList dummy = Utree.radius(p, m_dummyRadius);
    PointId n0 = Ui[dummy.back()];

    return n0;
}

void LiTreeFilter::computeLocalMax(PointView& view)
{
    // Build the 2D KD-tree, which is used throughout.
    const KD2Index& kdi = view.build2dIndex();

    // Each point in the view will be marked as a local max or not.
    m_localMax.resize(view.size());
    for (PointRef p : view)
    {
        // Points are assumed to be locally maximal.
        m_localMax[p.pointId()] = 1;

        // Test this assumption considering neighbors within radius of 2m.
        PointIdList neighbors = kdi.radius(p, 2.0);
        double hag_p(p.getFieldAs<double>(Id::HeightAboveGround));
        for (PointId const& n : neighbors)
        {
            double hag_n = view.getFieldAs<double>(Id::HeightAboveGround, n);

            // If even one neighbor has a height greater than the current
            // point, the current point is not a local max and we can continue
            // iterating.
            if (hag_n > hag_p)
            {
                m_localMax[p.pointId()] = 0;
                break;
            }
        }
    }
}

void LiTreeFilter::classifyPoint(PointId ui, PointView& view, PointIdList& Ni,
                                 PointIdList& Pi)
{
    // Skip the points that were used to seed Pi and Ni, else they will be
    // repeated.
    if (std::count(Pi.begin(), Pi.end(), ui))
        return;
    if (std::count(Ni.begin(), Ni.end(), ui))
        return;

    PointRef u = view.point(ui);
    double Zu = u.getFieldAs<double>(Id::HeightAboveGround);

    PointViewPtr Nview = view.makeNew();
    for (PointId const& n : Ni)
        Nview->appendPoint(view, n);
    KD2Index& Ntree = Nview->build2dIndex();

    PointViewPtr Pview = view.makeNew();
    for (PointId const& p : Pi)
        Pview->appendPoint(view, p);
    KD2Index& Ptree = Pview->build2dIndex();

    // determine appropriate threshold based on HAG of current point
    double dt = 1.5;
    if (Zu > 15)
        dt = 2.0;

    // compute dmin1 and dmin2 (nearest neighbor in each set)
    PointIdList idx(1);
    std::vector<double> sqr_dists(1);
    Ptree.knnSearch(u, 1, &idx, &sqr_dists);
    double dmin1 = sqr_dists[0];
    Ntree.knnSearch(u, 1, &idx, &sqr_dists);
    double dmin2 = sqr_dists[0];

    if (!m_localMax[ui])
    {
        if (dmin1 <= dmin2)
            Pi.push_back(ui);
        else
            Ni.push_back(ui);
    }
    else
    {
        if (dmin1 > dt)
        {
            Ni.push_back(ui);
        }
        else
        {
            if (dmin1 <= dmin2)
                Pi.push_back(ui);
            else
                Ni.push_back(ui);
        }
    }
}

void LiTreeFilter::segmentTree(PointView& view, PointIdList& Ui,
                               int64_t& tree_id)
{
    // "The proposed segmentation algorithm isolates trees individually and
    // sequentially from the point cloud, from the tallest tree to the
    // shortest."

    // "Let Pi denote a set of points that belong to tree i (target), and Ni
    // denote a set of points that do not belong to tree i."
    PointIdList Pi;
    PointIdList Ni;

    log()->get(LogLevel::Debug) << "Classifying points for tree " << tree_id
                                << " (|Ui| = " << Ui.size() << ")\n";

    // "During each iteration, only one tree (target) is segmented, and the
    // points corresponding to this target tree are removed from the point
    // cloud."

    // "We find the highest point t0 (global maximum) in Ui, which is assumed
    // to be the top of the tallest tree i in Ui."
    PointId t0 = locateHighestPoint(view, Ui);
    if (view.getFieldAs<double>(Id::HeightAboveGround, t0) < m_minHag)
    {
        log()->get(LogLevel::Debug)
            << "Minimum height above ground not met; terminating.\n";
        return;
    }
    Pi.push_back(t0);

    // "We then insert a dummy point n0 that is far away (e.g., 100m) from t0
    // into Ni."
    PointId n0 = locateDummyPoint(view, Ui, t0);
    Ni.push_back(n0);

    // "For iteration i, we classify each point in Ui as Pi or Ni."
    for (PointId const& ui : Ui)
        classifyPoint(ui, view, Ni, Pi);

    log()->get(LogLevel::Debug3)
        << "|Pi| = " << Pi.size() << ", |Ni| = " << Ni.size() << std::endl;

    // Use Pi to assign current tree_id to TreeID dimension, only if minimum
    // size is met
    if (Pi.size() >= m_minSize)
    {
        for (PointId const& i : Pi)
            view.setField(Id::ClusterID, i, tree_id);
        tree_id++;
    }
    else
        log()->get(LogLevel::Debug)
            << "Minimum cluster size not met; skipping tree.\n";

    Ui.swap(Ni);
}

void LiTreeFilter::filter(PointView& view)
{
    // Preprocess cloud to determine which points are locally maximal with
    // respect to HeightAboveGround.
    computeLocalMax(view);

    // "Let Ui denote a set of points to be segmented."
    PointIdList Ui(view.size());
    std::iota(Ui.begin(), Ui.end(), 0);

    // "...stop when Ui is empty."
    int64_t tree_id(1);
    point_count_t prevSize(Ui.size());
    while (Ui.size() > 1)
    {
        segmentTree(view, Ui, tree_id);

        // Ui now contains all points not yet labeled with a TreeID. If it
        // remains unchanged between two iterations, it will be unable to
        // segment any further.
        if (Ui.size() == prevSize)
            break;
        prevSize = Ui.size();
    }
}

} // namespace pdal
