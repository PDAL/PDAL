/******************************************************************************
 * Copyright (c) 2016, 2017, 2019, 2020 Bradley J Chambers (brad.chambers@gmail.com)
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

// Normal refinement algorithm is presented in [1] and adapted to PDAL based on
// the implementation provided in [2].
//
// [1] H. Hoppe, T.  DeRose, T. Duchamp, J. McDonald, and W. Stuetzle, "Surface
// reconstruction from unorganized points," Computer Graphics, vol. 26. no. 2,
// pp. 71-78, 1992.
// [2] https://github.com/CloudCompare/CloudCompare.

#include "NormalFilter.hpp"
#include "private/Point.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/private/MathUtils.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace pdal
{

using namespace Eigen;
using namespace Dimension;

static StaticPluginInfo const s_info{
    "filters.normal", "Normal Filter",
    "http://pdal.io/stages/filters.normal.html"};

CREATE_STATIC_STAGE(NormalFilter, s_info)

struct NormalArgs
{
    int m_knn;
    filter::Point m_viewpoint;
    bool m_up;
    bool m_refine;
};

NormalFilter::NormalFilter() : m_args(new NormalArgs), m_count(0) {}

NormalFilter::~NormalFilter() {}

std::string NormalFilter::getName() const
{
    return s_info.name;
}

void NormalFilter::addArgs(ProgramArgs& args)
{
    args.add("knn", "k-Nearest Neighbors", m_args->m_knn, 8);
    m_viewpointArg = &args.add("viewpoint", "Viewpoint as WKT or GeoJSON",
                               m_args->m_viewpoint);
    args.add("always_up", "Normals always oriented with positive Z?",
             m_args->m_up, true);
    args.add("refine",
             "Refine normals using minimum spanning tree propagation?",
             m_args->m_refine, false);
}

void NormalFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims(
        {Id::NormalX, Id::NormalY, Id::NormalZ, Id::Curvature});
}

// public method to access filter, used by GreedyProjection and Poisson filters
void NormalFilter::doFilter(PointView& view, int knn)
{
    m_args->m_knn = knn;
    ProgramArgs args;
    addArgs(args);
    // We're never parsing anything, so we'll just end up with default vals.
    // This makes sure that the arg pointer (m_viewpointArg) is valid.
    filter(view);
}

void NormalFilter::prepared(PointTableRef table)
{
    if (m_args->m_up && m_viewpointArg->set())
    {
        log()->get(LogLevel::Warning)
            << "Viewpoint provided. Ignoring always_up = TRUE." << std::endl;
        m_args->m_up = false;
    }

    // The query point is returned as a neighbor of itself, so we must increase
    // k by one to get the desired number of neighbors.
    ++m_args->m_knn;
}

void NormalFilter::compute(PointView& view, KD3Index& kdi)
{
    log()->get(LogLevel::Debug) << "Computing normal vectors\n";
    for (auto&& p : view)
    {
        // Perform eigen decomposition of covariance matrix computed from
        // neighborhood composed of k-nearest neighbors.
        PointIdList neighbors = kdi.neighbors(p.pointId(), m_args->m_knn);
        auto B = math::computeCovariance(view, neighbors);
        SelfAdjointEigenSolver<Matrix3d> solver(B);
        if (solver.info() != Success)
            throwError("Cannot perform eigen decomposition.");

        // The curvature is computed as the ratio of the first (smallest)
        // eigenvalue to the sum of all eigenvalues.
        auto eval = solver.eigenvalues();
        double sum = eval[0] + eval[1] + eval[2];
        double curvature = sum ? std::fabs(eval[0] / sum) : 0;

        // The normal is defined by the eigenvector corresponding to the
        // smallest eigenvalue.
        Vector3d normal = solver.eigenvectors().col(0);

        if (m_viewpointArg->set())
        {
            // If a viewpoint has been specified, orient the normals to face the
            // viewpoint by taking the dot product of the vector connecting the
            // point with the viewpoint and the normal. Flip the normal, where
            // the dot product is negative.
            double dx = m_args->m_viewpoint.x() - p.getFieldAs<double>(Id::X);
            double dy = m_args->m_viewpoint.y() - p.getFieldAs<double>(Id::Y);
            double dz = m_args->m_viewpoint.z() - p.getFieldAs<double>(Id::Z);
            Vector3d vp(dx, dy, dz);
            if (vp.dot(normal) < 0)
                normal *= -1.0;
        }
        else if (m_args->m_up)
        {
            // If normals are expected to be upward facing, invert them when the
            // Z component is negative.
            if (normal[2] < 0)
                normal *= -1.0;
        }

        // Set the computed normal and curvature dimensions.
        p.setField(Id::NormalX, normal[0]);
        p.setField(Id::NormalY, normal[1]);
        p.setField(Id::NormalZ, normal[2]);
        p.setField(Id::Curvature, curvature);
    }
}

void NormalFilter::update(
    PointView& view, KD3Index& kdi, std::vector<bool> inMST,
    std::priority_queue<Edge, EdgeList, CompareEdgeWeight> edge_queue,
    PointId updateIdx)
{
    // Add the current PointId to the minimum spanning tree.
    inMST[updateIdx] = true;
    ++m_count;

    // Consider neighbors of the newly added PointId, adding them to
    // the edge queue if they are not already part of the minimum
    // spanning tree. The first neighbor is the query point which is
    // already part of the minimum spanning tree and can safely be
    // skipped.
    PointIdList neighbors = kdi.neighbors(updateIdx, m_args->m_knn);
    neighbors.erase(neighbors.begin());
    PointRef p = view.point(updateIdx);
    Vector3d N0(p.getFieldAs<double>(Id::NormalX),
                p.getFieldAs<double>(Id::NormalY),
                p.getFieldAs<double>(Id::NormalZ));
    for (PointId const& neighborIdx : neighbors)
    {
        if (!inMST[neighborIdx])
        {
            PointRef q = view.point(neighborIdx);
            Vector3d N1(q.getFieldAs<double>(Id::NormalX),
                        q.getFieldAs<double>(Id::NormalY),
                        q.getFieldAs<double>(Id::NormalZ));
            double weight = 1.0 - std::fabs(N0.dot(N1));
            edge_queue.emplace(updateIdx, neighborIdx, weight);
        }
    }
}

void NormalFilter::refine(PointView& view, KD3Index& kdi)
{
    log()->get(LogLevel::Debug)
        << "Refining normals using minimum spanning tree\n";

    std::priority_queue<Edge, EdgeList, CompareEdgeWeight> edge_queue;
    std::vector<bool> inMST(view.size(), false);
    PointId nextIdx(0);
    while (m_count < view.size())
    {
        // Find the PointId of the next point not currently part of the minimum
        // spanning tree.
        while (inMST[nextIdx])
            ++nextIdx;

        update(view, kdi, inMST, edge_queue, nextIdx);

        // Iterate on the edge queue until empty (or all points have been added
        // to the minimum spanning tree).
        while (!edge_queue.empty() && (m_count < view.size()))
        {
            // Retrieve the edge with the smallest weight.
            Edge edge(edge_queue.top());
            edge_queue.pop();

            // Record the PointId and normal of the PointId (if one exists)
            // that is not already in the minimum spanning tree.
            PointId newIdx(0);
            Vector3d normal;
            PointRef p = view.point(edge.m_v0);
            Vector3d N0(p.getFieldAs<double>(Id::NormalX),
                        p.getFieldAs<double>(Id::NormalY),
                        p.getFieldAs<double>(Id::NormalZ));
            PointRef q = view.point(edge.m_v1);
            Vector3d N1(q.getFieldAs<double>(Id::NormalX),
                        q.getFieldAs<double>(Id::NormalY),
                        q.getFieldAs<double>(Id::NormalZ));
            if (!inMST[edge.m_v0])
            {
                newIdx = edge.m_v0;
                normal = N0;
            }
            else if (!inMST[edge.m_v1])
            {
                newIdx = edge.m_v1;
                normal = N1;
            }
            else
                continue;

            // Where the dot product of the normals is less than 0, invert the
            // normal of the selected PointId.
            if (N0.dot(N1) < 0)
            {
                normal *= -1;
                view.setField(Id::NormalX, newIdx, normal(0));
                view.setField(Id::NormalY, newIdx, normal(1));
                view.setField(Id::NormalZ, newIdx, normal(2));
            }

            update(view, kdi, inMST, edge_queue, newIdx);
        }
    }
}

void NormalFilter::filter(PointView& view)
{
    KD3Index& kdi = view.build3dIndex();

    // Compute the normal/curvature and optionally orient toward viewpoint or
    // positive Z.
    compute(view, kdi);

    // If requested, refine normals through minimum spanning tree propagation.
    if (m_args->m_refine)
        refine(view, kdi);
}

} // namespace pdal
