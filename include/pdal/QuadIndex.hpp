/******************************************************************************
* Copyright (c) 2014, Connor Manning (connor@hobu.co)
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

#pragma once

#include <vector>
#include <memory>

#include <pdal/pdal_export.hpp>
#include <pdal/pdal_types.hpp>
#include <pdal/util/Bounds.hpp>

namespace pdal
{

class PointView;

struct Point
{
    Point(double x, double y) : x(x), y(y) { }
    Point(const Point& other) : x(other.x), y(other.y) { }

    // Calculates the distance-squared to another point.
    double sqDist(const Point& other) const
    {
        return (x - other.x) * (x - other.x) + (y - other.y) * (y - other.y);
    }

    const double x;
    const double y;

    Point& operator=(const Point&); // not implemented
};

struct QuadPointRef
{
    QuadPointRef(const Point& point, std::size_t pbIndex)
        : point(point)
        , pbIndex(pbIndex)
    { }

    const Point point;
    const std::size_t pbIndex;

    QuadPointRef& operator=(const QuadPointRef&); // not implemented
    QuadPointRef(const QuadPointRef&); // not implemented
};

class PDAL_DLL QuadIndex
{
public:
    QuadIndex(const PointView& view, std::size_t topLevel = 0);
    QuadIndex(
            const PointView& view,
            double xMin,
            double yMin,
            double xMax,
            double yMax,
            std::size_t topLevel = 0);
    QuadIndex(
            const std::vector<std::shared_ptr<QuadPointRef> >& points,
            double xMin,
            double yMin,
            double xMax,
            double yMax,
            std::size_t topLevel = 0);
    ~QuadIndex();

    void getBounds(
            double& xMin,
            double& yMin,
            double& xMax,
            double& yMax) const;

    std::size_t getDepth() const;

    std::vector<std::size_t> getFills() const;

    // Return all points at depth levels strictly less than depthEnd.
    // A depthEnd value of zero returns all points in the tree.
    std::vector<PointId> getPoints(
            std::size_t depthEnd = 0) const;

    // Return all points at depth levels between [depthBegin, depthEnd).
    // A depthEnd value of zero will return all points at levels >= depthBegin.
    std::vector<PointId> getPoints(
            std::size_t depthBegin,
            std::size_t depthEnd) const;

    // Rasterize a single level of the tree.  Empty positions will contain
    // std::numeric_limits<PointId>::max().
    std::vector<PointId> getPoints(
            std::size_t rasterize,
            double& xBegin,
            double& xEnd,
            double& xStep,
            double& yBegin,
            double& yEnd,
            double& yStep) const;

    // Get custom raster via bounds and resolution query.  Empty positions will
    // contain std::numeric_limits<PointId>::max().
    std::vector<PointId> getPoints(
            double xBegin,
            double xEnd,
            double xStep,
            double yBegin,
            double yEnd,
            double yStep) const;

    // Return all points within the query bounding box, searching only up to
    // depth levels strictly less than depthEnd.
    // A depthEnd value of zero will return all existing points that fall
    // within the query range regardless of depth.
    std::vector<PointId> getPoints(
            double xMin,
            double yMin,
            double xMax,
            double yMax,
            std::size_t depthEnd = 0) const;

    std::vector<PointId> getPoints(
            const BOX3D& box,
            std::size_t depthEnd=0) const
    {
        return getPoints(box.minx, box.miny, box.maxx, box.maxy, depthEnd);
    }

    // Return all points within the bounding box, searching at tree depth
    // levels from [depthBegin, depthEnd).
    // A depthEnd value of zero will return all points within the query range
    // that have a tree level >= depthBegin.
    std::vector<PointId> getPoints(
            double xMin,
            double yMin,
            double xMax,
            double yMax,
            std::size_t depthBegin,
            std::size_t depthEnd) const;

private:
    struct QImpl;
    std::unique_ptr<QImpl> m_qImpl;

    // Disable copying and assignment.
    QuadIndex(const QuadIndex&);
    QuadIndex& operator=(QuadIndex&);
};

} // namespace pdal

