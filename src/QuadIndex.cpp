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

#include <limits>
#include <cmath>
#include <memory>

#include <pdal/QuadIndex.hpp>
#include <pdal/PointBuffer.hpp>

namespace pdal
{

// Helper classes for the quadtree implementation.

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
};

struct PointRef
{
    PointRef(const Point& point, std::size_t pbIndex)
        : point(point)
        , pbIndex(pbIndex)
    { }

    const Point point;
    const std::size_t pbIndex;
};

struct BBox
{
    BBox(Point min, Point max)
        : min(min)
        , max(max)
        , center(min.x + (max.x - min.x) / 2, min.y + (max.y - min.y) / 2)
        , halfWidth(center.x - min.x)
        , halfHeight(center.y - min.y)
    { }

    BBox(const BBox& other)
        : min(other.min)
        , max(other.max)
        , center(other.center)
        , halfWidth(other.halfWidth)
        , halfHeight(other.halfHeight)
    { }

    // Returns true if this BBox shares any area in common with another.
    bool overlaps(const BBox& other) const
    {
        return
            std::abs(center.x - other.center.x) <
                halfWidth + other.halfWidth &&
            std::abs(center.y - other.center.y) <
                halfHeight + other.halfHeight;
    }

    // Returns true if the requested point is contained within this BBox.
    bool contains(const Point& p) const
    {
        return p.x >= min.x && p.y >= min.y && p.x < max.x && p.y < max.y;
    }

    const Point min;
    const Point max;

    // Pre-calculate these properties, rather than exposing functions to
    // calculate them on-demand, due to the large number of times that
    // these will be needed when querying the quad tree.
    const Point center;
    const double halfWidth;
    const double halfHeight;
};

// Recursive quadtree implementation.
struct Tree
{
    Tree(BBox bbox, const PointRef* data = 0)
        : bbox(bbox)
        , data(data)
        , nw()
        , ne()
        , se()
        , sw()
    { }

    void addPoint(const PointRef* toAdd);

    void getPoints(
            std::vector<std::size_t>& results,
            std::size_t depthBegin,
            std::size_t depthEnd) const
    {
        getPoints(results, depthBegin, depthEnd, 0);
    }

    void getPoints(
            std::vector<std::size_t>& results,
            const BBox& query,
            std::size_t depthBegin,
            std::size_t depthEnd) const
    {
        getPoints(results, query, depthBegin, depthEnd, 0);
    }

    const BBox bbox;
    const PointRef* data;

    std::unique_ptr<Tree> nw;
    std::unique_ptr<Tree> ne;
    std::unique_ptr<Tree> se;
    std::unique_ptr<Tree> sw;

private:
    void getPoints(
            std::vector<std::size_t>& results,
            std::size_t depthBegin,
            std::size_t depthEnd,
            std::size_t curDepth) const;

    void getPoints(
            std::vector<std::size_t>& results,
            const BBox& query,
            std::size_t depthBegin,
            std::size_t depthEnd,
            std::size_t curDepth) const;
};

void Tree::addPoint(const PointRef* toAdd)
{
    if (data)
    {
        const Point& center(bbox.center);

        if (toAdd->point.sqDist(center) < data->point.sqDist(center))
        {
            std::swap(data, toAdd);
        }

        if (toAdd->point.x < center.x)
        {
            if (toAdd->point.y < center.y)
            {
                if (sw)
                {
                    sw->addPoint(toAdd);
                }
                else
                {
                    sw.reset(new Tree(
                            BBox(
                                Point(bbox.min.x, bbox.min.y),
                                Point(center.x, center.y)),
                            toAdd));
                }
            }
            else
            {
                if (nw)
                {
                    nw->addPoint(toAdd);
                }
                else
                {
                    nw.reset(new Tree(
                            BBox(
                                Point(bbox.min.x, center.y),
                                Point(center.x, bbox.max.y)),
                            toAdd));
                }
            }
        }
        else
        {
            if (toAdd->point.y < center.y)
            {
                if (se)
                {
                    se->addPoint(toAdd);
                }
                else
                {
                    se.reset(new Tree(
                            BBox(
                                Point(center.x, bbox.min.y),
                                Point(bbox.max.x, center.y)),
                            toAdd));
                }
            }
            else
            {
                if (ne)
                {
                    ne->addPoint(toAdd);
                }
                else
                {
                    ne.reset(new Tree(
                            BBox(
                                Point(center.x, center.y),
                                Point(bbox.max.x, bbox.max.y)),
                            toAdd));
                }
            }
        }
    }
    else
    {
        data = toAdd;
    }
}

void Tree::getPoints(
        std::vector<std::size_t>& results,
        const std::size_t depthBegin,
        const std::size_t depthEnd,
        std::size_t curDepth) const
{
    if (data && curDepth >= depthBegin)
    {
        results.push_back(data->pbIndex);
    }

    if (++curDepth < depthEnd || depthEnd == 0)
    {
        if (nw) nw->getPoints(results, depthBegin, depthEnd, curDepth);
        if (ne) ne->getPoints(results, depthBegin, depthEnd, curDepth);
        if (se) se->getPoints(results, depthBegin, depthEnd, curDepth);
        if (sw) sw->getPoints(results, depthBegin, depthEnd, curDepth);
    }
}

void Tree::getPoints(
        std::vector<std::size_t>& results,
        const BBox& query,
        const std::size_t depthBegin,
        const std::size_t depthEnd,
        std::size_t curDepth) const
{
    if (!query.overlaps(bbox))
    {
        return;
    }

    if (data && query.contains(data->point) && curDepth >= depthBegin)
    {
        results.push_back(data->pbIndex);
    }

    if (++curDepth < depthEnd || depthEnd == 0)
    {
        if (nw) nw->getPoints(results, query, depthBegin, depthEnd, curDepth);
        if (ne) ne->getPoints(results, query, depthBegin, depthEnd, curDepth);
        if (se) se->getPoints(results, query, depthBegin, depthEnd, curDepth);
        if (sw) sw->getPoints(results, query, depthBegin, depthEnd, curDepth);
    }
}

struct QuadIndex::QImpl
{
    QImpl(const PointBuffer& pointBuffer);

    void build();

    bool getBounds(
            double& xMin,
            double& yMin,
            double& xMax,
            double& yMax) const;

    std::vector<std::size_t> getPoints(
            std::size_t depthBegin,
            std::size_t depthEnd) const;

    std::vector<std::size_t> getPoints(
            double xMin,
            double yMin,
            double xMax,
            double yMax,
            std::size_t depthBegin,
            std::size_t depthEnd) const;

    const PointBuffer& m_pointBuffer;
    std::vector<std::unique_ptr<PointRef> > m_pointRefVec;
    std::unique_ptr<Tree> m_tree;
};

QuadIndex::QImpl::QImpl(const PointBuffer& pointBuffer)
    : m_pointBuffer(pointBuffer)
    , m_pointRefVec()
    , m_tree()
{ }

void QuadIndex::QImpl::build()
{
    m_pointRefVec.resize(m_pointBuffer.size());


    double xMin(std::numeric_limits<double>::max());
    double yMin(std::numeric_limits<double>::max());
    double xMax(std::numeric_limits<double>::min());
    double yMax(std::numeric_limits<double>::min());

    for (std::size_t i(0); i < m_pointBuffer.size(); ++i)
    {
        m_pointRefVec[i].reset(
                new PointRef(
                    Point(
                        m_pointBuffer.getFieldAs<double>(Dimension::Id::X, i),
                        m_pointBuffer.getFieldAs<double>(Dimension::Id::Y, i)),
                i));

        const PointRef* pointRef(m_pointRefVec[i].get());
        if (pointRef->point.x < xMin) xMin = pointRef->point.x;
        if (pointRef->point.x > xMax) xMax = pointRef->point.x;
        if (pointRef->point.y < yMin) yMin = pointRef->point.y;
        if (pointRef->point.y > yMax) yMax = pointRef->point.y;
    }

    m_tree.reset(new Tree(BBox(Point(xMin, yMin), Point(xMax, yMax))));

    for (std::size_t i = 0; i < m_pointRefVec.size(); ++i)
    {
        m_tree->addPoint(m_pointRefVec[i].get());
    }
}

bool QuadIndex::QImpl::getBounds(
        double& xMin,
        double& yMin,
        double& xMax,
        double& yMax) const
{
    if (m_tree)
    {
        xMin = m_tree->bbox.min.x;
        yMin = m_tree->bbox.min.y;
        xMax = m_tree->bbox.max.x;
        yMax = m_tree->bbox.max.y;
        return true;
    }
    else
    {
        return false;
    }
}

std::vector<std::size_t> QuadIndex::QImpl::getPoints(
        const std::size_t minDepth,
        const std::size_t maxDepth) const
{
    std::vector<std::size_t> results;

    if (m_tree)
    {
        m_tree->getPoints(results, minDepth, maxDepth);
    }

    return results;
}

std::vector<std::size_t> QuadIndex::QImpl::getPoints(
        double xMin,
        double yMin,
        double xMax,
        double yMax,
        std::size_t minDepth,
        std::size_t maxDepth) const
{
    std::vector<std::size_t> results;

    // Making BBox from external parameters here, so do some light validation.
    if (m_tree)
    {
        m_tree->getPoints(
                results,
                BBox(
                    Point(std::min(xMin, xMax), std::min(yMin, yMax)),
                    Point(std::max(xMin, xMax), std::max(yMin, yMax))),
                minDepth,
                maxDepth);
    }

    return results;
}

QuadIndex::QuadIndex(const PointBuffer& pointBuffer)
    : m_qImpl(new QImpl(pointBuffer))
{ }

QuadIndex::~QuadIndex()
{ }

void QuadIndex::build()
{
    m_qImpl->build();
}

bool QuadIndex::getBounds(
        double& xMin,
        double& yMin,
        double& xMax,
        double& yMax) const
{
    return m_qImpl->getBounds(xMin, yMin, xMax, yMax);
}

std::vector<std::size_t> QuadIndex::getPoints(
        std::size_t depthEnd) const
{
    return m_qImpl->getPoints(0, depthEnd);
}

std::vector<std::size_t> QuadIndex::getPoints(
        std::size_t depthBegin,
        std::size_t depthEnd) const
{
    return m_qImpl->getPoints(depthBegin, depthEnd);
}

std::vector<std::size_t> QuadIndex::getPoints(
        double xMin,
        double yMin,
        double xMax,
        double yMax,
        std::size_t depthEnd) const
{
    return m_qImpl->getPoints(
            xMin,
            yMin,
            xMax,
            yMax,
            0,
            depthEnd);
}

std::vector<std::size_t> QuadIndex::getPoints(
        double xMin,
        double yMin,
        double xMax,
        double yMax,
        std::size_t depthBegin,
        std::size_t depthEnd) const
{
    return m_qImpl->getPoints(
            xMin,
            yMin,
            xMax,
            yMax,
            depthBegin,
            depthEnd);
}

} // namespace pdal

