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
#include <pdal/Utils.hpp>

namespace
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

struct PtRef
{
    PtRef(const Point& point, std::size_t pbIndex)
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

    bool overlaps(
        const double xBegin,
        const double xEnd,
        const double yBegin,
        const double yEnd) const
    {
        const BBox other(
                Point(xBegin, yBegin),
                Point(xEnd, yEnd));

        return overlaps(other);
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

} // anonymous namespace

namespace pdal
{

// Recursive quadtree implementation.
struct Tree
{
    Tree(BBox bbox, const PtRef* data = 0)
        : bbox(bbox)
        , data(data)
        , nw()
        , ne()
        , se()
        , sw()
    { }

    void getFills(std::vector<std::size_t>& fills, std::size_t level = 0) const;

    // Returns depth resulting from the insertion of this point.
    std::size_t addPoint(const PtRef* toAdd, std::size_t curDepth = 0);

    void getPoints(
            std::vector<std::size_t>& results,
            std::size_t depthBegin,
            std::size_t depthEnd) const
    {
        getPoints(results, depthBegin, depthEnd, 0);
    }

    void getPoints(
            std::vector<std::size_t>& results,
            std::size_t rasterize,
            double xBegin,
            double xEnd,
            double xStep,
            double yBegin,
            double yEnd,
            double yStep) const
    {
        getPoints(
                results,
                rasterize,
                xBegin,
                xEnd,
                xStep,
                yBegin,
                yEnd,
                yStep,
                0);
    }

    void getPoints(
            std::vector<std::size_t>& results,
            double xBegin,
            double xEnd,
            double xStep,
            double yBegin,
            double yEnd,
            double yStep) const;

    void getPoints(
            std::vector<std::size_t>& results,
            const BBox& query,
            std::size_t depthBegin,
            std::size_t depthEnd) const
    {
        getPoints(results, query, depthBegin, depthEnd, 0);
    }

    const BBox bbox;
    const PtRef* data;

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
            std::size_t rasterize,
            double xBegin,
            double xEnd,
            double xStep,
            double yBegin,
            double yEnd,
            double yStep,
            std::size_t curDepth) const;

    void getPoints(
            std::vector<std::size_t>& results,
            const BBox& query,
            std::size_t depthBegin,
            std::size_t depthEnd,
            std::size_t curDepth) const;
};

std::size_t Tree::addPoint(const PtRef* toAdd, const std::size_t curDepth)
{
    if (data)
    {
        const Point& center(bbox.center);

        if (toAdd->point.sqDist(center) < data->point.sqDist(center))
        {
            std::swap(data, toAdd);
        }

        const std::size_t nextDepth(curDepth + 1);

        if (toAdd->point.x < center.x)
        {
            if (toAdd->point.y < center.y)
            {
                if (sw)
                {
                    return sw->addPoint(toAdd, nextDepth);
                }
                else
                {
                    sw.reset(new Tree(
                            BBox(
                                Point(bbox.min.x, bbox.min.y),
                                Point(center.x, center.y)),
                            toAdd));

                    return nextDepth;
                }
            }
            else
            {
                if (nw)
                {
                    return nw->addPoint(toAdd, nextDepth);
                }
                else
                {
                    nw.reset(new Tree(
                            BBox(
                                Point(bbox.min.x, center.y),
                                Point(center.x, bbox.max.y)),
                            toAdd));

                    return nextDepth;
                }
            }
        }
        else
        {
            if (toAdd->point.y < center.y)
            {
                if (se)
                {
                    return se->addPoint(toAdd, nextDepth);
                }
                else
                {
                    se.reset(new Tree(
                            BBox(
                                Point(center.x, bbox.min.y),
                                Point(bbox.max.x, center.y)),
                            toAdd));

                    return nextDepth;
                }
            }
            else
            {
                if (ne)
                {
                    return ne->addPoint(toAdd, nextDepth);
                }
                else
                {
                    ne.reset(new Tree(
                            BBox(
                                Point(center.x, center.y),
                                Point(bbox.max.x, bbox.max.y)),
                            toAdd));

                    return nextDepth;
                }
            }
        }
    }
    else
    {
        data = toAdd;
        return curDepth;
    }
}

void Tree::getFills(std::vector<std::size_t>& fills, std::size_t level) const
{
    if (data)
    {
        if (level >= fills.size())
        {
            fills.resize(level + 1);
        }

        ++fills.at(level);
    }

    ++level;

    if (nw) nw->getFills(fills, level);
    if (ne) ne->getFills(fills, level);
    if (sw) sw->getFills(fills, level);
    if (se) se->getFills(fills, level);
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
        const std::size_t rasterize,
        const double xBegin,
        const double xEnd,
        const double xStep,
        const double yBegin,
        const double yEnd,
        const double yStep,
        std::size_t curDepth) const
{
    if (curDepth == rasterize)
    {
        if (data)
        {
            const std::size_t xOffset(
                    Utils::sround((bbox.center.x - xBegin) / xStep));
            const double yOffset(
                    Utils::sround((bbox.center.y - yBegin) / yStep));

            const std::size_t index(
                Utils::sround(yOffset * (xEnd - xBegin) / xStep + xOffset));

            results.at(index) = data->pbIndex;
        }
    }
    else
    {
        ++curDepth;

        if (nw) nw->getPoints(
                results,
                rasterize,
                xBegin,
                xEnd,
                xStep,
                yBegin,
                yEnd,
                yStep,
                curDepth);

        if (ne) ne->getPoints(
                results,
                rasterize,
                xBegin,
                xEnd,
                xStep,
                yBegin,
                yEnd,
                yStep,
                curDepth);

        if (se) se->getPoints(
                results,
                rasterize,
                xBegin,
                xEnd,
                xStep,
                yBegin,
                yEnd,
                yStep,
                curDepth);

        if (sw) sw->getPoints(
                results,
                rasterize,
                xBegin,
                xEnd,
                xStep,
                yBegin,
                yEnd,
                yStep,
                curDepth);
    }
}

void Tree::getPoints(
        std::vector<std::size_t>& results,
        const double xBegin,
        const double xEnd,
        const double xStep,
        const double yBegin,
        const double yEnd,
        const double yStep) const
{
    if (!bbox.overlaps(xBegin, xEnd, yBegin, yEnd))
    {
        return;
    }

    if (nw) nw->getPoints(
            results,
            xBegin,
            xEnd,
            xStep,
            yBegin,
            yEnd,
            yStep);

    if (ne) ne->getPoints(
            results,
            xBegin,
            xEnd,
            xStep,
            yBegin,
            yEnd,
            yStep);

    if (se) se->getPoints(
            results,
            xBegin,
            xEnd,
            xStep,
            yBegin,
            yEnd,
            yStep);

    if (sw) sw->getPoints(
            results,
            xBegin,
            xEnd,
            xStep,
            yBegin,
            yEnd,
            yStep);

    // Add data after calling child nodes so we prefer upper levels of the tree.
    if (
            data &&
            data->point.x >= xBegin &&
            data->point.y >= yBegin &&
            data->point.x < xEnd - xStep &&
            data->point.y < yEnd - yStep)
    {
        const std::size_t xOffset(
                Utils::sround((data->point.x - xBegin) / xStep));
        const std::size_t yOffset(
                Utils::sround((data->point.y - yBegin) / yStep));

        const std::size_t index(
            Utils::sround(yOffset * (xEnd - xBegin) / xStep + xOffset));

        if (index < results.size())
        {
            results.at(index) = data->pbIndex;
        }
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

    std::size_t getDepth() const;

    std::vector<std::size_t> getFills();

    std::vector<std::size_t> getPoints(
            std::size_t depthBegin,
            std::size_t depthEnd) const;

    std::vector<std::size_t> getPoints(
            std::size_t rasterize,
            double& xBegin,
            double& xEnd,
            double& xStep,
            double& yBegin,
            double& yEnd,
            double& yStep) const;

    std::vector<std::size_t> getPoints(
            double xBegin,
            double xEnd,
            double xStep,
            double yBegin,
            double yEnd,
            double yStep) const;

    std::vector<std::size_t> getPoints(
            double xMin,
            double yMin,
            double xMax,
            double yMax,
            std::size_t depthBegin,
            std::size_t depthEnd) const;

    const PointBuffer& m_pointBuffer;
    std::vector<std::unique_ptr<PtRef> > m_pointRefVec;
    std::unique_ptr<Tree> m_tree;
    std::size_t m_depth;
    std::vector<std::size_t> m_fills;
};

QuadIndex::QImpl::QImpl(const PointBuffer& pointBuffer)
    : m_pointBuffer(pointBuffer)
    , m_pointRefVec()
    , m_tree()
    , m_depth(0)
    , m_fills()
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
                new PtRef(
                    Point(
                        m_pointBuffer.getFieldAs<double>(Dimension::Id::X, i),
                        m_pointBuffer.getFieldAs<double>(Dimension::Id::Y, i)),
                i));

        const PtRef* pointRef(m_pointRefVec[i].get());
        if (pointRef->point.x < xMin) xMin = pointRef->point.x;
        if (pointRef->point.x > xMax) xMax = pointRef->point.x;
        if (pointRef->point.y < yMin) yMin = pointRef->point.y;
        if (pointRef->point.y > yMax) yMax = pointRef->point.y;
    }

    m_tree.reset(new Tree(BBox(Point(xMin, yMin), Point(xMax, yMax))));

    for (std::size_t i = 0; i < m_pointRefVec.size(); ++i)
    {
        m_depth = std::max(m_tree->addPoint(m_pointRefVec[i].get()), m_depth);
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

std::size_t QuadIndex::QImpl::getDepth() const
{
    return m_depth;
}

std::vector<std::size_t> QuadIndex::QImpl::getFills()
{
    if (m_tree && !m_fills.size())
    {
        m_tree->getFills(m_fills);
    }

    return m_fills;
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
        const std::size_t rasterize,
        double& xBegin,
        double& xEnd,
        double& xStep,
        double& yBegin,
        double& yEnd,
        double& yStep) const
{
    std::vector<std::size_t> results;

    if (m_tree)
    {
        const std::size_t exp(std::pow(2, rasterize));
        const double xWidth(m_tree->bbox.max.x - m_tree->bbox.min.x);
        const double yWidth(m_tree->bbox.max.y - m_tree->bbox.min.y);

        xStep = xWidth / exp;
        yStep = yWidth / exp;
        xBegin =    m_tree->bbox.min.x + (xStep / 2);
        yBegin =    m_tree->bbox.min.y + (yStep / 2);
        xEnd =      m_tree->bbox.max.x + (xStep / 2); // One tick past the end.
        yEnd =      m_tree->bbox.max.y + (yStep / 2);

        results.resize(exp * exp, std::numeric_limits<std::size_t>::max());

        m_tree->getPoints(
                results,
                rasterize,
                xBegin,
                xEnd,
                xStep,
                yBegin,
                yEnd,
                yStep);
    }

    return results;
}

std::vector<std::size_t> QuadIndex::QImpl::getPoints(
        const double xBegin,
        const double xEnd,
        const double xStep,
        const double yBegin,
        const double yEnd,
        const double yStep) const
{
    std::vector<std::size_t> results;

    if (m_tree)
    {
        const std::size_t width (Utils::sround((xEnd - xBegin) / xStep));
        const std::size_t height(Utils::sround((yEnd - yBegin) / yStep));
        results.resize(width * height, std::numeric_limits<std::size_t>::max());

        m_tree->getPoints(
                results,
                xBegin,
                xEnd,
                xStep,
                yBegin,
                yEnd,
                yStep);
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

std::size_t QuadIndex::getDepth() const
{
    return m_qImpl->getDepth();
}

std::vector<std::size_t> QuadIndex::getFills() const
{
    return m_qImpl->getFills();
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
        const std::size_t rasterize,
        double& xBegin,
        double& xEnd,
        double& xStep,
        double& yBegin,
        double& yEnd,
        double& yStep) const
{
    return m_qImpl->getPoints(
            rasterize,
            xBegin,
            xEnd,
            xStep,
            yBegin,
            yEnd,
            yStep);
}

std::vector<std::size_t> QuadIndex::getPoints(
        const double xBegin,
        const double xEnd,
        const double xStep,
        const double yBegin,
        const double yEnd,
        const double yStep) const
{
    return m_qImpl->getPoints(
            xBegin,
            xEnd,
            xStep,
            yBegin,
            yEnd,
            yStep);
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

