/******************************************************************************
 * Copyright (c) 2014, Hobu Inc. (howard@hobu.co)
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
 *     * Neither the name of the Howard Butler or Hobu, Inc.
 *       the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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


#include <cmath>
#include <algorithm>

#include "HexGrid.hpp"
#include "HexIter.hpp"
#include "Mathpair.hpp"
#include "Processor.hpp"
#include "Segment.hpp"

#include "pdal/util/Utils.hpp"

using namespace std;

namespace hexer
{

HexGrid::HexGrid(int dense_limit) : m_height(-1.0), m_width(-1.0),
    m_pos_roots(HexCompare()), m_dense_limit(dense_limit), m_miny(1)
{}

void HexGrid::initialize(double height)
{
    m_maxSample = 10000;
    m_height = height;
    m_miny = 1;
    m_width = (3 / (2 * SQRT_3)) * m_height;
    m_offsets[0] = Point(0, 0);
    m_offsets[1] = Point(-m_width / 3, m_height / 2);
    m_offsets[2] = Point(0, m_height);
    m_offsets[3] = Point(2 * m_width / 3, m_height);
    m_offsets[4] = Point(m_width, m_height / 2);
    m_offsets[5] = Point(2 * m_width / 3, 0);
    m_center_offset = Point(m_width / 3, m_height / 2);
}

bool HexGrid::dense(Hexagon *h)
{
    return h->count() >= m_dense_limit;
}

void HexGrid::addPoint(Point p)
{
    if (m_width < 0)
    {
        m_sample.push_back(p);
        if (m_sample.size() >= m_maxSample)
            processSample();
        return;
    }

    Hexagon *h = findHexagon(p);
    h->increment();
    if (!h->dense())
    {
        if (dense(h))
        {
            h->setDense();
            m_miny = std::min(m_miny, h->y() - 1);
            if (h->possibleRoot())
                m_pos_roots.insert(h);
            markNeighborBelow(h);
        }
    }
}

void HexGrid::processSample()
{
    if (m_width > 0 || m_sample.empty())
        return;

    double height = computeHexSize(m_sample, m_dense_limit);
    initialize(height);
    for (auto pi = m_sample.begin(); pi != m_sample.end(); ++pi)
        addPoint(*pi);
    m_sample.clear();
}

// A debugging function that can be used to make a particular hexagon
// dense.
void HexGrid::addDenseHexagon(int x, int y)
{
    Hexagon *h = getHexagon(x, y);
    if (!h->dense())
    {
        h->setCount(m_dense_limit);
        h->setDense();
        m_miny = std::min(m_miny, h->y() - 1);
        if (h->possibleRoot())
            m_pos_roots.insert(h);
        markNeighborBelow(h);
    }
}

HexIter HexGrid::hexBegin()
{
    return HexIter(m_hexes.begin(), this);
}

HexIter HexGrid::hexEnd()
{
    return HexIter(m_hexes.end(), this);
}

void HexGrid::markNeighborBelow(Hexagon *h)
{
    Coord c = h->neighborCoord(3);
    Hexagon *neighbor = getHexagon(c);
    neighbor->setDenseNeighbor(0);
    if (neighbor->dense() && !neighbor->possibleRoot())
        m_pos_roots.erase(neighbor);
}

//  first point (origin) and start of column 0
//   |
//   |
//   |  ------- column 0
//   |  |
//   |  |   end of column 0 - start of column 1
//   |  |   |
//   |  v   |                             _____
//   v____  v                            |\    |
//   /    \                              | \   |  Here's an expansion of what
//  / 0,0  \____                         |  \  |  I'm calling a mini-column,
//  \      /    \                        |   \ |  showing two half rows.
//   \____/ 1,0  \                       |____\|
//   /    \      /                       |    /|  The top rectangle is the
//  / 0,1  \____/                        |   / |  negative slope case and
//  \      /    \                        |  /  |  the lower rectangle is the
//   \____/      \                       | /   |  positive slope case.
//                                       |/____|
//        ** <--  The area above these
//                asterisks are the "mini-column"
//                The mini-column is 1/3 the total width of the column.
//
// We are creating a tesselated plane of hexagons where one side of each
// hexagon is parallel with the X axis.  We think of the columns of
// hexagons as all having the same X value.  Hexagons lower than their
// neighbor have successive Y values.
//
// The hexagon in the second column but just below the hexagon in the first
// column as the same Y value as the hexagon above and to the left.  The third
// column's Y values are the one less than the hexagon below and to the left
// as the second column.
//
// The first point, whatever it's X/Y location, is made the origin, and is
// placed at the top-left edge of hexagon 0,0.
//
Hexagon *HexGrid::findHexagon(Point p)
{
    int x, y;

    if (m_hexes.empty())
    {
        m_origin = p;
        // Make a hex at the origin and insert it.  Return a pointer
        // to the hexagon in the map.
        HexMap::value_type hexpair(Hexagon::key(0, 0), Hexagon(0, 0));
        HexMap::iterator it = m_hexes.insert(hexpair).first;
        return &it->second;
    }

    // Offset by the origin.
    p -= m_origin;

    double col = p.m_x / m_width;

    // First calculate X and Y as if we had a bunch of offset rectangles.
    // This works for 2/3 of the width of the hexagons.
    x = (int)floor(col);
    if (x % 2 == 0)
        y = static_cast<int>(floor(p.m_y / m_height));
    else
        y = static_cast<int>(floor((p.m_y - (m_height / 2)) / m_height));

    // Compute the column remainder to determine if we are in a strip where
    // the hexagons overlap (the mini-column).
    double xcolOffset = col - floor(col);
    if (xcolOffset > 2.0/3.0)
    {
        // Calculate the xvalue as a fraction of the width of the column-piece
        // containing multiple hex columns.  These overlap columns are 1/3
        // the total width of any column.

        // Subtract the 2/3 of the value not relevant to the mini-column.
        xcolOffset -= 2.0/3.0;
        // Scale the value to the width of the mini-column.
        xcolOffset *= 3.0;

        // Each halfrow contains a single sloping edge of a hexagon.
        // The slope of the edge is either sqrt(3) or -sqrt(3).  The edge
        // extends from top left to lower right or from bottom left to top
        // right.  What we do here is compute the horizontal fraction of
        // the box (xcolOffset) and the vertical fraction of the box
        // (yrowOffset) and then compare them.
        double halfrow = p.m_y / (m_height / 2);
        int halfy = (int)halfrow;
        double yrowOffset = halfrow - floor(halfrow);

        // Negative slope case.
        if ((halfy % 2 == 0 && x % 2 == 0) || (x % 2 && halfy % 2))
        {
            if (xcolOffset > yrowOffset)
            {
                if (x % 2 == 0)
                    y--;
                x++;
            }
        }
        // Positive slope case.
        else
        {
            if (yrowOffset > xcolOffset)
            {
                if (x % 2)
                    y++;
                x++;
            }
        }
    }
    return getHexagon(x, y);
}

// Get the hexagon at position x, y.  If it doesn't exist, create it.
// Never returns NULL.
Hexagon *HexGrid::getHexagon(int x, int y)
{
    // Stick a hexagon into the map if necessary.
    HexMap::value_type hexpair(Hexagon::key(x, y), Hexagon(x, y));
    std::pair<HexMap::iterator,bool> retval;
    retval = m_hexes.insert(hexpair);
    HexMap::iterator it = retval.first;

    Hexagon *hex_p = &(it->second);

    // Return a pointer to the located hexagon.
    return hex_p;
}

/**
// Walk the outside of the hexagons to make a path.  Hexagon sides are labeled:
//
//     __0_
//  1 /    \ 5
//   /      \
//   \      /
//  2 \____/ 4
//      3
**/
void HexGrid::findShapes()
{
    if (m_pos_roots.empty())
    {
        throw hexer_error("No areas of sufficient density - no shapes. "
            "Decrease density or area size.");
    }

    while (m_pos_roots.size())
    {
        Hexagon *h = *m_pos_roots.begin();
        findShape(h);
    }
}

void HexGrid::findParentPaths()
{
    std::vector<Path *> roots;
    for (size_t i = 0; i < m_paths.size(); ++i)
    {
        Path *p = m_paths[i];
        findParentPath(p);
        // Either add the path to the root list or the parent's list of
        // children.
        !p->parent() ?  roots.push_back(p) : p->parent()->addChild(p);
    }
    for (size_t i = 0; i < roots.size(); ++i)
       roots[i]->finalize(CLOCKWISE);

    // In the end, the list of paths is just the root paths.  Children can
    // be retrieved from their parents.
    m_paths = roots;
}

void HexGrid::findParentPath(Path *p)
{
    Segment s = p->rootSegment();
    Hexagon *h = s.hex();
    int y = h->y();
    while (y >= m_miny)
    {
        HexPathMap::iterator it = m_hex_paths.find(h);
        if (it != m_hex_paths.end())
        {
            Path *parentPath = it->second;
            if (parentPath == p->parent())
            {
               p->setParent(NULL);
            }
            else if (!p->parent() && parentPath != p)
            {
               p->setParent(parentPath);
            }
        }
        h = getHexagon(h->x(), --y);
    }
}

void HexGrid::findShape(Hexagon *hex)
{
    if (!hex)
        throw hexer_error("hexagon was null!");

    Path *p = new Path(this, CLOCKWISE);
    Segment first(hex, 0);
    Segment cur(first);
    do {
        cleanPossibleRoot(cur, p);
        p->push_back(cur);
        Segment next = cur.leftClockwise(this);
        if (!next.hex()->dense())
            next = cur.rightClockwise(this);
        cur = next;
    } while (cur != first);
    m_paths.push_back(p);
}

void HexGrid::cleanPossibleRoot(Segment s, Path *p)
{
    if (s.possibleRoot(this))
        m_pos_roots.erase(s.hex());
    if (s.horizontal())
    {
        s.normalize(this);
        HexPathMap::value_type hexpath(s.hex(), p);
        m_hex_paths.insert(hexpath);
    }
}

void HexGrid::toWKT(std::ostream& out) const
{
    pdal::Utils::StringStreamClassicLocale ss;

    auto writePath = [this, &ss](size_t pathNum)
    {
       rootPaths()[pathNum]->toWKT(ss);
    };

    ss << "MULTIPOLYGON (";

    if (rootPaths().size())
        writePath(0);
    for (size_t pi = 1; pi < rootPaths().size(); ++pi)
    {
        ss << ",";
        writePath(pi);
    }
    ss << ")";
    out << ss.str();
}

size_t HexGrid::densePointCount() const
{
    size_t count = 0;
    for (auto it = m_hexes.begin(); it != m_hexes.end(); ++it)
    {
        const Hexagon& h = it->second;
        if (h.dense())
            count += h.count();
    }
    return count;
}

} //namespace hexer
