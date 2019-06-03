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

#include <cassert>

#include "Path.hpp"

#include <cassert>

using namespace std;

namespace hexer
{

Point Path::getPoint(size_t pointnum) const
{
    pointnum = (m_orientation == ANTICLOCKWISE) ?
        m_segs.size() - pointnum - 1 : pointnum;
    return m_segs[pointnum].startPos(m_grid);
}

vector<Point> Path::points() const
{
    vector<Point> points;
    if (m_orientation == CLOCKWISE)
    {
        for (size_t i = 0; i < m_segs.size(); ++i)
            points.push_back(m_segs[i].startPos(m_grid));
        points.push_back(m_segs[0].startPos(m_grid));
    }
    else
    {
        // Note that i will wrap to max of size_t when decrementing 0.
        for (size_t i = m_segs.size() - 1; i < m_segs.size(); --i)
            points.push_back(m_segs[i].startPos(m_grid));
        points.push_back(m_segs[m_segs.size()-1].startPos(m_grid));

    }
    return points;
}

void Path::writeRing(std::ostream& out) const
{
    auto outputPoint = [&out](const Point& p)
    {
        out << p.m_x << " " << p.m_y;
    };

    const vector<Point>& pts = points();
    assert(pts.size() > 2);
    out << "(";
    outputPoint(pts.front());
    for (auto it = pts.begin() + 1; it != pts.end(); ++it)
    {
        out << ", ";
        outputPoint(*it);
    }
    out << ")";
}

// WKT (or GeoJSON) doesn't allow nesting of polygons.  You can just have
// polygons and holes.  Islands within the holes need to be described as
// separate polygons.  To that end, we gather the islands from all holes
// and return them to be processed as separate polygons.
PathPtrList Path::writePolygon(std::ostream& out) const
{
    PathPtrList islands;

    out << "(";
    writeRing(out);
    const PathPtrList& paths = subPaths();
    for (auto& p : paths)
    {
        out << ", ";
        p->writeRing(out);
        const PathPtrList& subs(p->subPaths());
        islands.insert(islands.end(), subs.begin(), subs.end());
    }
    out << ")";
    return islands;
}


void Path::toWKT(std::ostream& out) const
{
    PathPtrList islands = writePolygon(out);

    // See the note on writePolygon()
    while (islands.size())
    {
        PathPtrList paths;
        paths.swap(islands);
        for (Path *p : paths)
        {
            out << ", ";
            PathPtrList subIslands = p->writePolygon(out);
            islands.insert(islands.end(), subIslands.begin(), subIslands.end());
        }
    }
}

} // namespace
