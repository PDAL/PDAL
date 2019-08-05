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
#pragma once

#include <vector>
#include <ostream>

#include "Mathpair.hpp"
#include "Segment.hpp"

namespace hexer
{

enum Orientation
{
    CLOCKWISE,     // Outer
    ANTICLOCKWISE  // Hole
};

class HexGrid;
class Path;
using PathPtrList = std::vector<Path *>;

class Path
{
public:
    Path(HexGrid *m_grid, Orientation orient) :
        m_grid(m_grid), m_parent(NULL), m_orientation(orient)
    {}

    ~Path()
    {
        for (auto p : m_children)
            delete p;
    }

    void push_back(const Segment& s)
        { m_segs.push_back(s); }
    Segment rootSegment()
        { return m_segs[0]; }
    Path *parent()
        { return m_parent; }
    void setParent(Path *p)
        { m_parent = p; }
    void addChild(Path *p)
        { m_children.push_back(p); }
    void finalize(Orientation o)
    {
        m_orientation = o;
        for (size_t i = 0; i < m_children.size(); ++i)
            m_children[i]->finalize(o == CLOCKWISE ? ANTICLOCKWISE : CLOCKWISE);
    }
    size_t pathLength() const
        { return m_segs.size(); }
    Point getPoint(size_t pointnum) const;
    Orientation orientation() const
        { return m_orientation; }
    std::vector<Point> points() const;
    PathPtrList subPaths() const
        { return m_children; }
    void toWKT( std::ostream& output) const;

private:
    /// Grid that owns the path.
    HexGrid *m_grid;
    /// Parent path (NULL if root path)
    Path *m_parent;
    /// Children
    PathPtrList m_children;
    /// Orientation of path AT EXTRACTION - segments are ALWAYS ordered
    /// clockwise.
    Orientation m_orientation;
    /// List of segments that make up the path.
    std::vector<Segment> m_segs;

    void writeRing(std::ostream& out) const;
    PathPtrList writePolygon(std::ostream& out) const;
};

} //namespace hexer

