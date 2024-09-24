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
#include <iostream>
#include <algorithm>

#include "Mathpair.hpp"
#include "Segment.hpp"

namespace hexer
{

enum Orientation
{
    CLOCKWISE,     // Outer
    ANTICLOCKWISE  // Hole
};

class Path
{
public:
    Path(HexId root_hex) : m_rootHex(root_hex), m_parent(NULL)
    {}

    void toWKT(std::ostream& output) const;

    void addChild(Path *path)
        { m_children.push_back(path); }
    void setParent(Path *p)
        { m_parent = p; }
    Path *parent()
        { return m_parent; }
    const std::vector<Path*>& subPaths() const
        { return m_children; }
    const std::vector<Point>& points() const
    {
        return m_points;
    }
    HexId rootHex() const
        { return m_rootHex; }
    void addPoint(Point p)
        { m_points.push_back(p); }
    void finalize(Orientation o)
    {
        m_orientation = o;
        for (size_t i = 0; i < m_children.size(); ++i)
            m_children[i]->finalize(o == CLOCKWISE ? ANTICLOCKWISE : CLOCKWISE);
        if (o == ANTICLOCKWISE){
            std::reverse(m_points.begin(), m_points.end());
        }
    }
    int numChildren()
        { return m_children.size(); }
    int numPoints()
        { return m_points.size(); }

    // Test function
    void sortPath()
    {
        std::sort(m_children.begin(), m_children.end(), [](const Path* p1, const Path* p2) 
            { return p1->rootHex() < p2->rootHex(); });
        for (Path* p : m_children)
        {
            p->sortPath();
        }
    }

private:
    void writeRing(std::ostream& out) const;
    std::vector<Path *> writePolygon(std::ostream& out) const;

    /// Hexagon associated with path, used for finding child paths
    HexId m_rootHex;
    /// Parent path (NULL if root)
    Path *m_parent;
    /// Children
    std::vector<Path *> m_children;
    /// Orientation of path AT EXTRACTION - points are ordered clockwise
    /// until finalize() sets orientation.
    Orientation m_orientation;
    /// points that make up the path
    std::vector<Point> m_points;
};

} //namespace hexer
