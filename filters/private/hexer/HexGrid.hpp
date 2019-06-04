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

#include <unordered_map>
#include <set>

#include <pdal/pdal_export.hpp>

#include "exception.hpp"
#include "Hexagon.hpp"
#include "Mathpair.hpp"
#include "Path.hpp"
#include "Segment.hpp"

namespace hexer
{

class HexIter;

static const double SQRT_3 = 1.732050808;

class HexGrid
{
    friend class HexIter;
public:
    PDAL_DLL HexGrid(int dense_limit);
    HexGrid(double height, int dense_limit) : m_dense_limit(dense_limit)
        { initialize(height); }

    ~HexGrid()
    {
        for (std::vector<Path*>::size_type i = 0; i < m_paths.size(); i++)
            delete m_paths[i];
    }

    // Exported for testing.
    PDAL_DLL void findShapes();
    PDAL_DLL void findParentPaths();
    PDAL_DLL void toWKT(std::ostream&) const;
    PDAL_DLL void addDenseHexagon(int x, int y);

    bool dense(Hexagon *h);
    void addPoint(double x, double y)
        { addPoint(Point(x, y)); }
    void addPoint(Point p);
    void processSample();

    void extractShapes();
    void dumpInfo();
    void drawHexagons();
    Hexagon *getHexagon(int x, int y);
    Hexagon *getHexagon(const Coord& c)
        { return getHexagon(c.m_x, c.m_y); }
    HexIter hexBegin();
    HexIter hexEnd();
    double width() const
        { return m_width; }
    double height() const
        { return m_height; }
    Point const& offset(int idx) const
        { return m_offsets[idx]; }
    Point centerOffset(int idx) const
        { return (m_offsets[idx] - m_center_offset); }
    Point const& origin() const
        { return m_origin; }
    int denseLimit() const
        { return m_dense_limit; }
    std::vector<Path *> const& rootPaths() const
        { return m_paths; }
    void setSampleSize(unsigned sampleSize)
        { m_maxSample = sampleSize; }
    size_t densePointCount() const;

private:
    void initialize(double height);
    Hexagon *findHexagon(Point p);
    void findShape(Hexagon *hex);
    void findHole(Hexagon *hex);
    void cleanPossibleRoot(Segment s, Path *p);
    void findParentPath(Path *p);
    void markNeighborBelow(Hexagon *hex);

    /// Height of the hexagons in the grid (2x apothem)
    double m_height;
    /// Width of the hexagons in the grid
    double m_width;
    /// Origin of the hex grid in point coordinates.
    Point m_origin;
    /// Offsets of vertices of hexagon, going anti-clockwise from upper-left
    Point m_offsets[6];
    /// Offset of the center of the hexagons.
    Point m_center_offset;
    /// Map of hexagons based on keys.
    typedef std::unordered_map<uint64_t, Hexagon> HexMap;
    HexMap m_hexes;
    /// Set of dense hexagons with non-dense neighbors above.
    typedef std::set<Hexagon *, HexCompare> HexSet;
    HexSet m_pos_roots;
    /// Map of roots and their associated paths.
    typedef std::unordered_map<Hexagon *, Path *> HexPathMap;
    HexPathMap m_hex_paths;
    /// List of paths
    std::vector<Path *> m_paths;
    /// Number of points that must lie in a hexagon for it to be interesting.
    int m_dense_limit;
    /// Minimum y - 1.
    int m_miny;
    /// Vector of points to use to determine hex height.
    std::vector<Point> m_sample;
    /// Maximum sample size.
    unsigned m_maxSample;
};

} // namespace

