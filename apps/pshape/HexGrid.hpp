#ifndef INCLUDED_PSHAPE_HEX_GRID_HPP
#define INCLUDED_PSHAPE_HEX_GRID_HPP

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include "Hexagon.hpp"
#include "Mathpair.hpp"
#include "Draw.hpp"

namespace Pshape
{

static const double SQRT_3 = 1.732050808; 

enum Orientation
{
    CLOCKWISE,
    ANTICLOCKWISE
};

class HexGrid
{
public:
    HexGrid(double height, int dense_limit) :
        m_height(height), m_dense_limit(dense_limit), m_draw(this), m_min(NULL),
        m_miny(1)
    {
        m_width = (3 / (2 * SQRT_3)) * m_height;
        m_offsets[0] = Point(0, 0);
        m_offsets[1] = Point(-m_width / 3, m_height / 2);
        m_offsets[2] = Point(0, m_height);
        m_offsets[3] = Point(2 * m_width / 3, m_height);
        m_offsets[4] = Point(m_width, m_height / 2);
        m_offsets[5] = Point(2 * m_width / 3, 0);
    }

    bool dense(Hexagon *h);
    void addPoint(Point p);
    void findShapes();
    void extractShapes();
    void dumpInfo();
    void drawHexagons();
    Hexagon *getHexagon(int x, int y);
    Hexagon *getHexagon(const Coord& c)
        { return getHexagon(c.m_x, c.m_y); }

    double width()
        { return m_width; }
    double height()
        { return m_height; }
    Point offset(int idx)
        { return m_offsets[idx]; }
    Point origin()
        { return m_origin; }
    int denseLimit()
        { return m_dense_limit; }

private:
    Hexagon *findHexagon(Point p);
    void findShape(Hexagon *hex);
    void findHole(Hexagon *hex);
    Orientation calcOrientation(Hexagon *hex);
    void cleanPossibleRoot(Segment s);

    /// Height of the hexagons in the grid (2x apothem)
    double m_height;
    /// Width of the hexagons in the grid
    double m_width;
    /// Origin of the hex grid in point coordinates.
    Point m_origin;
    /// Offsets of vertices of hexagon, going anti-clockwise from upper-left
    Point m_offsets[6];
    /// Map of hexagons based on keys.
    typedef boost::unordered_map<uint64_t, Hexagon> HexMap;
    HexMap m_hexes;
    /// Set of root hexagons for positive paths.
    typedef boost::unordered_set<Hexagon *> HexSet;
    HexSet m_pos_roots;
    /// Number of points that must like in a hexagon for it to be interesting.
    int m_dense_limit;
    /// Drawing interface.
    Draw m_draw;
    /// Minimum hex.
    Hexagon *m_min;
    /// Minimum y - 1.
    int m_miny;
};

} // namespace

#endif // file guard
