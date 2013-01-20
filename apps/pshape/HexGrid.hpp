#ifndef INCLUDED_PSHAPE_HEX_GRID_HPP
#define INCLUDED_PSHAPE_HEX_GRID_HPP

#include <boost/unordered_map.hpp>

#include "Hexagon.hpp"
#include "Point.hpp"

namespace Pshape
{

static const double SQRT_3 = 1.732050808; 

class HexGrid
{
public:
    HexGrid(double height, int dense_limit) :
        m_height(height), m_min( NULL ), m_dense_limit(dense_limit)
    { m_width = (3 / (2 * SQRT_3)) * m_height; }

    void addPoint(Point p);
    void findShapes();
    void extractShapes();
    Hexagon *getHexagon(int x, int y);

private:
    Hexagon *findHexagon(Point p);

    /// Height of the hexagons in the grid (2x apothem)
    double m_height;
    /// Width of the hexagons in the grid
    double m_width;
    /// Origin of the hex grid in point coordinates.
    Point m_origin;
    /// Map of hexagons based on keys.
    typedef boost::unordered_map<uint64_t, Hexagon> HexMap;
    HexMap m_hexes;
    /// Minimum hexagon (see Hexagon::less())
    Hexagon *m_min;
    /// Number of points that must like in a hexagon for it to be interesting.
    int m_dense_limit;
};

} // namespace

#endif // file guard
