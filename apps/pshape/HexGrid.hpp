#ifndef INCLUDED_PSHAPE_HEX_GRID_HPP
#define INCLUDED_PSHAPE_HEX_GRID_HPP

//ABELL
#include <iostream>

#include <boost/unordered_map.hpp>

#include "Hexagon.hpp"
#include "Point.hpp"

namespace Pshape
{

static const double SQRT_3 = 1.732050808; 

class HexGrid
{
public:
    HexGrid(double height) : m_height(height)
        { m_width = (3 / (2 * SQRT_3)) * m_height;
std::cerr << "Height/Width = " << m_height << "/" << m_width << "!\n";
}

    void addPoint(Point p);
    void findShapes();
    void extractShapes();

private:
    Hexagon *findHexagon(Point p);

    // Height of the hexagons in the grid (2x apothem)
    double m_height;
    // Width of the hexagons in the grid
    double m_width;
    // Origin of the hex grid in point coordinates.
    Point m_origin;
    // Map of hexagons based on keys.
    typedef boost::unordered_map<uint64_t, Hexagon> HexMap;
    HexMap m_hexes;
};

} // namespace

#endif // file guard
