//ABELL
#include <iostream>

#include "HexGrid.hpp"
#include "Point.hpp"

using namespace std;

namespace Pshape
{

void HexGrid::addPoint(Point p)
{
    Hexagon *h = findHexagon(p);
    cerr << "Point x/y - Hex x/y = " << p.m_x << "/" << p.m_y << " - " <<
        h->x() << "/" << h->y() << "\n";
    h->increment();
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
//   \____/      \                       | /   |  negative slope case.
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
    {
        y = floor(p.m_y / m_height);
    }
    else
    {
        y = floor((p.m_y - (m_height / 2)) / m_height);
    }

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

    // Stick a hexagon into the map if necessary.
    HexMap::value_type hexpair(Hexagon::key(x, y), Hexagon(x, y));
    HexMap::iterator it = m_hexes.insert(hexpair).first;

    // Return a pointer to the located hexagon.
    return &it->second;
}

} //namespace
