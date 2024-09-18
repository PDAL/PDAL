#include <hexer/BaseGrid.hpp>
#include <hexer/HexGrid.hpp>

namespace hexer
{

void HexGrid::processHeight(double height)
{
    m_maxSample = 10000;
    m_height = height;
    m_minY = 1;
    m_width = (3 / (2 * SQRT_3)) * m_height;
    m_offsets[0] = Point(0, 0);
    m_offsets[1] = Point(-m_width / 3, m_height / 2);
    m_offsets[2] = Point(0, m_height);
    m_offsets[3] = Point(2 * m_width / 3, m_height);
    m_offsets[4] = Point(m_width, m_height / 2);
    m_offsets[5] = Point(2 * m_width / 3, 0);
    m_centerOffset = Point(m_width / 3, m_height / 2);
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
HexId HexGrid::findHexagon(Point p)
{
    int x, y;

    if (m_counts.empty())
    {
        m_origin = p;
        return HexId{0,0};
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

    // minimum Y (HexId.j) value, used in inGrid() for finding root/child paths in parentOrChild(); 
    // set as y - 1 to account for m_hexPaths containing hexagons across edge 3
    m_minY = std::min(m_minY, y - 1);

    return HexId{x, y};
}

HexId HexGrid::edgeHex(HexId hex, int edge) const
{
    //               (+ Y)
    //                __3_
    //             2 /    \ 4 
    //              /      \
    //              \      /
    //             1 \____/ 5  
    //                  0
    //               (- Y)

    static const HexId even[] = {{0, -1}, {-1, -1}, {-1, 0}, {0, 1}, {1, 0}, {1, -1}};
    static const HexId odd[] = {{0, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}};

    if (hex.i % 2)
        return hex + odd[edge];
    else
        return hex + even[edge];

}

Point HexGrid::findPoint(Segment& s)
{
    HexId hex = s.hex;
    Point pos;

    pos.m_x = hex.i * m_width;
    pos.m_y = hex.j * m_height;
    if (hex.i % 2)
        pos.m_y += (m_height / 2);

    return pos + offset(s.edge) + m_origin;
}

} // namespace hexer
