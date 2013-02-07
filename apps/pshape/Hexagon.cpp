#include "Hexagon.hpp"

namespace Pshape
{

//
//     __0_
//  1 /    \ 5
//   /      \
//   \      /
//  2 \____/ 4
//      3
//

bool Hexagon::less(Hexagon *h) const
{
    if (y() < h->y())
    {
        return true;
    }
    if (y() > h->y())
    {
        return false;
    }
    if (xeven() && h->xodd())
    {
        return true;
    }
    if (xodd() && h->xeven())
    {
        return false;
    }
    return x() < h->x();
}

// Find the x and Y in hex coordinates of the hexagon next to this hexagon
// in the direction specified.
Coord Hexagon::neighborCoord(int dir) const
{
    static int evenx[] = { 0, -1, -1, 0, 1, 1 };
    static int eveny[] = { -1, -1, 0, 1, 0, -1 };
    static int oddx[] = { 0, -1, -1, 0, 1, 1 };
    static int oddy[] = { -1, 0, 1, 1, 1, 0 };

    Coord coord(m_x, m_y);
    if (xeven())
    {
        coord.m_x += evenx[dir];  
        coord.m_y += eveny[dir];
    }
    else
    {
        coord.m_x += oddx[dir];  
        coord.m_y += oddy[dir];
    }
    return coord;
}

} // namespace
