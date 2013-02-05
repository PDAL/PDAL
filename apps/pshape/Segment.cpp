#include "Segment.hpp"

#include "HexGrid.hpp"

namespace Pshape
{

// When we're traversing a hexagon counter-clockwise, deterimine
// the next segment we'll traverse assume we're taking a right turn.
Segment Segment::rightAntiClockwise(HexGrid *grid)
{
    Segment next;
    static int nextside[] = { 5, 0, 1, 2, 3, 4 };

    next.m_side = nextside[m_side];
    Coord coord = m_hex->neighborCoord(m_side);
    next.m_hex = grid->getHexagon(coord.m_x, coord.m_y);
    return next;
}

// When we're traversing a hexagon counter-clockwise, determine the
// next segment we'll traverse assuming we're taking a right turn.
Segment Segment::leftAntiClockwise(HexGrid *grid)
{
    (void)grid; //unused
    Segment next(*this);
    next.m_side = (next.m_side + 1) % 6;
    return next;
}

bool operator == (const Segment& s1, const Segment &s2)
{
    static int sharedside[] = { 3, 4, 5, 0, 1, 2 };
    static int evenx[] = { 0, -1, -1, 0, 1, 1 };
    static int eveny[] = { -1, -1, 0, 1, 0, -1 };
    static int oddx[] = { 0, -1, -1, 0, 1, 1, };
    static int oddy[] = { -1, 0, 1, 1, 1, 0 };
    if (s1.m_hex == s2.m_hex && s1.m_side == s2.m_side)
    {
        return true;
    }
    if (s2.m_side == sharedside[s1.m_side])
    {
        int xinc;
        int yinc;
        if (s1.m_hex->xeven())
        {
            xinc = evenx[s1.m_side];
            yinc = eveny[s1.m_side];
        }
        else
        {
            xinc = oddx[s1.m_side];
            yinc = oddy[s1.m_side];
        }
        if ((s2.m_hex->x() == s1.m_hex->x() + xinc) &&
            (s2.m_hex->y() == s1.m_hex->y() + yinc))
        {
            return true;
        }
    }
    return false;
}

bool operator != (const Segment& s1, const Segment &s2)
{
    return !(s1 == s2);
}

} // namespace
