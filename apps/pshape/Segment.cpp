#include "Segment.hpp"

#include "HexGrid.hpp"

namespace Pshape
{

// The segment is normalized if necessary.
bool Segment::possibleRoot(HexGrid *grid)
{
    if (m_side == 3)
    {
        m_side = 0;
        m_hex = grid->getHexagon(m_hex->x(), m_hex->y() + 1);
    }
    return m_hex->possibleRoot() && (m_side == 0);
}

// When we're traversing a hexagon counter-clockwise, determine
// the next segment we'll traverse assume we're taking a right turn.
Segment Segment::rightAntiClockwise(HexGrid *grid)
{
    Segment next;
    static int nextside[] = { 5, 0, 1, 2, 3, 4 };
    static int neighborside[] = { 1, 2, 3, 4, 5, 0 };

    Coord coord = m_hex->neighborCoord(neighborside[m_side]);
    next.m_side = nextside[m_side];
    next.m_hex = grid->getHexagon(coord.m_x, coord.m_y);
    return next;
}

// When we're traversing a hexagon counter-clockwise, determine the
// next segment we'll traverse assuming we're taking a right turn.
Segment Segment::leftAntiClockwise(HexGrid *grid)
{
    static int nextside[] = { 1, 2, 3, 4, 5, 0 };

    (void)grid; //unused
    Segment next(*this);
    next.m_side = nextside[next.m_side];
    return next;
}

// When we're travsersing a hexagon clockwise, determine the
// next segment we'll traverse assume we're taking a right turn.
Segment Segment::rightClockwise(HexGrid *grid)
{
    static int nextside[] = { 5, 0, 1, 2, 3, 4 };

    (void)grid; //unused
    Segment next(*this);
    next.m_side = nextside[next.m_side];
    return next;
}

// When we're traversing a hexagon clockwise, determine the next segment
// we'll traverse assume we're taking a left turn.
Segment Segment::leftClockwise(HexGrid *grid)
{
    Segment next;
    static int nextside[] = { 1, 2, 3, 4, 5, 0 };
    static int neighborside[] = { 5, 0, 1, 2, 3, 4 };

    Coord coord = m_hex->neighborCoord(neighborside[m_side]);
    next.m_side = nextside[m_side];
    next.m_hex = grid->getHexagon(coord.m_x, coord.m_y);
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

std::ostream& operator << (std::ostream& os, const Segment &s)
{
    os << s.m_hex->x() << "/" << s.m_hex->y() << " - " << s.m_side;
    return os;
};


} // namespace
