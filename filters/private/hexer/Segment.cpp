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


#include "Segment.hpp"
#include "HexGrid.hpp"

namespace hexer
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

// Change the edge so that it has side 0-2 if necessary.
void Segment::normalize(HexGrid *grid)
{
    if (m_side >= 3)
    {
        Coord coord = m_hex->neighborCoord(m_side);
        m_side -= 3;
        m_hex = grid->getHexagon(coord.m_x, coord.m_y);
    }
}

Point Segment::startPos(HexGrid *grid) const
{
    int side = m_side - 1;
    side = side < 0 ? 5 : side;
    return pos(grid, grid->offset(side));
}

Point Segment::endPos(HexGrid *grid) const
{
    return pos(grid, grid->offset(m_side));
}

Point Segment::pos(HexGrid *grid, const Point& offset) const
{
    Point pos;
    pos.m_x = m_hex->x() * grid->width();
    pos.m_y = m_hex->y() * grid->height();
    if (m_hex->xodd())
    {
        pos.m_y += (grid->height() / 2);
    }
    return pos + offset + grid->origin();
}

bool operator == (const Segment& s1, const Segment &s2)
{
    static int sharedside[] = { 3, 4, 5, 0, 1, 2 };
    static int evenx[] = { 0, -1, -1, 0, 1, 1 };
    static int eveny[] = { -1, -1, 0, 1, 0, -1 };
    static int oddx[] = { 0, -1, -1, 0, 1, 1, };
    static int oddy[] = { -1, 0, 1, 1, 1, 0 };
    if (s1.m_hex == s2.m_hex && s1.m_side == s2.m_side)
        return true;
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

} // namespace hexer

