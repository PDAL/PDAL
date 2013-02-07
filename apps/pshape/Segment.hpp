#ifndef INCLUDED_PSHAPE_SEGMENT_HPP
#define INCLUDED_PSHAPE_SEGMENT_HPP

#include <iostream>
#include <stdlib.h>
#include <vector>

namespace Pshape
{

class Hexagon;
class HexGrid;

class Segment
{
private:
    /// Hexagon who's side is the segment.
    Hexagon *m_hex;
    /// Which side of the hexagon.
    int m_side;

public:
    Segment() : m_hex(NULL), m_side(0)
        {}

    Segment(Hexagon *h, int side) : m_hex(h), m_side(side)
        {}

    Hexagon *hex()
        { return m_hex; }

    int side()
        { return m_side; }

    Segment rightAntiClockwise(HexGrid *grid);
    Segment leftAntiClockwise(HexGrid *grid);
    Segment rightClockwise(HexGrid *grid);
    Segment leftClockwise(HexGrid *grid);

    friend bool operator == (const Segment& s1, const Segment &s2);
    friend bool operator != (const Segment& s1, const Segment &s2);
    friend std::ostream& operator << (std::ostream& os, const Segment &s);
};

typedef std::vector<Segment> Path;

} // namespace

#endif // file guard
