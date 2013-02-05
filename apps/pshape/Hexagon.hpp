#ifndef INCLUDED_PSHAPE_HEXAGON_HPP
#define INCLUDED_PSHAPE_HEXAGON_HPP

//ABELL - Assertions that int isn't bigger than int32_t
#include <stdint.h>

#include "Mathpair.hpp"

namespace Pshape
{

class Hexagon
{
public:
    Hexagon(int x, int y) : m_x(x), m_y(y), m_count(0), m_dense(false),
        m_dense_neighbors(0)
        {}

    uint64_t key()
    {
        return key(m_x, m_y);
    }

    void increment()
       { m_count++; }

    static uint64_t key(int32_t x, int32_t y)
    {
        uint32_t ux = (uint32_t)x;
        uint32_t uy = (uint32_t)y;
        return (ux | ((uint64_t)uy << 32));
    }

    int x() const
        { return m_x; }

    int y() const
        { return m_y; }

    bool xodd() const
        { return (x() % 2 != 0); }

    bool xeven() const
        { return !xodd(); }

    void setDense()
        { m_dense = true; }

    bool dense() const
        { return m_dense; }

    int count() const
        { return m_count; } 

    void setDenseNeighbor(int dir)
        { m_dense_neighbors |= (1 << dir); }

    // We're surrounded by dense neighbors when the six low bits are set.
    bool surrounded() const
        { return (m_dense && (m_dense_neighbors == 0x3F)); }

    bool less(Hexagon *h) const;
    Coord neighborCoord(int dir) const;

private:
    int32_t m_x;
    int32_t m_y;
    int m_count;
    bool m_dense;
    int m_dense_neighbors;
};

} // namespace

#endif // file guard
