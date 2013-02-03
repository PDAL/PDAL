#ifndef INCLUDED_PSHAPE_HEXAGON_HPP
#define INCLUDED_PSHAPE_HEXAGON_HPP

//ABELL - Assertions that int isn't bigger than int32_t
#include <stdint.h>

namespace Pshape
{

class Hexagon
{
public:
    Hexagon(int x, int y) : m_x(x), m_y(y), m_count(0)
        {}

    uint64_t key()
    {
        return key(m_x, m_y);
    }

    void increment()
       { m_count++; }

    void incrementIf(int dense_limit)
    {
        if (m_count < dense_limit)
        {
            m_count++;
        }
    }

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

    bool dense(int dense_limit) const
        { return m_count >= dense_limit; }

   int count() const
       { return m_count; } 

    bool less(Hexagon *h) const
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

private:
    int32_t m_x;
    int32_t m_y;
    int m_count;
};

} // namespace

#endif // file guard
