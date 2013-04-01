#ifndef INCLUDED_PSHAPE_POINT_HPP
#define INCLUDED_PSHAPE_POINT_HPP

namespace Pshape
{

template <typename T>
struct Mathpair
{
public:
    Mathpair() : m_x(0.0), m_y(0.0)
    {}

    Mathpair(T x, T y) : m_x(x), m_y(y)
    {}

    T m_x;
    T m_y;

    void operator -= (const Mathpair& p)
    {
       m_x -= p.m_x;
       m_y -= p.m_y;
    }

    Mathpair& operator += (const Mathpair& p)
    {
        m_x += p.m_x;
        m_y += p.m_y;
        return *this;
    }

    friend Mathpair operator - (Mathpair p1, const Mathpair& p2)
    {
        p1 -= p2;
        return p1;
    }

    friend Mathpair operator + (Mathpair p1, const Mathpair& p2)
    {
        p1 += p2;
        return p1;
    }
};

typedef Mathpair<double> Point;
typedef Mathpair<int> Coord;

} // namespace

#endif // file guard
