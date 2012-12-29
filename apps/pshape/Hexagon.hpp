#ifndef INCLUDED_PSHAPE_HEXAGON_HPP
#define INCLUDED_PSHAPE_HEXAGON_HPP

namespace Pshape
{

class Hexagon
{
public:
    Hexagon(int x, int y) : m_x(x), m_y(y)
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
        std::cerr << "x = " << x << "!\n";
        std::cerr << "ux = " << ux << "!\n";
        std::cerr << "y = " << y << "!\n";
        std::cerr << "uy = " << uy << "!\n";
        std::cerr << "Key = " << (ux | ((uint64_t)uy << 32)) << "!\n";
        return (ux | ((uint64_t)uy << 32));
    }

    int x() const
        { return m_x; }

    int y() const
        { return m_y; }

private:
    int32_t m_x;
    int32_t m_y;
    int m_count;
};

} // namespace

#endif // file guard
