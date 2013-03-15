#ifndef INCLUDED_PSHAPE_GRID_INFO_HPP
#define INCLUDED_PSHAPE_GRID_INFO_HPP

#include <vector>

#include "HexGrid.hpp"

namespace Pshape
{

class HexInfo
{
public:
    Point m_center;
    int m_density;
};

class HexIter
{
public:
    HexIter(HexGrid::HexMap::iterator iter, HexGrid *grid) :
        m_iter(iter), m_grid(grid)
    {
        advance();
    }

    HexIter& operator++()
    {
        m_iter++;
        advance();
        return *this;
    }

    HexInfo operator*()
    {
        HexInfo info;
        Hexagon& hex = m_iter->second;
        Point p;
        p.m_x = hex.x() * m_grid->width();
        p.m_y = hex.y() * m_grid->height();
        if (hex.xodd())
        {
            p.m_y += (m_grid->height() / 2);
        }
        info.m_center = p + m_grid->centerOffset(0);
        info.m_density = hex.count();
        return info;
    }

    bool operator == (const HexIter& iter)
        { return m_iter == iter.m_iter; }
    bool operator != (const HexIter& iter)
        { return m_iter != iter.m_iter; }

private:
    void advance()
    {
        while (m_iter != m_grid->m_hexes.end())
        {
            if (m_iter->second.count())
            {
                break;
            }
            m_iter++;
        }
    }

    HexGrid::HexMap::iterator m_iter;
    HexGrid *m_grid;
};

class GridInfo
{
public:
    GridInfo() : m_hexsize(-1), m_density(10)
        {}
    ~GridInfo()
        { delete m_grid; }

    double m_hexsize;
    int m_density;

    std::vector<Path *> rootPaths()
        { return m_grid->rootPaths(); }


    Point offset(int idx)
        { return m_grid->centerOffset(idx); }

    double width()
        { return m_grid->width(); }

    double height()
        { return m_grid->height(); }

    HexIter begin()
        { return HexIter(m_grid->m_hexes.begin(), m_grid); }

    HexIter end()
        { return HexIter(m_grid->m_hexes.end(), m_grid); }

    HexGrid *m_grid;
};

} // namespace

#endif

