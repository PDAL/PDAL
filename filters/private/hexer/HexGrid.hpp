#pragma once

#if WINDOWS
#undef min
#undef max
#endif // WINDOWS

#include "BaseGrid.hpp"

namespace hexer
{

class PDAL_DLL HexGrid : public BaseGrid
{
public:
    HexGrid(double height, int denseLimit) : BaseGrid(denseLimit)
        { processHeight(height); }
    HexGrid(int denseLimit) : BaseGrid(denseLimit), m_height(-1.0),
        m_width(-1.0), m_minY(std::numeric_limits<int>::max())
    {}
    ~HexGrid();


    void addXY(double& x, double& y)
        { 
          Point p{x, y};
          addPoint(p);
        }

    Point offset(int idx) const
        { return m_offsets[idx]; }
    double height()
        { return m_height; }
    bool sampling() const
        { return m_width < 0; }
    bool isH3()
        { return false; }
    Point findPoint(Segment& s);

private:
    void processHeight(double height);
    HexId findHexagon(Point p);
    HexId edgeHex(HexId hex, int edge) const;

    bool inGrid(HexId& h)
        { return h.j >= m_minY; }
    HexId moveCoord(HexId& h)
        { return HexId{h.i, h.j - 1}; }

    // minimum Y (HexId.j) value, used in inGrid() for finding root/child paths in parentOrChild()
    void setMinCoord(HexId& h)
        { m_minY = std::min(m_minY, h.j); }

    /// Height of the hexagons in the grid (2x apothem)
    double m_height;
    /// Width of the hexagons in the grid
    double m_width;
    /// Minimum y - 1.
    int m_minY;
    /// Offsets of vertices of hexagon, going anti-clockwise from upper-left
    Point m_offsets[6];
    /// Offset of the center of the hexagons.
    Point m_centerOffset;
    /// Origin of the hex grid in point coordinates
    Point m_origin;

};

} // namespace hexer
