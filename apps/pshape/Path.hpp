#ifndef INCLUDED_PSHAPE_PATH_HPP
#define INCLUDED_PSHAPE_PATH_HPP

#include <vector>

#include "Mathpair.hpp"
#include "Segment.hpp"

namespace Pshape
{

enum Orientation
{
    CLOCKWISE,     // Outer
    ANTICLOCKWISE  // Hole
};

class HexGrid;

class Path
{
public:
    Path(HexGrid *m_grid, Orientation orient) :
        m_parent(NULL), m_orientation(orient)
    {}

    ~Path()
    {
        for (int i = 0; i < m_children.size(); ++i)
            delete m_children[i];
    }

    void push_back(const Segment& s)
        { m_segs.push_back(s); }
    Segment rootSegment()
        { return m_segs[0]; }
    Path *parent()
        { return m_parent; }
    void setParent(Path *p)
        { m_parent = p; }
    void addChild(Path *p)
        { m_children.push_back(p); }
    void finalize(Orientation o)
    {
        m_orientation = o;
        for (size_t i = 0; i < m_children.size(); ++i)
            m_children[i]->finalize(o == CLOCKWISE ? ANTICLOCKWISE : CLOCKWISE);
    }
    int pathLength()
        { return m_segs.size(); }
    Point getPoint(int pointnum);
    Orientation orientation() const
        { return m_orientation; }
    std::vector<Point> points();
    std::vector<Path *> subPaths()
        { return m_children; }

private:
    /// Grid that owns the path.
    HexGrid *m_grid;
    /// Parent path (NULL if root path)
    Path *m_parent;
    /// Children
    std::vector<Path *> m_children;
    /// Orientation of path AT EXTRACTION - segments are ALWAYS ordered
    /// clockwise.
    Orientation m_orientation;
    /// List of segments that make up the path.
    std::vector<Segment> m_segs;
};

} //namespace

#endif
