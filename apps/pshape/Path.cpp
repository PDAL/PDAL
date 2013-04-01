#include "Path.hpp"

using namespace std;

namespace Pshape
{

Point Path::getPoint(int pointnum)
{
    pointnum = (m_orientation == ANTICLOCKWISE) ?
        m_segs.size() - pointnum - 1 : pointnum;
    return m_segs[pointnum].startPos(m_grid);
}

vector<Point> Path::points()
{
    vector<Point> points;
    if (m_orientation == CLOCKWISE)
    {
        for (size_t i = 0; i < m_segs.size(); ++i)
        {
            points.push_back(m_segs[i].startPos(m_grid));
        }
    }
    else
    {
        // Note that i will wrap to max of size_t when decrementing 0.
        for (size_t i = m_segs.size() - 1; i < m_segs.size(); --i)
        {
            points.push_back(m_segs[i].startPos(m_grid));
        }
    }
    return points;
}

} // namespace
