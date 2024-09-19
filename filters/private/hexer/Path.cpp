#include <cassert>

#include "Path.hpp"

namespace hexer 
{

void Path::writeRing(std::ostream& out) const
{
    auto outputPoint = [&out](const Point& p)
    {
        out << p.m_x << " " << p.m_y;
    };

    const std::vector<Point>& pts = points();
    assert(pts.size() > 2);
    out << "(";
    outputPoint(pts.front());
    for (auto it = pts.begin() + 1; it != pts.end(); ++it)
    {
        out << ", ";
        outputPoint(*it);
    }
    out << ")";
}

// WKT (or GeoJSON) doesn't allow nesting of polygons.  You can just have
// polygons and holes.  Islands within the holes need to be described as
// separate polygons.  To that end, we gather the islands from all holes
// and return them to be processed as separate polygons.
std::vector<Path *> Path::writePolygon(std::ostream& out) const
{
    std::vector<Path *> islands;

    out << "(";
    writeRing(out);
    const std::vector<Path *>& paths = subPaths();
    for (auto& p : paths)
    {
        out << ", ";
        p->writeRing(out);
        const std::vector<Path *>& subs(p->subPaths());
        islands.insert(islands.end(), subs.begin(), subs.end());
    }
    out << ")";

    return islands;
}

void Path::toWKT(std::ostream& output) const
{
    std::vector<Path *> islands = writePolygon(output);

    // See the note on writePolygon()
    while (islands.size())
    {
        std::vector<Path *> paths;
        paths.swap(islands);
        for (Path *p : paths)
        {
            output << ", ";
            std::vector<Path *> subIslands = p->writePolygon(output);
            islands.insert(islands.end(), subIslands.begin(), subIslands.end());
        }
    }
}
} // namespace hexer