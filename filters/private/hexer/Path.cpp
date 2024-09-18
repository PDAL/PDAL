#include <hexer/Path.hpp>

namespace hexer 
{

void Path::toWKT(std::ostream& output) const
{
    std::vector<Point> pts = m_points;

    auto outputPoint = [&output](Point& p)
    {
        output << p.m_x << " " << p.m_y;
    };

    output << "(";

    auto pi = pts.begin();
    if (pi != pts.end())
        outputPoint(*pi++);
    for (; pi != pts.end(); ++pi)
    {
        output << ", ";
        outputPoint(*pi);
    }

    output << ")";

    std::vector<Path *> paths = subPaths();
    for (auto p : paths)
    {
        output <<",";
        p->toWKT(output);
    }
}
} // namespace hexer