#include <set>
#include <iostream>
#include <cmath>

#include <h3api.h>

#include "BaseGrid.hpp"
#include "Path.hpp"

namespace hexer
{

BaseGrid::~BaseGrid()
{}

void BaseGrid::addPoint(Point& p)
{
    if (sampling())
    {
        handleSamplePoint(p);
        return;
    }
    // find the hexagon that the point is contained within
    HexId h = findHexagon(p);

    // add the hexagon to the grid, and increment its count if it exists.
    int count = increment(h);

    // if the hexagon of interest has reached the density threshold, we see if it
    // has neighbors at edge 0. If not, it's added to our list of possible starting points
    // for path finding (m_possibleRoots). If the hexagon at edge 3 was in m_possibleRoots we 
    // remove it since it no longer has a non-dense neighbor at edge 0.
    if (count == m_denseLimit)
    {
        HexId above = edgeHex(h, 0);
        HexId below = edgeHex(h, 3);
        if (!isDense(above))
            addRoot(h);
        removeRoot(below);
    }
}

void BaseGrid::addHexagon(HexId& hex)
{
    int count = increment(hex);

    if (count == m_denseLimit)
    {
        HexId above = edgeHex(hex, 0);
        HexId below = edgeHex(hex, 3);

        if (!isDense(above))
            addRoot(hex);
        removeRoot(below);
    }
}

void BaseGrid::handleSamplePoint(Point& p)
{
    m_sample.push_back(p);
    if (m_sample.size() >= (size_t)m_maxSample) {
        double height = computeHexSize();
        processHeight(height);
        for (Point p : m_sample) {
            addPoint(p);
        }
        m_sample.clear();
    }
}

void BaseGrid::addRoot(HexId h)
{
    m_possibleRoots.insert(h);
}

void BaseGrid::removeRoot(HexId h)
{
    m_possibleRoots.erase(h);
}

int BaseGrid::increment(HexId h)
{
    int& i = m_counts[h];
    i++;
    return i;
}

bool BaseGrid::isDense(HexId h)
{
    return m_counts[h] >= m_denseLimit;
}

// Find the full boundary around our dense hexagons
void BaseGrid::findShapes()
{
    if (m_possibleRoots.empty())
        throw hexer_error("No areas of sufficient density - no shapes. "
            "Decrease density or area size.");

    while (m_possibleRoots.size())
        findShape(*m_possibleRoots.begin());
}

// Walk one contiguous segment of the boundary
void BaseGrid::findShape(HexId root)
{
    m_paths.push_back(Path(root));
    Path& path = m_paths.back();

    const Segment first(root, 0);
    Segment cur(root, 0);

    do {
        // removes possible roots that are passed over, and sets information
        // to be used in parentOrChild()
        if (cur.horizontal())
        {
            // all hexagons with non-dense neighbors at edge 0 are possible roots.
            if (cur.edge == 0)
                m_possibleRoots.erase(cur.hex);

            // if path is at edge 3, normalize to edge 0 of hex across edge 3
            // so hexagons can be processed separately in parentOrChild()
            HexId pathHex = (cur.edge == 0 ? cur.hex : edgeHex(cur.hex, 3));
            m_hexPaths.insert({pathHex, &path});

            // here we set the minimum I or J coordinate, to be used for finding
            // parents and children from m_hexPaths
            setMinCoord(pathHex);
        }
        // adds the first point of the segment to the path
        path.addPoint(findPoint(cur));
        const auto& [left, right] = nextSegments(cur);
        // left.hex: the hexagon we would "walk into" moving clockwise from the
        // current segment.
        cur = isDense(left.hex) ? left : right;
    } while (cur != first);

    path.addPoint(findPoint(cur));
}

// Finds the possibilities for the next boundary segment, moving clockwise
std::pair<Segment, Segment> BaseGrid::nextSegments(const Segment& s) const
{
    //             (example with HexGrid coordinates)
    //    ____
    //   /    \  <---- Current segment: edge 4 of (0,0)
    //  / 0,0  \__v----------- Possible next segment, left: edge 3 of (1,-1)
    //  \      /<---\----- Possible next segment, right: edge 5 of (0,0)
    //   \____/ 1,-1 \
    //   /    \      /
    //  / 0,-1 \____/
    //  \      /    \
    //   \____/      \
    //
    static const int next[] { 1, 2, 3, 4, 5, 0 };
    static const int prev[] { 5, 0, 1, 2, 3, 4 };

    Segment right(s.hex, next[s.edge]);
    Segment left(edgeHex(s.hex, right.edge), prev[s.edge]);
    return { left, right };
}

// Determine whether a path is enclosed within another
void BaseGrid::findParentPaths()
{
    for (auto& p : m_paths) {
        // the only real difference between parentOrChild in the two grid 
        // types is whether they look down i or j.
        parentOrChild(p);

        !p.parent() ?  m_roots.push_back(&p) : p.parent()->addChild(&p);
    }
    for (size_t i = 0; i < m_roots.size(); ++i) {
        m_roots[i]->finalize(CLOCKWISE);
    }
}

// Finds whether a path should be a root path or child path
void BaseGrid::parentOrChild(Path& p)
{
    // get the possible root hexagon that was used to initialize the path
    HexId hex = p.rootHex();
    // get the i or j component of the hexagon, depending on which indexing
    // system is being moved through (-I for H3Grid, -J for HexGrid)
    while (inGrid(hex))
    {
        // see if the current hexagon has a path at edge 0 or 3.
        auto it = m_hexPaths.find(hex);
        if (it != m_hexPaths.end())
        {
            // get the path associated with the current hexagon
            Path *parentPath = it->second;
            // if we pass through another path an even number of times,
            // it can't be our path's parent.
            // a path's parent is always null to start
            if (parentPath == p.parent())
                p.setParent(NULL);
            // if a unique path is passed through an odd # of times, it's
            // set as our path's parent if our doesn't have one already.
            else if (!p.parent() && parentPath != &p)
                p.setParent(parentPath);
        }
        // move to the next hexagon
        hex = moveCoord(hex);
    }
}

double BaseGrid::distance(const Point& p1, const Point& p2)
{
    double xdist = p2.m_x - p1.m_x;
    double ydist = p2.m_y - p1.m_y;
    return std::sqrt(xdist * xdist + ydist * ydist);
}

// Compute hex size based on distance between consecutive points and
// density
double BaseGrid::computeHexSize()
{
    double dist = 0;
    for (std::vector<Point>::size_type i = 0; i < m_sample.size() - 1; ++i)
    {
        Point p1 = m_sample[i];
        Point p2 = m_sample[i + 1];
        dist += distance(p1, p2);
    }
    return ((m_denseLimit * dist) / m_sample.size());
}

void BaseGrid::sortPaths()
{
    std::sort(m_roots.begin(), m_roots.end(), [](const Path* p1, const Path* p2) 
        { return p1->rootHex() < p2->rootHex(); });
    for (Path* p : m_roots)
    {
        p->sortPath();
    }
}

void BaseGrid::toWKT(std::ostream& output) const
{
    auto outputPath = [&output](Path *p)
    {
        p->toWKT(output);
    };

    output << "MULTIPOLYGON (";

    auto it = m_roots.begin();
    if (it != m_roots.end())
        outputPath(*it++);
    for (; it != m_roots.end(); ++it)
    {
        output << ",";
        outputPath(*it);
    }
    output << ")";
}

} // namespace hexer
