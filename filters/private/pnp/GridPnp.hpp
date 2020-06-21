#pragma once

#include <random>
#include <vector>
#include <set>

#include "Comparison.hpp"
#include "Grid.hpp"
#include "VoxelRayTrace.hpp"

namespace pdal
{

struct grid_error : public std::runtime_error
{
    grid_error(const std::string& s) : std::runtime_error(s)
    {}
};


class GridPnp
{
public:
    using Point = std::pair<double, double>;
    using Ring = std::vector<Point>;

    // Initialize the point-in-poly engine by creating an overlay grid and
    // attaching the index of each polygon edge to any cell it passes
    // through.
    // NOTE: This code does only minimal validation to try to make sure
    //  that is won't hang.  Make sure your polygon is valid before
    //  calling.  Make sure there are no self-intersections, for example.
    GridPnp(const Ring& outer, const std::vector<Ring>& inners)
    {
        validateRing(outer);
        for (const Ring& inner : inners)
            validateRing(inner);

        calcBounds(outer);

        fillRingList(outer, inners);
        setupGrid();
    }


    GridPnp(const Ring& outer)
    {
        validateRing(outer);
        calcBounds(outer);

        std::vector<Ring> inners;  // no outers.
        fillRingList(outer, inners);
        setupGrid();
    }

    bool inside(const Point& p) const
    { return inside(p.first, p.second); }

    // Determine if a point is inside the polygon attached to this class.
    bool inside(double x, double y) const
    {
        // Find the index of the grid cell at the position.
        // If the position isn't in the grid, we're certainly outside.
        XYIndex idx;
        if (!m_grid->cellPos(x, y, idx))
            return false;

        Cell& cell = m_grid->cell(idx);
        // If we don't already have a reference point with computed state,
        // do it now.
        if (!cell.computed())
            computeCell(cell, idx);

        // If there are no edges in the cell, the status of the cell is uniform,
        // just return the state.
        if (cell.empty())
            return cell.inside();
        return testCell(cell, x, y);
    }

private:
    using XYIndex = std::pair<size_t, size_t>;
    using Edge = std::pair<Point, Point>;
    using RingList = Ring; // Structure is the same.
    using EdgeId = size_t;

    enum class IntersectType
    {
        Cross,
        On,
        None
    };

    class Cell
    {
    public:
        Cell() : m_point(
            {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::quiet_NaN() })
        {}

        void addEdge(size_t edge)
            { m_edges.push_back(edge); }
        bool empty() const
            { return m_edges.empty(); }
        void setPoint(double x, double y)
            { m_point = Point(x, y); }
        bool computed() const
            { return !std::isnan(m_point.first); }
        GridPnp::Point point() const
            { return m_point; }
        const std::vector<size_t>& edges() const
            { return m_edges; }
        bool inside() const
            { return m_inside; }
        void setInside(bool inside)
            { m_inside = inside; }

    private:
        std::vector<size_t> m_edges;
        bool m_inside;
        GridPnp::Point m_point;
    };

    class EdgeIt
    {
    public:
        EdgeIt(const RingList& r) : m_points(r), m_id(0)
        { skipInvalid(); }

        void next()
        {
            m_id++;
            skipInvalid();
        }

        EdgeId operator * () const
        { return m_id; }

        operator bool () const
        { return m_id < m_points.size() - 1; }

    private:
        void skipInvalid()
        {
            while (m_id < m_points.size() - 1 &&
                (std::isnan(m_points[m_id].first) ||
                 std::isnan(m_points[m_id + 1].first) ||
                 (m_points[m_id] == m_points[m_id + 1])))
            {
                m_id++;
            }
        }

        const RingList& m_points;
        size_t m_id;
    };


    const Point& point1(EdgeId id) const
        { return m_rings[id]; }
    const Point& point2(EdgeId id) const
        { return m_rings[id + 1]; }
    double xval(const Point& p) const
        { return p.first; }
    double yval(const Point& p) const
        { return p.second; }

    void validateRing(const Ring& r) const
    {
        if (r.size() < 4)
            throw grid_error("Invalid ring. Ring must consist of at least "
                " four points.");
        if (r[0] != r[r.size() - 1])
            throw grid_error("Invalid ring. First point is not equal to "
                "the last point.");
    }

    // Calculate the bounding box of the polygon.  At the same time
    // calculate the average length of the X and Y components of the
    // polygon edges.
    void calcBounds(const Ring& outer)
    {
        // Inialize max/min with X/Y of first point.
        const Point& p = outer[0];
        m_xMin = xval(p);
        m_xMax = xval(p);
        m_yMin = yval(p);
        m_yMax = yval(p);

        size_t numEdges = 0;
        // The first point is duplicated as the last, so we skip the last
        // point when looping.
        for (size_t id = 0; id < outer.size() - 1; ++id)
        {
            const Point& p1 = outer[id];
            const Point& p2 = outer[id + 1];

            // Calculate bounding box.
            m_xMin = (std::min)(m_xMin, xval(p1));
            m_xMax = (std::max)(m_xMax, xval(p1));
            m_yMin = (std::min)(m_yMin, yval(p1));
            m_yMax = (std::max)(m_yMax, yval(p1));
        }
    }


    void fillRingList(const Ring& inner, const std::vector<Ring>& outers)
    {
        double nan = std::numeric_limits<double>::quiet_NaN();

        for (size_t i = 0; i < inner.size(); ++i)
            m_rings.push_back(inner[i]);
        for (const Ring& r : outers)
        {
            // Nan is a separator between rings.
            m_rings.push_back({nan, nan});
            for (size_t i = 0; i < r.size(); ++i)
                m_rings.push_back(r[i]);
        }
    }


    void setupGrid()
    {
        double xAvgLen;
        double yAvgLen;

        calcAvgEdgeLen(xAvgLen, yAvgLen);
        XYIndex gridSize = calcGridSize(xAvgLen, yAvgLen);
        createGrid(gridSize);
        assignEdges();
    }


    void calcAvgEdgeLen(double& xAvgLen, double& yAvgLen)
    {
        double xdist{0};
        double ydist{0};
        size_t numEdges{0};
        for (EdgeIt it(m_rings); it; it.next())
        {
            EdgeId id = *it;
            const Point& p1 = point1(*it);
            const Point& p2 = point2(*it);

            // Sum the lengths of the X and Y components of the edges.
            xdist += std::abs(xval(p2) - xval(p1));
            ydist += std::abs(yval(p2) - yval(p1));
            numEdges++;
        }

        // Find the average X and Y component length.
        xAvgLen = xdist / numEdges;
        yAvgLen = ydist / numEdges;
    }

    // The paper calculates an X and Y based on a) the number of edges
    // and b) the relative length of edges in the X and Y direction.
    // This seems fine, but it misses out on considering the common
    // case where a polygon delineates some area of interest.  In this
    // case there is much "empty" space in the interior of the polygon and
    // it seems likely that most pnp tests will happen in the empty interior.
    // So, it would seem that we'd want the grid sufficiently large to cover
    // this case well.  That way, most pnp tests would take no work beyond
    // an array lookup since most cells would be empty.  Lots of tradeoff,
    // though, in preprocessing vs. actual pnp tests.  Hopefully more work
    // can be done on this later.  My stupid way of dealing with this is
    // to set a minimum grid size of 1000 cells.
    XYIndex calcGridSize(double xAvgLen, double yAvgLen) const
    {
        // I'm setting a minimum number of cells as 1000, because, why not?
        // m_rings isn't necessarily an exact count of edges, but it's close
        // enough for this purpose.
        size_t m = (std::max)((size_t)1000, m_rings.size());

        // See paper for this calc.
        double scalex = ((m_xMax - m_xMin) * yAvgLen) /
            ((m_yMax - m_yMin) * xAvgLen);
        double scaley = 1 / scalex;
        size_t mx = (size_t)std::sqrt(m * scalex);
        size_t my = (size_t)std::sqrt(m * scaley);

        // We always round up, because why not.
        return XYIndex(mx + 1, my + 1);
    }


    // Figure out the grid origin.
    void createGrid(XYIndex gridSize)
    {
        // Make the grid extend 1/2 cell beyond bounds box.
        double boxWidth = m_xMax - m_xMin;
        double boxHeight = m_yMax - m_yMin;
        //
        double cellWidth = boxWidth / (gridSize.first - 1);
        double cellHeight = boxHeight / (gridSize.second - 1);
        double xOrigin = m_xMin - (cellWidth / 2);
        double yOrigin = m_yMin - (cellHeight / 2);

        m_grid.reset(new Grid<Cell>(gridSize.first, gridSize.second,
            cellWidth, cellHeight, xOrigin, yOrigin));
        m_xDistribution.reset(
            new std::uniform_real_distribution<>(0, m_grid->cellWidth()));
        m_yDistribution.reset(
            new std::uniform_real_distribution<>(0, m_grid->cellHeight()));
    }


    // Loop through edges.  Add the edge to each cell traversed.
    void assignEdges()
    {
        for (EdgeIt it(m_rings); it; it.next())
        {
            EdgeId id = *it;
            const Point& p1 = point1(id);
            const Point& p2 = point2(id);
            Point origin = m_grid->origin();
            VoxelRayTrace vrt(m_grid->cellWidth(), m_grid->cellHeight(),
                xval(origin), yval(origin),
                xval(p1), yval(p1), xval(p2), yval(p2));
            VoxelRayTrace::CellList traversedCells = vrt.emit();
            for (auto& c : traversedCells)
                m_grid->cell(XYIndex(c.first, c.second)).addEdge(id);
        }
    }


    // Determine if a point is collinear with an edge.
    bool pointCollinear(double x, double y, EdgeId edgeId) const
    {
        const Point& p1 = point1(edgeId);
        const Point& p2 = point2(edgeId);
        const double x1 = xval(p1);
        const double x2 = xval(p2);
        const double y1 = yval(p1);
        const double y2 = yval(p2);

        // If p1 == p2, this will fail.

        // This is the same as saying slopes are equal.
        return Comparison::closeEnough((x - x2) * (y - y1),
            (y - y2) * (x - x1));
    }


    // Put a reference point in the cell.  Figure out if the reference point
    // is inside the polygon.
    void computeCell(Cell& cell, XYIndex& pos) const
    {
        generateRefPoint(cell, pos);
        determinePointStatus(cell, pos);
    }


    // The paper uses point centers, but then has to deal with points that
    // are "singular" (rest on a polygon edge).  But there's nothing special
    // about the point center.  The center is just a point in a cell with
    // a known status (inside or outside the polygon).  So we just pick a point
    // that isn't collinear with any of the segments in the cell.  Eliminating
    // collinearity eliminates special cases when counting crossings.
    void generateRefPoint(Cell& cell, XYIndex& pos) const
    {
        // A test point is valid if it's not collinear with any segments
        // in the cell.
        auto validTestPoint = [this](double x, double y, Cell& cell)
        {
            for (auto edgeId : cell.edges())
                if (pointCollinear(x, y, edgeId))
                    return false;
            return true;
        };

        Grid<Cell>::Point origin = m_grid->cellOrigin(pos);
        double x, y;
        do
        {
            x = xval(origin) + (*m_xDistribution)(m_ranGen);
            y = yval(origin) + (*m_yDistribution)(m_ranGen);
        } while (!validTestPoint(x, y, cell));
        cell.setPoint(x, y);
    }


    // Determine the status of a cell's reference point by drawing a segment
    // from the reference point in the cell to the left and count crossings.
    // Knowing the number of edge crossings and the inside/outside status
    // of the cell determines the status of this reference point.
    // If we're determining the status of the leftmost cell, choose a point
    // to the left of the leftmost cell, which is guaranteed to be outside
    // the polygon.
    void determinePointStatus(Cell& cell, XYIndex& pos) const
    {
        Point p1(cell.point());

        double x1 = xval(p1);
        double y1 = yval(p1);

        size_t intersectCount = 0;
        if (pos.first == 0)
        {
            double x2 = x1 - m_grid->cellWidth();
            double y2 = y1;

            Edge edge{{x1, y1}, {x2, y2}};

            intersectCount = intersections(edge, cell.edges());
        }
        else
        {
            XYIndex prevPos {pos.first - 1, pos.second};
            Cell& prevCell = m_grid->cell(prevPos);
            if (!prevCell.computed())
                computeCell(prevCell, prevPos);
            double x2 = xval(prevCell.point());
            double y2 = yval(prevCell.point());

            Edge edge{{x1, y1}, {x2, y2}};

            // Stick the edges in the current cell and the previous cell
            // in a set so as not to double-count.
            std::set<size_t> edges;
            edges.insert(cell.edges().begin(), cell.edges().end());
            edges.insert(prevCell.edges().begin(), prevCell.edges().end());
            intersectCount = intersections(edge, edges);
            if (prevCell.inside())
                intersectCount++;
        }
        cell.setInside(intersectCount % 2 == 1);
    }


    // Determine the number of intersections between an edge and
    // all edges indexes by the 'edges' list.
    template<typename EDGES>
    size_t intersections(Edge& e1, const EDGES& edges) const
    {
        size_t isect = 0;
        for (auto& edgeId : edges)
        {
            Edge e2{ point1(edgeId), point2(edgeId) };
            if (intersects(e1, e2) != IntersectType::None)
                isect++;
        }
        return isect;
    }


    // Determine if a point in a cell is inside the polygon or outside.
    // We're always calling a point that lies on an edge as 'inside'
    // the polygon.
    bool testCell(Cell& cell, double x, double y) const
    {
        Edge tester({x, y}, cell.point());

        bool inside = cell.inside();
        for (auto edgeIdx: cell.edges())
        {
            Edge other{ point1(edgeIdx), point2(edgeIdx) };
            IntersectType intersection = intersects(tester, other);
            if (intersection == IntersectType::On)
                return true;
            if (intersection == IntersectType::Cross)
                inside = !inside;
        }
        return inside;
    }

    // Determine if two edges intersect.  Note that because of the way
    // we've chosen reference points, the two segments should never be
    // collinear, which eliminates some special cases.
    //
    // One segment endpoint lies on the other if the slope factor (t or u)
    // is one or 0 and the other factor is between 0 and 1.
    // This is standard math, but it's shown nicely on Stack Overflow
    // question 563198.  The variable names map to the good response there.
    IntersectType intersects(Edge& e1, Edge& e2) const
    {
        using Vector = std::pair<double, double>;

        Vector p = e1.first;
        Vector r { e1.second.first - e1.first.first,
            e1.second.second - e1.first.second };
        Vector q = e2.first;
        Vector s { e2.second.first - e2.first.first,
            e2.second.second - e2.first.second };

        // Should never be 0.
        double rCrossS = r.first * s.second - r.second * s.first;
        Vector pq = { q.first - p.first, q.second - p.second };

        //ABELL - This can be improved to eliminate the division
        //  because we're testing for 1 and 0.  Later...
        double pqCrossS = pq.first * s.second - pq.second * s.first;
        double t = (pqCrossS / rCrossS);
        bool tCloseEnough = Comparison::closeEnough(t, 0) ||
            Comparison::closeEnough(t, 1);
        bool intersect = (tCloseEnough || (t > 0 && t < 1));
        if (!intersect)
            return IntersectType::None;

        double pqCrossR = pq.first * r.second - pq.second * r.first;
        double u = (pqCrossR / rCrossS);
        bool uCloseEnough = Comparison::closeEnough(u, 0) ||
            Comparison::closeEnough(u, 1);
        intersect = (uCloseEnough || (u > 0 && u < 1));
        if (intersect)
        {
            if (uCloseEnough || tCloseEnough)
                return IntersectType::On;
            return IntersectType::Cross;
        }
        return IntersectType::None;
    }

    RingList m_rings;
    mutable std::mt19937 m_ranGen;
    std::unique_ptr<std::uniform_real_distribution<>> m_xDistribution;
    std::unique_ptr<std::uniform_real_distribution<>> m_yDistribution;
    std::unique_ptr<Grid<Cell>> m_grid;
    double m_xMin;
    double m_xMax;
    double m_yMin;
    double m_yMax;
};

} // namespace pdal
