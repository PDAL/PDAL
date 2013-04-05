/**
#include <iomanip>
#include <vector>
#include <fstream>

#include "../AppSupport.hpp"
#include "HexGrid.hpp"
**/

#include "Processor.hpp"

#include <math.h>
#include <vector>

#include "GridInfo.hpp"
#include "Mathpair.hpp"

using namespace std;

namespace Pshape
{

namespace
{

// Max number of points to read to determine grid spacing.
const int SAMPLE_COUNT = 5000;

double distance(const Point& p1, const Point& p2)
{
    double xdist = p2.m_x - p1.m_x;
    double ydist = p2.m_y - p1.m_y;
    return sqrt(xdist * xdist + ydist * ydist);
}

// Compute hex size based on distance between consecutive points and density.
// The probably needs some work based on more data.
double computeHexSize(const vector<Point>& samples, int density)
{
    double dist = 0;
    for (int i = 0; i < samples.size() - 1; ++i)
    {
       Point p1 = samples[i];
       Point p2 = samples[i + 1];
       dist += distance(p1, p2);
    }
    return ((density * dist) / samples.size());
}

} // unnamed namespace

void process(const std::vector<GridInfo *>& infos, PointReader reader)
{
    vector<Point> samples;

    int cnt = 0;
    double x, y;

    while (reader(x, y) && (cnt < SAMPLE_COUNT))
    {
        samples.push_back(Point(x,y));
        cnt++;
    }

    for (size_t gi = 0; gi < infos.size(); ++gi)
    {
        GridInfo *info = infos[gi];
        double hexsize = computeHexSize(samples, info->m_density);
        hexsize = (info->m_hexsize < 0) ?
            (-hexsize * info->m_hexsize) : info->m_hexsize;
        HexGrid *grid = new HexGrid(hexsize, info->m_density);
        info->m_grid = grid;

        for (int i = 0; i < samples.size(); ++i)
        {
            grid->addPoint(samples[i]);
        }
        while (reader(x, y))
        {
            grid->addPoint(Point(x, y));
        }
        grid->findShapes();
        grid->findParentPaths();
    }
}

void processHexes(const std::vector<GridInfo *>& infos, HexReader reader)
{
    int cnt = 0;
    int x, y;

    for (size_t gi = 0; gi < infos.size(); ++gi)
    {
        GridInfo *info = infos[gi];
        assert(info->m_hexsize > 0);
        assert(info->m_density > 0);
        HexGrid *grid = new HexGrid(info->m_hexsize, info->m_density);
        info->m_grid = grid;

        while (reader(x, y))
        {
            grid->addDenseHexagon(x, y);
        }
        grid->findShapes();
        grid->findParentPaths();
    }
}

} //namespace Pshape

