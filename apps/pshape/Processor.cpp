#include <iomanip>
#include <vector>
#include <fstream>

#include "../AppSupport.hpp"
#include "GridInfo.hpp"
#include "HexGrid.hpp"
#include "Mathpair.hpp"

#include "pdal/PointBuffer.hpp"
#include "pdal/Reader.hpp"
#include "pdal/StageIterator.hpp"

namespace
{
   // Max number of points to read to determine grid spacing.
   const int SAMPLE_COUNT = 5000;
}

using namespace std;

namespace Pshape
{

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

bool readlas(double& x, double& y)
{
    using namespace pdal;

    static Stage *reader = NULL;
    static StageSequentialIterator *iter = NULL;
    static PointBuffer *buf = NULL;
    static int i = 0;
    static Dimension const *xdim;
    static Dimension const *ydim;

    if (!reader)
    {
        Options options;
        options.add<std::string>("filename", "serpent_mound.las");

        reader = AppSupport::makeReader(options);
        reader->initialize();
        const Schema& schema = reader->getSchema();
        buf = new PointBuffer(schema, 1);
        iter = reader->createSequentialIterator(*buf);

        schema::index_by_index const& dimensions =
            schema.getDimensions().get<schema::index>();
        for (int dim = 0; dim < dimensions.size(); ++dim)
        {
            const Dimension& dimension = dimensions[dim];
            if (dimension.getName() == "X")
            {
                xdim = &dimension;
            }
            else if (dimension.getName() == "Y")
            {
                ydim = &dimension;
            }
        }
    }
    if (!iter->read(*buf))
    {
        delete reader;
        delete buf;
        //ABELL - Must be deleted elsewhere.
        // delete iter;
        return false;
    }
    i++;
    int32_t xi = buf->getField<int32_t>(*xdim, 0);
    int32_t yi = buf->getField<int32_t>(*ydim, 0);
    x = xdim->applyScaling<int32_t>(xi);
    y = ydim->applyScaling<int32_t>(yi);
    return true;
}

// Read from simple file of doubles
bool read(double& x, double& y)
{
    static ifstream in;
    static bool done = false;
    static int i = 0;

    if (done)
    {
        return false;
    }
    if ( !in.is_open() )
    {
        in.open("fastpoints.small");
    }
    if (in.eof() || i == 500000)
    {
        in.close();
        done = true;
        return false;
    }
    in.read((char *)&x, sizeof(x));
    in.read((char *)&y, sizeof(y));
    i++;
    return true;
}

bool readHex(int& x, int& y)
{
    static int pos = 0;
//    static int coords[] = {0, 0};

/**
    static int coords[] = {
        0, 0,
        0, -2,
        1, -2,
        2, -1,
        2, 0,
        2, 1,
        1, 1,
        0, 2,
        -1, 1,
        -2, 1,
        -2, 0,
        -2, -1,
        -1, -2
    };
**/

/**
    static int coords[] = {
        -1, 0,
        0, 0,
        1, 0,
        1, 2,
        2, 2,
        3, 1
    };
**/
    static int coords[] = {
        0, 0,
        1, 0,
        2, 1,
        3, 1,
        4, 2,
        4, 3,
        4, 4,
        3, 4,
        2, 5,
        1, 5,
        0, 5,
        -1, 4,
        -2, 4,
        -3, 3,
        -3, 2,
        -3, 1,
        -2, 1,
        -1, 0,
        -1, 2,
        0, 2,
        1, 3,
        2, 3
    };

    static int num_coords = sizeof(coords) / (sizeof(coords[0]));
    
    if (pos + 1 < num_coords) {
        x = coords[pos++];
        y = coords[pos++];
        return true;
    }
    return false;
}

void processHexes(std::vector<GridInfo *> infos)
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

        while (readHex(x, y))
        {
cerr << "Adding hex " << x << "," << y << "!\n";
            grid->addDenseHexagon(x, y);
        }
cerr << "Finding shapes!\n";
        grid->findShapes();
cerr << "Finding parent paths!\n";
        grid->findParentPaths();
    }
}

void process(std::vector<GridInfo *> infos)
{
    vector<Point> samples;

    int cnt = 0;
    double x, y;

    while (read(x, y) && (cnt < SAMPLE_COUNT))
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
        while (read(x, y))
        {
            grid->addPoint(Point(x, y));
        }
        grid->findShapes();
        grid->findParentPaths();
    }
}

} //namespace Pshape

using namespace Pshape;

std::string indent(int l)
{
    std::string tabs;

    tabs.append(l * 2, ' ');
    return tabs;
}

void dumpPath(Path *p)
{
    static int level = 0;
    Orientation o = p->orientation();
    std::string ostring = ((o == CLOCKWISE) ? "CLOCKWISE" : "ANTICLOCKWISE");
    indent(level);
    cerr << indent(level) << "Path length = " << p->pathLength() << "!\n";
    cerr << indent(level) << "Orientation = " << ostring << "!\n";
    vector<Path *> paths = p->subPaths();
    level++;
    for (int pi = 0; pi != paths.size(); ++pi)
    {
        dumpPath(paths[pi]);
    }
    level--;
}

int main()
{
    vector<GridInfo *> infos;
    GridInfo *gi = new GridInfo;

    gi->m_hexsize = 10;
    gi->m_density = 10;
    infos.push_back(gi);

    processHexes(infos);
    for (int pi = 0; pi < gi->rootPaths().size(); ++pi)
    {
        Path *p = gi->rootPaths()[pi];
        dumpPath(p);
    }

    // Dump hexes.
    for (HexIter iter = gi->begin(); iter != gi->end(); ++iter)
    {
        HexInfo hi = *iter;
        cerr << "Density/X/Y = " << hi.m_density << "/" <<
            hi.m_center.m_x << "/" << hi.m_center.m_y << "!\n";
    }

    delete gi;
    return 0;
}
