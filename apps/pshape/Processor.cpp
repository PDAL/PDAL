#include <iomanip>
#include <vector>
#include <fstream>

#include "../AppSupport.hpp"
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

double computeHexSize(const vector<Point>& samples)
{
    (void)samples;
    return 5.0;
}

bool readtest(double& x, double& y)
{
    static int cnt = 0;

    /**
    static double buf[][2] =
    {
        { 0.0, 0.0 },
        { 5.0, 5.0 },
        { 9.0, 1.0 },
        { 9.0, 2.0 },
        { 9.0, 7.0 },
        { 6.0, 0.0 },
        { 7.215, 1.0 },
        { 7.215, 4.0 },
        { 7.215, 5.0 },
        { 7.215, 7.0 },
        { 7.215, 9.0 },
        { 7.215, 11.0 },
        { -4.0, 0.0 },
        { -4.0, 5.5 },
        { -5.0, 11.0 },
        { -1.0, 1.0 },
        { -1.0, 3.0 },
        { -1.0, 6.0 },
        { -1.0, 9.0 },
    };
    **/

    static double buf[][2] =
    {
        { 0.0, 0.0 },
        { 5.0, 5.0 },
        { 5.0, 20.0 },
        { 12.0, 14.0 }
    };

    static int num = sizeof(buf) / sizeof(buf[0]);

    if (cnt >= num)
       return false;

    x = buf[cnt][0];
    y = buf[cnt][1];
    cnt++;
    return true;
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
        cerr << "Problem reading point number " << i << "!\n";
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

void process()
{
    vector<Point> samples;

    int cnt = 0;
    double x, y;

    while (readlas(x, y) && (cnt < SAMPLE_COUNT))
    {
        samples.push_back(Point(x,y));
        cnt++;
    }

    double hexsize = computeHexSize(samples);
    int dense_limit = 10;
    HexGrid grid(hexsize, dense_limit);
    //ABELL - Need to do something about the density calculation.

    for (int i = 0; i < samples.size(); ++i)
    {
        grid.addPoint(samples[i]);
    }
    while (readlas(x, y))
    {
        grid.addPoint(Point(x, y));
    }
    grid.drawHexagons();
//    grid.dumpInfo();
    grid.findShapes();
/**
    grid.extractShapes();
**/
    // while (true)
    //     ;
}

} //namespace Pshape

int main()
{
    Pshape::process();
    return 0;
}
