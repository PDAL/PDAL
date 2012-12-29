#include <vector>

#include "HexGrid.hpp"
#include "Point.hpp"

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
    return 10.0;
}

bool read(double& x, double& y)
{
    static int cnt = 0;

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

    static int num = sizeof(buf) / sizeof(buf[0]);

    if (cnt >= num)
       return false;

    x = buf[cnt][0];
    y = buf[cnt][1];
    cnt++;
    return true;
}

void process()
{
    vector<Point> samples;

    int cnt = 0;
    double x, y;

    while (read(x, y) && (cnt < SAMPLE_COUNT))
    {
        samples.push_back(Point(x,y));
        cnt++;
    }

    double hexsize = computeHexSize(samples);
    HexGrid grid(hexsize);
    for (int i = 0; i < samples.size(); ++i)
    {
        grid.addPoint(samples[i]);
    }
    while (read(x, y))
    {
        grid.addPoint(Point(x, y));
    }
/**
    grid.findShapes();
    grid.extractShapes();
**/
}

} //namespace Pshape

int main()
{
    Pshape::process();
    return 0;
}
