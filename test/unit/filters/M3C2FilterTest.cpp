
#include <pdal/pdal_test_main.hpp>

#include <filters/M3C2Filter.hpp>

#include <io/LasReader.hpp>
#include <io/BufferReader.hpp>
#include <io/TextReader.hpp>

#include "Support.hpp"

namespace pdal
{


TEST(M3C2FilterTest, test1)
{
    using namespace Dimension;
    PointTable table;

    Options ro1;
    Options ro2;
    Options ro3;
    LasReader r1;
    LasReader r2;
    TextReader r3;
    ro1.add("filename", Support::datapath("autzen/autzen-bmx-2010.las"));
    ro2.add("filename", Support::datapath("autzen/autzen-bmx-2023.las"));
    // contains 3 points picked from the first file to use as core points
    ro3.add("filename", Support::datapath("autzen/autzen-bmx-cores.txt"));
    r1.setOptions(ro1);
    r2.setOptions(ro2);
    r3.setOptions(ro3);

    M3C2Filter filter;
    Options fo;
    fo.add("normal_radius", 10);
    fo.add("cyl_radius", 20);
    fo.add("cyl_halflen", 10);

    filter.setOptions(fo);

    filter.setInput(r1);
    filter.setInput(r2);
    filter.setInput(r3);

    filter.prepare(table);
    
    PointViewSet viewSet = filter.execute(table);

    EXPECT_EQ(3u, viewSet.size());
    // The 3 core points should be in the 3rd view
    PointViewPtr viewOut = *std::next(viewSet.begin(), 2);

    // Values don't look right. working on it
    Dimension::Id distance = viewOut->layout()->findDim("m3c2_distance");
    for (size_t i = 0; i < viewOut->size(); ++i)
        std::cout << viewOut->getFieldAs<float>(distance, i) << std::endl;
}

} // namespace pdal