
#include <pdal/pdal_test_main.hpp>

#include <filters/M3C2Filter.hpp>

#include <io/LasReader.hpp>
#include <io/BufferReader.hpp>
#include <io/TextReader.hpp>

#include "Support.hpp"

namespace pdal
{

// 3 view input (fixed, comparison, core points)
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
    fo.add("normal_radius", 5.24414);
    fo.add("cyl_radius", 10.4882815);
    fo.add("cyl_halflen", 2.763006);

    filter.setOptions(fo);
    filter.setInput(r1);
    filter.setInput(r2);
    filter.setInput(r3);

    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);

    EXPECT_EQ(3u, viewSet.size());
    // The 3 core points should be in the 3rd view
    PointViewPtr viewOut = *std::next(viewSet.begin(), 2);

    Dimension::Id distance = viewOut->layout()->findDim("m3c2_distance");
    Dimension::Id uncertainty = viewOut->layout()->findDim("m3c2_uncertainty");
    Dimension::Id significant = viewOut->layout()->findDim("m3c2_significant");
    Dimension::Id stdDev1 = viewOut->layout()->findDim("m3c2_std_dev1");
    Dimension::Id stdDev2 = viewOut->layout()->findDim("m3c2_std_dev2");
    Dimension::Id c1 = viewOut->layout()->findDim("m3c2_count1");
    Dimension::Id c2 = viewOut->layout()->findDim("m3c2_count2");

    EXPECT_TRUE(viewOut->hasDim(distance));
    EXPECT_TRUE(viewOut->hasDim(uncertainty));
    EXPECT_TRUE(viewOut->hasDim(significant));
    EXPECT_TRUE(viewOut->hasDim(stdDev1));
    EXPECT_TRUE(viewOut->hasDim(stdDev2));
    EXPECT_TRUE(viewOut->hasDim(c1));
    EXPECT_TRUE(viewOut->hasDim(c2));

    EXPECT_NEAR(viewOut->getFieldAs<float>(distance, 0), 1.396, 0.01);
    EXPECT_NEAR(viewOut->getFieldAs<float>(distance, 1), 0.670, 0.01);
    EXPECT_NEAR(viewOut->getFieldAs<float>(distance, 2), 1.246, 0.01);
    for (size_t i = 0; i < viewOut->size(); ++i)
        EXPECT_EQ(viewOut->getFieldAs<float>(significant, i), 1);
    EXPECT_NEAR(viewOut->getFieldAs<float>(uncertainty, 0), 0.275, 0.01);
    EXPECT_NEAR(viewOut->getFieldAs<float>(uncertainty, 1), 0.140, 0.01);
    EXPECT_NEAR(viewOut->getFieldAs<float>(uncertainty, 2), 0.226, 0.01);

    // Just checking for point 0 here.
    EXPECT_NEAR(viewOut->getFieldAs<float>(stdDev1, 0), 0.732, 0.01);
    EXPECT_NEAR(viewOut->getFieldAs<float>(stdDev2, 0), 1.582, 0.01);
    EXPECT_EQ(viewOut->getFieldAs<int>(c1, 0), 118);
    EXPECT_EQ(viewOut->getFieldAs<int>(c2, 0), 166);

}

// 2 view input (first view is core points)
TEST(M3C2FilterTest, test2)
{
    LasReader r1;
    LasReader r2;

    Options ro1;
    Options ro2;
    ro1.add("filename", Support::datapath("autzen/autzen-bmx-2010.las"));
    ro2.add("filename", Support::datapath("autzen/autzen-bmx-2023.las"));
    r1.setOptions(ro1);
    r2.setOptions(ro2);

    M3C2Filter filter;
    Options fo;
    fo.add("normal_radius", 2);
    fo.add("cyl_radius", 5);
    fo.add("cyl_halflen", 2.5);
    fo.add("sample_pct", 100);

    filter.setOptions(fo);
    filter.setInput(r1);
    filter.setInput(r2);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);

    EXPECT_EQ(2u, viewSet.size());

    PointViewPtr viewOut = *viewSet.begin();

    Dimension::Id distance = viewOut->layout()->findDim("m3c2_distance");
    EXPECT_TRUE(viewOut->hasDim(distance));
    EXPECT_NEAR(viewOut->getFieldAs<float>(distance, 0), 0.371, 0.01);

    // verifying we get the correct number of points in a sample
    M3C2Filter filter2;
    Options fo2;
    fo2.add("sample_pct", 10);
    filter2.setOptions(fo2);
    filter2.setInput(r1);
    filter2.setInput(r2);
    PointTable table2;
    filter2.prepare(table2);
    PointViewSet viewSet2 = filter2.execute(table2);
    PointViewPtr viewOut2 = *viewSet2.begin();

    // Checking that we only get 83 points in the output 
    // (10% of the # of input (829), rounds down from 82.9 currently)
    int nonZero = 0;
    for (size_t i = 0; i < viewOut2->size(); ++i)
        if (viewOut2->getFieldAs<float>(distance, i) != 0)
            nonZero++;

    EXPECT_EQ(nonZero, 82);
}

// Comparing core points to those created by CloudCompare
TEST(M3C2FilterTest, verifyPoints)
{
    LasReader r1;
    LasReader r2;
    LasReader r3;

    Options ro1;
    Options ro2;
    Options ro3;

    ro1.add("filename", Support::datapath("autzen/autzen-bmx-2010.las"));
    ro2.add("filename", Support::datapath("autzen/autzen-bmx-2023.las"));
    ro3.add("filename", Support::datapath("autzen/autzen-bmx-largersample.las"));
    r1.setOptions(ro1);
    r2.setOptions(ro2);
    r3.setOptions(ro3);

    M3C2Filter filter;
    Options fo;
    fo.add("normal_radius", 1.39);
    fo.add("cyl_radius", 2.89);
    fo.add("cyl_halflen", 5.5);

    filter.setOptions(fo);
    filter.setInput(r1);
    filter.setInput(r2);
    filter.setInput(r3);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);

    EXPECT_EQ(3u, viewSet.size());

    PointViewPtr viewOut = *std::next(viewSet.begin(), 2);

    // exported CC comparison cloud
    LasReader comp_reader;
    Options comp_ro;
    comp_ro.add("filename", Support::datapath("autzen/autzen-bmx-cc.las"));
    comp_reader.setOptions(comp_ro);

    PointTable comp_table;
    comp_reader.prepare(comp_table);
    PointViewSet comp_viewSet = comp_reader.execute(comp_table);
    PointViewPtr comp_viewOut = *comp_viewSet.begin();

    EXPECT_EQ(viewOut->size(), comp_viewOut->size());

    for (size_t i = 0; i < viewOut->size(); ++i)
    {
        EXPECT_EQ(viewOut->getFieldAs<float>(Dimension::Id::X, i),
            comp_viewOut->getFieldAs<float>(Dimension::Id::X, i));
        EXPECT_EQ(viewOut->getFieldAs<float>(Dimension::Id::Y, i),
            comp_viewOut->getFieldAs<float>(Dimension::Id::Y, i));
        EXPECT_EQ(viewOut->getFieldAs<float>(Dimension::Id::Z, i),
            comp_viewOut->getFieldAs<float>(Dimension::Id::Z, i));
    }

    Dimension::Id distance = viewOut->layout()->findDim("m3c2_distance");
    Dimension::Id comp_distance = comp_viewOut->layout()->findDim("M3C2 distance");
    EXPECT_TRUE(viewOut->hasDim(distance) && comp_viewOut->hasDim(comp_distance));

    for (size_t i = 0; i < viewOut->size(); ++i)
    {
        // Ignoring nans 
        if (std::isnan(viewOut->getFieldAs<float>(distance, i)) && std::isnan(comp_viewOut->getFieldAs<float>(comp_distance, i)))
            continue;
        EXPECT_NEAR(viewOut->getFieldAs<float>(distance, i),
            comp_viewOut->getFieldAs<float>(comp_distance, i), 0.12);
    }
}

} // namespace pdal
