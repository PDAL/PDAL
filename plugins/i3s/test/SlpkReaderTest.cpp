#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"

#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/FileUtils.hpp>

#include <io/LasReader.hpp>
#include <io/LasWriter.hpp>
#include "../io/SlpkReader.hpp"

using namespace pdal;
//
//test small autzen slpk data provided by esri
TEST(SlpkReaderTest, SlpkReaderTest_read_local)
{

    StageFactory f;
    //create args
    Options slpk_options;
    slpk_options.add("filename", Support::datapath("i3s/SMALL_AUTZEN_LAS_All.slpk"));
    slpk_options.add("threads", 64);

    SlpkReader reader;
    reader.setOptions(slpk_options);

    PointTable table;
    reader.prepare(table);

    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(view->size(), 106u);
}


TEST(SlpkReaderTest, slpkReaderTest_bounded)
{
    StageFactory f;
    //create args
    Options slpk_options;
    slpk_options.add("filename", Support::datapath("i3s/SMALL_AUTZEN_LAS_All.slpk"));
    slpk_options.add("threads", 64);
    slpk_options.add("bounds", "([-123.077,-123.063],[44.053, 44.060], [130, 175])");

    SlpkReader reader;
    reader.setOptions(slpk_options);

    PointTable table;
    reader.prepare(table);

    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();

    BOX3D bounds = reader.createBounds();

    double x, y, z;
    for(std::size_t i = 0; i < view->size(); i++)
    {

        x = view->getFieldAs<double>(Dimension::Id::X, i);
        y = view->getFieldAs<double>(Dimension::Id::Y, i);
        z = view->getFieldAs<double>(Dimension::Id::Z, i);
        ASSERT_TRUE(bounds.contains(x,y,z));
    }

    Options slpk2_options;
    slpk2_options.add("filename", Support::datapath("i3s/SMALL_AUTZEN_LAS_All.slpk"));
    slpk2_options.add("threads", 64);

    SlpkReader reader2;
    reader2.setOptions(slpk2_options);

    PointTable table2;
    reader.prepare(table2);

    PointViewSet viewSet2 = reader.execute(table2);
    PointViewPtr view2 = *viewSet2.begin();

    double x2, y2, z2;
    uint64_t count = 0;
    for(std::size_t i = 0; i < view->size(); i++)
    {
        x2 = view->getFieldAs<double>(Dimension::Id::X, i);
        y2 = view->getFieldAs<double>(Dimension::Id::Y, i);
        z2 = view->getFieldAs<double>(Dimension::Id::Z, i);
        if(bounds.contains(x2,y2,z2))
            count++;
    }

    EXPECT_EQ(view->size(), count);

}
