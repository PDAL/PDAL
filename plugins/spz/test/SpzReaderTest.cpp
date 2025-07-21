
#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"

#include <pdal/StageFactory.hpp>
#include <pdal/PipelineManager.hpp>

#include "SpzReader.hpp"

using namespace pdal;

TEST(SpzReaderTest, test1)
{
    Options opts;
    opts.add("filename", Support::datapath("spz/fourth_st.spz"));
    
    SpzReader reader;
    reader.setOptions(opts);

    PointTable table;
    reader.prepare(table);
    PointViewSet set = reader.execute(table);
    PointViewPtr view = *set.begin();

    EXPECT_EQ(view->size(), 131199);
    ASSERT_TRUE(table.layout()->hasDim(Dimension::Id::X));
    //check custom dimensions too
    Dimension::Id rot0 = table.layout()->findProprietaryDim("rot_0");
    Dimension::Id sh2 = table.layout()->findProprietaryDim("f_dc_2");
    ASSERT_TRUE(table.layout()->hasDim(rot0));
    ASSERT_TRUE(table.layout()->hasDim(sh2));
}

TEST(SpzReaderTest, orientation_test)
{
    Options opts;
    opts.add("filename", Support::datapath("spz/fourth_st.spz"));
    
    SpzReader reader;
    reader.setOptions(opts);

    PointTable origTable;
    reader.prepare(origTable);
    PointViewSet set = reader.execute(origTable);
    PointViewPtr origView = *set.begin();

    //test orientation
    SpzReader reader2;
    opts.add("out_orientation", "LUF");
    reader2.setOptions(opts);

    PointTable table;
    reader2.prepare(table);
    PointViewSet set2 = reader2.execute(table);
    PointViewPtr view = *set2.begin();

    // Input RUB, output LUF: X = -X, Y = Y, Z = -Z
    EXPECT_FLOAT_EQ(view->getFieldAs<float>(Dimension::Id::X, 0),
        (-1.0 * origView->getFieldAs<float>(Dimension::Id::X, 0)));
    EXPECT_FLOAT_EQ(view->getFieldAs<float>(Dimension::Id::Y, 0),
        origView->getFieldAs<float>(Dimension::Id::Y, 0));
    EXPECT_FLOAT_EQ(view->getFieldAs<float>(Dimension::Id::Z, 0),
        (-1.0 * origView->getFieldAs<float>(Dimension::Id::Z, 0)));
}