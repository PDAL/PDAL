
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
