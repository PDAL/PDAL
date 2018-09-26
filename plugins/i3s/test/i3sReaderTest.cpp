#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"

#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/FileUtils.hpp>

#include <io/LasReader.hpp>
#include <io/LasWriter.hpp>
#include "../io/I3SReader.hpp"
#include "../io/SlpkReader.hpp"

using namespace pdal;
//test full autzen lidar i3s with bounds that hold the entire data
TEST(i3sReaderTest, i3sReaderTest_read_url)
{
    StageFactory f;
    //create args
    Options i3s_options;
    i3s_options.add("filename", "i3s://https://tiles.arcgis.com/tiles/8cv2FuXuWSfF0nbL/arcgis/rest/services/AUTZEN_LiDAR/SceneServer");
    i3s_options.add("threads", 64);
    i3s_options.add("bounds",
            "([-123.075542,-123.06196],[44.049719,44.06278]))");//full extents
    i3s_options.add("dims", "RGB, INTENSITY");

    I3SReader reader;
    reader.setOptions(i3s_options);

    PointTable table;
    reader.prepare(table);

    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 10653336u);
    ASSERT_TRUE(table.layout()->hasDim(Dimension::Id::Red));
    ASSERT_TRUE(table.layout()->hasDim(Dimension::Id::Intensity));
    ASSERT_FALSE(table.layout()->hasDim(Dimension::Id::NumberOfReturns));


}


//Test full autzen lidar i3s bounded compared to the full without bounds.
//Check that the node trimming is working correctly
TEST(i3sReaderTest, i3sReaderTest_remote_bounded)
{
    //first run
    StageFactory f;
    //create args
    Options i3s_options;
    i3s_options.add("filename", "i3s://https://tiles.arcgis.com/tiles/8cv2FuXuWSfF0nbL/arcgis/rest/services/AUTZEN_LiDAR/SceneServer");
    i3s_options.add("threads", 64);
    i3s_options.add("bounds", "([-123.077,-123.063],[44.053, 44.060], [130, 175])");

    I3SReader reader;
    reader.setOptions(i3s_options);

    PointTable table;
    reader.prepare(table);

    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();

    BOX3D bounds = reader.createBounds();

    //second run
    StageFactory f2;
    Options options2;
    options2.add("filename", "i3s://https://tiles.arcgis.com/tiles/8cv2FuXuWSfF0nbL/arcgis/rest/services/AUTZEN_LiDAR/SceneServer");
    options2.add("threads", 64);

    I3SReader reader2;
    reader2.setOptions(options2);

    PointTable table2;
    reader2.prepare(table2);

    PointViewSet viewSet2 = reader2.execute(table2);
    PointViewPtr view2 = *viewSet2.begin();

    double x, y, z;
    for(std::size_t i = 0; i < view->size(); i++)
    {
        x = view->getFieldAs<double>(Dimension::Id::X, i);
        y = view->getFieldAs<double>(Dimension::Id::Y, i);
        z = view->getFieldAs<double>(Dimension::Id::Z, i);
        ASSERT_TRUE(bounds.contains(x,y,z));
    }

    unsigned int pointcount = 0;
    for(std::size_t i = 0; i < view2->size(); i++)
    {
        x = view2->getFieldAs<double>(Dimension::Id::X, i);
        y = view2->getFieldAs<double>(Dimension::Id::Y, i);
        z = view2->getFieldAs<double>(Dimension::Id::Z, i);
        if(bounds.contains(x,y,z))
                pointcount++;
    }
    EXPECT_EQ(view->size(), pointcount);

}

