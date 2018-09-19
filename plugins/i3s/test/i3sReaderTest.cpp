#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"

#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/FileUtils.hpp>

#include <io/LasReader.hpp>
#include <io/LasWriter.hpp>
#include "../io/i3sReader.hpp"

using namespace pdal;

TEST(i3sReaderTest, i3sReaderTest_read_url)//test with remote file
{
    StageFactory f;
    //create args
    Options i3s_options;
    i3s_options.add("filename", "i3s://https://tiles.arcgis.com/tiles/8cv2FuXuWSfF0nbL/arcgis/rest/services/AUTZEN_LiDAR/SceneServer");
    i3s_options.add("threads", 64);
    i3s_options.add("bounds", 
            "([-123.075542,-123.06196],[44.049719,44.06278]))");//full extents
    
    I3SReader reader;
    reader.setOptions(i3s_options);

    PointTable table;
    reader.prepare(table);

    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(view->size(), 10653336u);

} 

TEST(i3sReaderTest, i3sReaderTest_read_local)
{
    
    StageFactory f;
    //create args
    Options i3s_options;
    i3s_options.add("filename", Support::datapath("SMALL_AUTZEN_LAS_All.slpk"));
    i3s_options.add("threads", 64);
    
    I3SReader reader;
    reader.setOptions(i3s_options);

    PointTable table;
    reader.prepare(table);

    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(view->size(), 106u);
}

TEST(i3sReaderTest, i3sReaderTest_remote_bounded)
{
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

    double x, y, z;
    for(std::size_t i = 0; i < view->size(); i++)
    {
        x = view->getFieldAs<double>(Dimension::Id::X, i);
        y = view->getFieldAs<double>(Dimension::Id::Y, i);
        z = view->getFieldAs<double>(Dimension::Id::Z, i);
        ASSERT_TRUE(bounds.contains(x,y,z));
    }
}


TEST(i3sReaderTest, i3sReaderTest_local_bounded)
{
    StageFactory f;
    //create args
    Options i3s_options;
    i3s_options.add("filename", Support::datapath("SMALL_AUTZEN_LAS_All.slpk"));
    i3s_options.add("threads", 64);
    i3s_options.add("bounds", "([-123.077,-123.063],[44.053, 44.060], [130, 175])");
    
    I3SReader reader;
    reader.setOptions(i3s_options);

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
}
