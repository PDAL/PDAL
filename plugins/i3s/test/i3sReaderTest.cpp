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

void compareI3sLas(const std::string& i3sFilename,
        Options& i3sOptions, const std::string& lasFilename)
{
    I3SReader i;
    i3sOptions.add("filename", i3sFilename);
    i.setOptions(i3sOptions);
    
    LasReader l;
    Options lo;
    lo.add("filename", lasFilename);
    l.setOptions(lo);

    PointTable it;
    i.prepare(it);
    PointViewSet is = i.execute(it);
    EXPECT_EQ(is.size(), 1U);
    PointViewPtr iv = * is.begin();

    PointTable lt;
    l.prepare(lt);
    PointViewSet ls = l.execute(lt);
    EXPECT_EQ(ls.size(), 1U);
    PointViewPtr lv = *ls.begin(); 

    EXPECT_EQ(iv->size(), lv->size());

    //Validate some point data
    for (PointId i = 0; i < lv->size(); ++i)
    {
       EXPECT_DOUBLE_EQ(iv->getFieldAs<double>(Dimension::Id::X, i),
           lv->getFieldAs<double>(Dimension::Id::X, i));
       EXPECT_DOUBLE_EQ(iv->getFieldAs<double>(Dimension::Id::Y, i),
           lv->getFieldAs<double>(Dimension::Id::Y, i));
       EXPECT_DOUBLE_EQ(iv->getFieldAs<double>(Dimension::Id::Z, i),
           lv->getFieldAs<double>(Dimension::Id::Z, i));
    }
}

TEST(i3sReaderTest, test_one)
{
    StageFactory f;
    Options i3s_options;
    i3s_options.add("filename", "i3s://https://tiles.arcgis.com/tiles/8cv2FuXuWSfF0nbL/arcgis/rest/services/AUTZEN_LiDAR/SceneServer");

    PointTable table;
    Stage* i3s_reader(f.createStage("readers.i3s"));
    EXPECT_TRUE(i3s_reader);
    i3s_reader->setOptions(i3s_options);
    i3s_reader->prepare(table);
     
    PointViewSet viewSet = nitf_reader->execute(table);
    PointViewPtr view = *viewSet.begin();
    PointLayoutPtr layout = view->layout();
    Dimension::Id 

    //check metadata

} 


