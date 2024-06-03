/******************************************************************************
* Copyright (c) 2023, Antoine Lavenant  (antoine.lavenant@ign.fr)
*
* All rights reserved.
*
****************************************************************************/

#include <pdal/pdal_test_main.hpp>

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <io/FauxReader.hpp>
#include <filters/GridDecimationFilter.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(GridDecimationFilterTest, create)
{
    StageFactory f;
    Stage* filter(f.createStage("filters.gridDecimation"));
    EXPECT_TRUE(filter);
}

TEST(DecimationFilterTest, GridDecimationFilterTest_test1)
{
    Options ro;
    ro.add("filename", Support::datapath("las/4_6_crop.las"));
     
    StageFactory factory;
    Stage& r = *(factory.createStage("readers.las"));
    r.setOptions(ro);

    Options gdOps;
    gdOps.add("output_type", "max");
    gdOps.add("resolution", 10.);
    gdOps.add("value", "Classification=5");

    GridDecimationFilter filter;
    filter.setOptions(gdOps);
    filter.setInput(r);

    PointTable table;

    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    
    EXPECT_EQ(viewSet.size(), 1u);
    
    PointViewPtr view = *viewSet.begin();

    int nbThreadPts (0);
    for (PointId i = 0; i < view->size(); ++i)
    {
        PointRef point = view->point(i);
        uint8_t classification = point.getFieldAs<uint8_t>(Dimension::Id::Classification);
        if (classification==5) nbThreadPts++;
    }

    EXPECT_EQ(nbThreadPts, 400);
}
