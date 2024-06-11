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

#include <filters/CropFilter.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(GridDecimationFilterTest, create)
{
    StageFactory f;
    Stage* filter(f.createStage("filters.gridDecimation"));
    EXPECT_TRUE(filter);
}

// to be coherent with the grid decimation algorithm
bool contains(BOX2D box, double x, double y)
    { return box.minx <= x && x < box.maxx && box.miny <= y && y < box.maxy; }

TEST(DecimationFilterTest, GridDecimationFilterTest_test_empty)
{
    Options ro;
    ro.add("filename", Support::datapath("las/4_6_crop.las"));

    StageFactory factory;
    Stage& r = *(factory.createStage("readers.las"));
    r.setOptions(ro);

    Options gdOps;
    gdOps.add("output_type", "max");
    gdOps.add("resolution", "10.");
    gdOps.add("where", "Classification==10"); // no points at 10
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

    EXPECT_EQ(nbThreadPts, 0);
}

TEST(DecimationFilterTest, GridDecimationFilterTest_test1)
{
    Options ro;
    ro.add("filename", Support::datapath("las/4_6_crop.las"));
     
    StageFactory factory;
    Stage& r = *(factory.createStage("readers.las"));
    r.setOptions(ro);

    std::vector<double> resolution = {10., 10.3, 9.8, 9.6};
    
    for (auto res : resolution)
    {
        Options gdOps;
        gdOps.add("output_type", "max");
        gdOps.add("resolution", res);
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
        
        BOX2D bounds;
        view->calculateBounds(bounds);
        size_t d_width = std::floor((bounds.maxx - bounds.minx) / res) + 1;
        size_t d_height = std::floor((bounds.maxy - bounds.miny) / res) + 1;
        EXPECT_EQ(nbThreadPts, d_width*d_height);
        
        double xmin = bounds.minx;
        double ymin = bounds.miny;
        
        for (size_t l(0); l<d_height; l++){
            for (size_t c(0); c<d_width; c++){
                
                BOX2D dstBounds(xmin + c*res, ymin + l*res, xmin + (c+1)*res, ymin + (l+1)*res);
                
                int nbThreadPtsCrop (0);
                double Zmax(0), ZmaxGrid(0);
                for (PointId i = 0; i < view->size(); ++i)
                {
                    PointRef point = view->point(i);
                    double x_pt = point.getFieldAs<double>(Dimension::Id::X);
                    double y_pt = point.getFieldAs<double>(Dimension::Id::Y);
                    if (!contains(dstBounds, x_pt, y_pt)) continue;
                    
                    double z_pt = point.getFieldAs<double>(Dimension::Id::Z);
                    if (Zmax==0 or z_pt > Zmax) Zmax = z_pt;
                    
                    uint8_t classification = point.getFieldAs<uint8_t>(Dimension::Id::Classification);
                    if (classification==5) {nbThreadPtsCrop++; ZmaxGrid=z_pt;}
                }
                
                EXPECT_EQ(nbThreadPtsCrop, 1);
                EXPECT_EQ(Zmax, ZmaxGrid);
            }
        }
    }
   
}
