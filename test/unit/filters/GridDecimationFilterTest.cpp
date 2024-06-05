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

TEST(DecimationFilterTest, GridDecimationFilterTest_test1)
{
    Options ro;
    ro.add("filename", Support::datapath("las/4_6_crop.las"));
     
    StageFactory factory;
    Stage& r = *(factory.createStage("readers.las"));
    r.setOptions(ro);

    double resolution = 10.;
    
    Options gdOps;
    gdOps.add("output_type", "max");
    gdOps.add("resolution", resolution);
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
    
    double xmin = 1639600.0;
    double ymin = 1454500.02;
    
    for (size_t l(0); l<20; l++){
        for (size_t c(0); l<20; l++){
            
            BOX2D dstBounds(xmin + c*resolution, ymin + l*resolution, xmin + (c+1)*resolution, ymin + (l+1)*resolution);
            
            Options cropOpts;
            cropOpts.add("bounds", dstBounds);
            CropFilter cropfilter;
            cropfilter.setOptions(cropOpts);

            PointTable cropTable;
            cropfilter.prepare(cropTable);
            cropfilter.setInput(filter);
            viewSet = cropfilter.execute(table);
            view = *viewSet.begin();

            int nbThreadPtsCrop (0);
            double Zmax, ZmaxGrid;
            for (PointId i = 0; i < view->size(); ++i)
            {
                PointRef point = view->point(i);
                double z_pt = point.getFieldAs<double>(Dimension::Id::Z);
                if (i==0) Zmax = z_pt;
                else if (z_pt > Zmax) Zmax = z_pt;
                uint8_t classification = point.getFieldAs<uint8_t>(Dimension::Id::Classification);
                if (classification==5) {nbThreadPtsCrop++; ZmaxGrid=z_pt;}
            }
            
            EXPECT_EQ(nbThreadPtsCrop, 1);
            EXPECT_EQ(Zmax, ZmaxGrid);
        }
    }
    
   
}
