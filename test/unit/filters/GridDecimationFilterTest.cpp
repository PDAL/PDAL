/******************************************************************************
* Copyright (c) 2023, Antoine Lavenant (antoine.lavenant@ign.fr)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
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
    ro.add("filename", Support::datapath("text/decimate_file_grid.txt"));

    StageFactory factory;
    Stage& r = *(factory.createStage("readers.text"));
    r.setOptions(ro);

    Options gdOps;
    gdOps.add("output_type", "max");
    gdOps.add("resolution", "10.");
    gdOps.add("where", "Classification==2"); // no points at 2 (all points are at 1)
    gdOps.add("value", "Classification=2");

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
    ro.add("filename", Support::datapath("text/decimate_file_grid.txt"));

    StageFactory factory;
    Stage& r = *(factory.createStage("readers.text"));
    r.setOptions(ro);

    std::vector<double> resolution = {1., 1.3, 0.500000000001, 1.5, 0.8, 0.6, 0.1, 0.49999999999999999999};

    for (size_t it(0); it<1; it++) // max then min
    {
        for (auto res : resolution)
        {
            Options gdOps;
            gdOps.add("output_type", (it==0 ? "max" : "min"));
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
            // cf. GridDecimationFilter comment
            size_t d_width = std::floor((bounds.maxx - bounds.minx) / res) + 2;
            size_t d_height = std::floor((bounds.maxy - bounds.miny) / res) + 2;
            EXPECT_LE(nbThreadPts, d_width*d_height); // some cells coul'd be empty

            double xmin = bounds.minx;
            double ymin = bounds.miny;

            for (size_t l(0); l<d_height; l++){
                for (size_t c(0); c<d_width; c++){

                    BOX2D dstBounds(xmin + c*res, ymin + l*res, xmin + (c+1)*res, ymin + (l+1)*res);

                    int nbThreadPtsCrop (0), nbPtsCrop(0);
                    double Zref(0), ZrefGrid(0);
                    for (PointId i = 0; i < view->size(); ++i)
                    {
                        PointRef point = view->point(i);
                        double x_pt = point.getFieldAs<double>(Dimension::Id::X);
                        double y_pt = point.getFieldAs<double>(Dimension::Id::Y);
                        if (!contains(dstBounds, x_pt, y_pt)) continue;
                        nbPtsCrop++;

                        double z_pt = point.getFieldAs<double>(Dimension::Id::Z);
                        if (it==0)
                        {
                            if(Zref==0 || z_pt > Zref)
                                Zref = z_pt;
                        }
                        else
                        {
                            if(Zref==0 || z_pt < Zref)
                                Zref = z_pt;
                        }

                        uint8_t classification = point.getFieldAs<uint8_t>(Dimension::Id::Classification);
                        if (classification==5) {nbThreadPtsCrop++; ZrefGrid=z_pt;}
                    }

                    if (nbPtsCrop>0)
                    {
                        EXPECT_EQ(nbThreadPtsCrop, 1);
                        EXPECT_EQ(Zref, ZrefGrid);
                    }
                }
            }
        }
    }

}
