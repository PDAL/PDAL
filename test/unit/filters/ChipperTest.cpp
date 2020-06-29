/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/Options.hpp>
#include <pdal/StageWrapper.hpp>
#include <filters/ChipperFilter.hpp>
#include <io/LasWriter.hpp>
#include <io/LasReader.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(ChipperTest, test_construction)
{
    PointTable table;

    Options ops1;
    std::string filename(Support::datapath("las/1.2-with-color.las"));
    ops1.add("filename", filename);
    LasReader reader;
    reader.setOptions(ops1);

    {
        // need to scope the writer, so that's it dtor can use the stream

        Options options;
        Option capacity("capacity", 15);
        options.add(capacity);

        ChipperFilter chipper;
        chipper.setInput(reader);
        chipper.setOptions(options);
        chipper.prepare(table);
        PointViewSet viewSet = chipper.execute(table);
        EXPECT_EQ(viewSet.size(), 71u);

        std::vector<PointViewPtr> views;
        for (auto it = viewSet.begin(); it != viewSet.end(); ++it)
            views.push_back(*it);

        auto sorter = [](PointViewPtr p1, PointViewPtr p2)
        {
            //This is super inefficient, but we're doing tests.
            BOX2D b1;
            BOX2D b2;

            p1->calculateBounds(b1);
            p2->calculateBounds(b2);

            return b1.minx < b2.minx ?  true :
                b1.minx > b2.minx ? false :
                b1.miny < b2.miny;
        };

        std::sort(views.begin(), views.end(), sorter);

        PointViewPtr view = views[2];
        BOX2D bounds;
        view->calculateBounds(bounds);

        EXPECT_NEAR(bounds.minx, 635674.05, 0.05);
        EXPECT_NEAR(bounds.maxx, 635993.93, 0.05);
        EXPECT_NEAR(bounds.miny, 848992.45, 0.05);
        EXPECT_NEAR(bounds.maxy, 849427.07, 0.05);

        for (size_t i = 0; i < views.size(); ++i)
            EXPECT_EQ(views[i]->size(), 15u);
    }
}


// Test that the chipper runs with multiple inputs.
TEST(ChipperTest, issue_2479)
{
    PointTable table;
    PointViewPtr view(new PointView(table));

    LasReader r1;
    LasReader r2;
    Options rOpts;

    rOpts.add("filename", Support::datapath("las/autzen_trim.las"));
    r1.setOptions(rOpts);
    r2.setOptions(rOpts);

    ChipperFilter chipper;
    chipper.setInput(r1);
    chipper.setInput(r2);

    chipper.prepare(table);
    PointViewSet viewSet = chipper.execute(table);

    EXPECT_EQ(viewSet.size(), 44u);
}

// Make sure things don't crash if the point buffer is empty.
TEST(ChipperTest, empty_buffer)
{
    PointTable table;
    PointViewPtr view(new PointView(table));

    Options ops;

    ChipperFilter chipper;
    chipper.prepare(table);
    StageWrapper::ready(chipper, table);
    PointViewSet viewSet = StageWrapper::run(chipper, view);
    StageWrapper::done(chipper, table);

    EXPECT_EQ(viewSet.size(), 0u);
}

