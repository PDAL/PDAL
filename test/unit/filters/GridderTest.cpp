/******************************************************************************
* Copyright (c) 2015, Oscar Martinez Rubi (o.rubi@esciencecenter.nl)
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

#include <pdal/StageFactory.hpp>
#include <pdal/StageWrapper.hpp>
#include <LasReader.hpp>
#include <GridderFilter.hpp>
#include "Support.hpp"

using namespace pdal;

TEST(GridderTest, test_tile_filter)
{
    StageFactory f;

    // create the reader
    Options ops1;
    ops1.add("filename", Support::datapath("las/1.2-with-color.las"));
    LasReader r;
    r.setOptions(ops1);

    Options o;
    Option num_x("num_x", 4, "num_x");
    Option num_y("num_y", 4, "num_y");
    Option min_x("min_x", 635000, "min_x");
    Option min_y("min_y", 845000, "min_y");
    Option max_x("max_x", 640000, "max_x");
    Option max_y("max_y", 855000, "max_y");
    o.add(num_x);
    o.add(num_y);
    o.add(min_x);
    o.add(min_y);
    o.add(max_x);
    o.add(max_y);

    // create the grid filter and prepare
    GridderFilter s;
    s.setOptions(o);
    s.setInput(r);

    PointTable table;
    PointViewPtr view(new PointView(table));
    s.prepare(table);

    StageWrapper::ready(r, table);
    PointViewSet viewSet = StageWrapper::run(r, view);
    StageWrapper::done(r, table);
    EXPECT_EQ(viewSet.size(), 1u);
    view = *viewSet.begin();

    StageWrapper::ready(s, table);
    viewSet = StageWrapper::run(s, view);
    StageWrapper::done(s, table);

    std::vector<PointViewPtr> views;
    for (auto it = viewSet.begin(); it != viewSet.end(); ++it)
        views.push_back(*it);

    EXPECT_EQ(views.size(), 12u);
    size_t counts[] = {73, 42, 108, 233, 6, 101, 45, 216, 40, 103, 79, 19 };
                      
    for (size_t i = 0; i < views.size(); ++i)
    {
        PointViewPtr view = views[i];
        EXPECT_EQ(view->size(), counts[i]);
    }

}
