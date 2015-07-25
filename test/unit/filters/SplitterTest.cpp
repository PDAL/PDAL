/******************************************************************************
* Copyright (c) 2014, Bradley J Chambers (brad.chambers@gmail.com)
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
#include <SplitterFilter.hpp>
#include "Support.hpp"

using namespace pdal;

TEST(SplitterTest, test_tile_filter)
{
    StageFactory f;

    // create the reader
    Options ops1;
    ops1.add("filename", Support::datapath("las/1.2-with-color.las"));
    LasReader r;
    r.setOptions(ops1);

    Options o;
    Option length("length", 1000, "length");
    o.add(length);

    // create the tile filter and prepare
    SplitterFilter s;
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

    auto sorter = [](PointViewPtr p1, PointViewPtr p2)
    {
        BOX2D b1, b2;

        p1->calculateBounds(b1);
        p2->calculateBounds(b2);

        return b1.minx < b2.minx ?  true :
            b1.minx > b2.minx ? false :
            b1.miny < b2.miny;
    };
    std::sort(views.begin(), views.end(), sorter);

    EXPECT_EQ(views.size(), 15u);
    size_t counts[] = {24, 27, 26, 27, 10, 166, 142, 76, 141, 132, 63, 70, 67,
        34, 60 };
    for (size_t i = 0; i < views.size(); ++i)
    {
        PointViewPtr view = views[i];
        EXPECT_EQ(view->size(), counts[i]);
    }
}
