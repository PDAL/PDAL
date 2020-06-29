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
#include <io/LasReader.hpp>
#include <io/FauxReader.hpp>
#include <filters/SplitterFilter.hpp>
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
    Option length("length", 1000);
    o.add(length);

    // create the tile filter and prepare
    SplitterFilter s;
    s.setOptions(o);
    s.setInput(r);

    PointTable table;
    s.prepare(table);
    PointViewSet viewSet = s.execute(table);

    std::vector<PointViewPtr> views;
    std::map<PointViewPtr, BOX2D> bounds;

    for (auto it = viewSet.begin(); it != viewSet.end(); ++it)
    {
        BOX2D b;
        PointViewPtr v = *it;
        v->calculateBounds(b);
        EXPECT_TRUE(b.maxx - b.minx <= 1000);    
        EXPECT_TRUE(b.maxy - b.miny <= 1000);    

        for (auto& p : bounds)
            EXPECT_FALSE(p.second.overlaps(b));

        bounds[v] = b;
        views.push_back(v);
    }

    auto sorter = [&bounds](PointViewPtr p1, PointViewPtr p2)
    {
        BOX2D b1 = bounds[p1];
        BOX2D b2 = bounds[p2];

        return b1.minx < b2.minx ?  true :
            b1.minx > b2.minx ? false :
            b1.miny < b2.miny;
    };
    std::sort(views.begin(), views.end(), sorter);

    EXPECT_EQ(views.size(), 24u);
    size_t counts[] = {24, 25, 2, 26, 27, 10, 82, 68, 43, 57, 7, 71, 73,
        61, 33, 84, 74, 4, 59, 70, 67, 34, 60, 4 };
    for (size_t i = 0; i < views.size(); ++i)
    {
        PointViewPtr view = views[i];
        EXPECT_EQ(view->size(), counts[i]);
    }
}

TEST(SplitterTest, test_buffer)
{
    Options readerOptions;
    readerOptions.add("filename", Support::datapath("las/1.2-with-color.las"));
    LasReader reader;
    reader.setOptions(readerOptions);

    Options splitterOptions;
    splitterOptions.add("length", 1000);
    splitterOptions.add("buffer", 20);

    SplitterFilter splitter;
    splitter.setOptions(splitterOptions);
    splitter.setInput(reader);

    PointTable table;
    splitter.prepare(table);
    PointViewSet viewSet = splitter.execute(table);

    std::vector<PointViewPtr> views;
    std::map<PointViewPtr, BOX2D> bounds;
    for (auto it = viewSet.begin(); it != viewSet.end(); ++it)
    {
        BOX2D b;
        PointViewPtr v = *it;
        v->calculateBounds(b);
        EXPECT_TRUE(b.maxx - b.minx <= 1040);    
        EXPECT_TRUE(b.maxy - b.miny <= 1040);
        bounds[v] = b;
        views.push_back(v);
    }

    auto sorter = [&bounds](PointViewPtr p1, PointViewPtr p2)
    {
        BOX2D b1 = bounds[p1];
        BOX2D b2 = bounds[p2];

        return b1.minx < b2.minx ?  true :
            b1.minx > b2.minx ? false :
            b1.miny < b2.miny;
    };
    std::sort(views.begin(), views.end(), sorter);

    EXPECT_EQ(views.size(), 24u);
    size_t counts[] = {26, 26, 3, 28, 27, 13, 14, 65, 80, 47, 80, 89, 94,
        77, 5, 79, 65, 34, 63, 67, 74, 69, 36, 5};
    size_t i = 0;
    for (PointViewPtr view : views)
        EXPECT_EQ(view->size(), counts[i++]);
}

// This test make sure bounds are correct by using known and calculable counts.
TEST(SplitterTest, test_buffer2)
{
    Options readerOptions;
    readerOptions.add("mode", "grid");
    readerOptions.add("bounds", BOX3D(0, 0, 0, 1000, 1000, 0));

    FauxReader reader;
    reader.setOptions(readerOptions);

    Options splitterOptions;
    splitterOptions.add("length", 300);
    splitterOptions.add("origin_x", 500);
    splitterOptions.add("origin_y", 500);
    splitterOptions.add("buffer", 25);

    SplitterFilter splitter;
    splitter.setOptions(splitterOptions);
    splitter.setInput(reader);

    PointTable table;
    splitter.prepare(table);
    PointViewSet s = splitter.execute(table);

    EXPECT_EQ(s.size(), 16U); 

    std::vector<PointViewPtr> vvec;
    std::map<PointViewPtr, BOX2D> bounds;
    for (PointViewPtr v : s)
    {
        BOX2D b;
        v->calculateBounds(b);
        bounds[v] = b;
        vvec.push_back(v);
    }

    auto sorter = [&bounds](PointViewPtr p1, PointViewPtr p2)
    {
        BOX2D b1 = bounds[p1];
        BOX2D b2 = bounds[p2];

        return b1.minx < b2.minx ?  true :
            b1.minx > b2.minx ? false :
            b1.miny < b2.miny;
    };
    std::sort(vvec.begin(), vvec.end(), sorter);

    size_t counts[] = { 50625, 78525, 78525, 50400, 78525, 121801, 121801, 
        78176, 78525, 121801, 121801, 78176, 50400, 78176, 78176, 50176 };

    size_t i = 0;
    for (PointViewPtr v : vvec)
        EXPECT_EQ(v->size(), counts[i++]);
}

