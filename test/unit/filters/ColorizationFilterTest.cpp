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

#include <pdal/PointView.hpp>
#include <io/LasReader.hpp>
#include <filters/ColorizationFilter.hpp>
#include <filters/StreamCallbackFilter.hpp>
#include <filters/TransformationFilter.hpp>

#include "Support.hpp"

using namespace pdal;

namespace
{

void testFile(const Options& filterOps, StringList dimNames,
    uint16_t expectedRed, uint16_t expectedGreen, uint16_t expectedBlue)
{
    Options readerOps;
    readerOps.add("filename",
        Support::datapath("autzen/autzen-point-format-3.las"));

    LasReader reader;
    reader.setOptions(readerOps);

    ColorizationFilter filter;
    filter.setOptions(filterOps);
    filter.setInput(reader);

    PointTable table;

    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    PointLayoutPtr layout = table.layout();
    uint16_t r = view->getFieldAs<uint16_t>(layout->findDim(dimNames[0]), 0);
    uint16_t g = view->getFieldAs<uint16_t>(layout->findDim(dimNames[1]), 0);
    uint16_t b = view->getFieldAs<uint16_t>(layout->findDim(dimNames[2]), 0);

    EXPECT_EQ(r, expectedRed);
    EXPECT_EQ(g, expectedGreen);
    // We scaled this up to 16bit by multiplying by 255
    EXPECT_EQ(b, expectedBlue);
}

void testFileStreamed(const Options& filterOps, StringList dimNames,
    uint16_t expectedRed, uint16_t expectedGreen, uint16_t expectedBlue)
{
    Options readerOps;
    readerOps.add("filename",
        Support::datapath("autzen/autzen-point-format-3.las"));

    LasReader reader;
    reader.setOptions(readerOps);

    ColorizationFilter filter;
    filter.setOptions(filterOps);
    filter.setInput(reader);

    StreamCallbackFilter f2;
    f2.setInput(filter);

    FixedPointTable table(50);

    f2.prepare(table);

    PointLayoutPtr layout = table.layout();
    Dimension::Id id1 = layout->findDim(dimNames[0]);
    Dimension::Id id2 = layout->findDim(dimNames[1]);
    Dimension::Id id3 = layout->findDim(dimNames[2]);

    auto cb = [expectedRed, expectedGreen, expectedBlue, id1, id2, id3]
        (PointRef& point)
    {
        static bool done = false;

        if (done)
            return false;

        EXPECT_EQ(expectedRed, point.getFieldAs<uint16_t>(id1));
        EXPECT_EQ(expectedGreen, point.getFieldAs<uint16_t>(id2));
        EXPECT_EQ(expectedBlue, point.getFieldAs<uint16_t>(id3));
        done = true;

        return true;
    };

    f2.setCallback(cb);
    f2.execute(table);
}

} // unnamed namespace

// Test using the standard dimensions.
TEST(ColorizationFilterTest, test1)
{
    Options options;

    options.add("dimensions", "Red, Green,Blue::255  ");
    options.add("raster", Support::datapath("autzen/autzen.jpg"));

    StringList dims;
    dims.push_back("Red");
    dims.push_back("Green");
    dims.push_back("Blue");
    testFile(options, dims, 210, 205, 47175);
    testFileStreamed(options, dims, 210, 205, 47175);
}

// Allow use of default dimensions.
TEST(ColorizationFilterTest, test2)
{
    Options options;

    options.add("raster", Support::datapath("autzen/autzen.jpg"));

    StringList dims;
    dims.push_back("Red");
    dims.push_back("Green");
    dims.push_back("Blue");

    testFile(options, dims, 210, 205, 185);
}

// Check that dimension creation works.
TEST(ColorizationFilterTest, test3)
{
    Options options;

    options.add("dimensions", "Foo:1,Bar_:2,Baz2:3:255");
    options.add("raster", Support::datapath("autzen/autzen.jpg"));

    StringList dims;
    dims.push_back("Foo");
    dims.push_back("Bar_");
    dims.push_back("Baz2");
    testFile(options, dims, 210, 205, 47175);
}

// Check that dimension creation works.
TEST(ColorizationFilterTest, test4)
{
    Options options;

    options.add("dimensions", "Foo&:1,Bar:2,Baz:3:255");
    options.add("raster", Support::datapath("autzen/autzen.jpg"));

    StringList dims;
    dims.push_back("Foo");
    dims.push_back("Bar");
    dims.push_back("Baz");
    EXPECT_THROW(testFile(options, dims, 210, 205, 47175), pdal_error);
}

// Check input point count is equal to output point count when there are input points outside the
// raster area and we are running in stream mode.
TEST(ColorizationFilterTest, test5)
{
    // setup las reader.
    Options readerOps;
    readerOps.add("filename", Support::datapath("autzen/autzen-point-format-3.las"));
    LasReader reader;
    reader.setOptions(readerOps);

    // setup transformation filter to translate points away from input raster image by about half
    // of its bounding box size in positive x,y and z directions. this is so that we have points
    // inside the raster and outside the raster area. translating by the z direction isn't really
    // necessary since it shouldn't effect the colourisation output, but I did it for fun.
    Options transformationFilterOps;
    transformationFilterOps.add("matrix", "1 0 0 1600 0 1 0 2200 0 0 1 60 0 0 0 1");
    TransformationFilter transformationFilter;
    transformationFilter.setOptions(transformationFilterOps);
    transformationFilter.setInput(reader);

    // setup colourisation filter.
    Options colourisationFilterOps;
    colourisationFilterOps.add("raster", Support::datapath("autzen/autzen.jpg"));
    ColorizationFilter colourisationFilter;
    colourisationFilter.setOptions(colourisationFilterOps);
    colourisationFilter.setInput(transformationFilter);

    // setup stream callback filter to count streamed output points.
    point_count_t pointCount{0};
    StreamCallbackFilter streamCallbackFilter;
    streamCallbackFilter.setCallback([&pointCount](PointRef& point)
    {
        ++pointCount;
        return false;
    });
    streamCallbackFilter.setInput(colourisationFilter);

    FixedPointTable table{50};
    streamCallbackFilter.prepare(table);
    streamCallbackFilter.execute(table);

    // expect input points that were translated out of the raster image area are not filtered out.
    EXPECT_NE(pointCount, 23u);
    EXPECT_EQ(pointCount, 106u);
}