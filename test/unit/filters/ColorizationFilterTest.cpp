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

#include <LasReader.hpp>
#include <ColorizationFilter.hpp>
#include <StreamCallbackFilter.hpp>
#include <pdal/PointView.hpp>

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
    Dimension::Id::Enum id1 = layout->findDim(dimNames[0]);
    Dimension::Id::Enum id2 = layout->findDim(dimNames[1]);
    Dimension::Id::Enum id3 = layout->findDim(dimNames[2]);

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


