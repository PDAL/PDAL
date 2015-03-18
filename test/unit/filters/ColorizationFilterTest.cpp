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

#include "gtest/gtest.h"

#include <LasReader.hpp>
#include <ColorizationFilter.hpp>
#include <pdal/PointView.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(ColorizationFilterTest, ColorizationFilterTest_test_1)
{
    Options ops1;
    ops1.add("filename", Support::datapath("autzen/autzen-point-format-3.las"));
    LasReader reader;
    reader.setOptions(ops1);

    Options options;

    Option red("dimension", "Red", "");
    Option b0("band",1, "");
    Option s0("scale", 1.0f, "scale factor for this dimension");
    Options redO;
    redO.add(b0);
    redO.add(s0);
    red.setOptions(redO);

    Option green("dimension", "Green", "");
    Option b1("band",2, "");
    Option s1("scale", 1.0f, "scale factor for this dimension");
    Options greenO;
    greenO.add(b1);
    greenO.add(s1);
    green.setOptions(greenO);

    Option blue("dimension", "Blue", "");
    Option b2("band",3, "");
    Option s2("scale", 255.0f, "scale factor for this dimension");
    Options blueO;
    blueO.add(b2);
    blueO.add(s2);
    blue.setOptions(blueO);

    Option datasource("raster", Support::datapath("autzen/autzen.jpg"),
        "raster to read");

    Options reader_options;
    reader_options.add(red);
    reader_options.add(green);
    reader_options.add(blue);
    reader_options.add(datasource);

    ColorizationFilter filter;
    filter.setOptions(reader_options);
    filter.setInput(reader);

    PointTable table;

    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    uint16_t r = view->getFieldAs<uint16_t>(Dimension::Id::Red, 0);
    uint16_t g = view->getFieldAs<uint16_t>(Dimension::Id::Green, 0);
    uint16_t b = view->getFieldAs<uint16_t>(Dimension::Id::Blue, 0);

    EXPECT_EQ(r, 210u);
    EXPECT_EQ(g, 205u);
    // We scaled this up to 16bit by multiplying by 255
    EXPECT_EQ(b, 47175u);
}
