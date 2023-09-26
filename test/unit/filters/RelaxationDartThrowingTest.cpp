/******************************************************************************
* Copyright (c) 2023, Bradley J Chambers (brad.chambers@gmail.com)
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
#include <filters/RelaxationDartThrowing.hpp>
#include <filters/StreamCallbackFilter.hpp>

using namespace pdal;

TEST(RelaxationDartThrowingTest, create)
{
    StageFactory f;
    Stage* filter(f.createStage("filters.relaxationdartthrowing"));
    EXPECT_TRUE(filter);
}

TEST(RelaxationDartThrowingTest, RelaxationDartThrowingTest_smallInput)
{
    BOX3D srcBounds(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "uniform");
    ops.add("count", 30);
    FauxReader reader;
    reader.setOptions(ops);

    Options RelaxationDartThrowingOps;
    RelaxationDartThrowingOps.add("count", 40);

    RelaxationDartThrowing filter;
    filter.setOptions(RelaxationDartThrowingOps);
    filter.setInput(reader);

    PointTable table;

    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 30u);
}

TEST(RelaxationDartThrowingTest, RelaxationDartThrowingTest_success)
{
    BOX3D srcBounds(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "uniform");
    ops.add("count", 30);
    FauxReader reader;
    reader.setOptions(ops);

    Options RelaxationDartThrowingOps;
    RelaxationDartThrowingOps.add("count", 15);

    RelaxationDartThrowing filter;
    filter.setOptions(RelaxationDartThrowingOps);
    filter.setInput(reader);

    PointTable table;

    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 15u);
}

TEST(RelaxationDartThrowingTest, RelaxationDartThrowingTest_earlyTermination)
{
    BOX3D srcBounds(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "constant");
    ops.add("count", 30);
    FauxReader reader;
    reader.setOptions(ops);

    Options RelaxationDartThrowingOps;
    RelaxationDartThrowingOps.add("count", 15);

    RelaxationDartThrowing filter;
    filter.setOptions(RelaxationDartThrowingOps);
    filter.setInput(reader);

    PointTable table;

    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 1u);
}

