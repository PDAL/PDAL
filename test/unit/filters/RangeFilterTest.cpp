/******************************************************************************
* Copyright (c) 2015, Bradley J Chambers (brad.chambers@gmail.com)
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
#include <io/LasReader.hpp>
#include <io/TextReader.hpp>
#include <filters/RangeFilter.hpp>
#include <filters/StreamCallbackFilter.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(RangeFilterTest, createStage)
{
    StageFactory f;
    Stage* filter(f.createStage("filters.range"));
    EXPECT_TRUE(filter);
}

TEST(RangeFilterTest, noLimits)
{
    RangeFilter filter;

    PointTable table;
    EXPECT_THROW(filter.prepare(table), pdal_error);
}

TEST(RangeFilterTest, singleDimension)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("limits", "Z(3.50:6]  ");

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(3u, view->size());
    EXPECT_DOUBLE_EQ(4.0, view->getFieldAs<double>(Dimension::Id::Z, 0));
    EXPECT_DOUBLE_EQ(5.0, view->getFieldAs<double>(Dimension::Id::Z, 1));
    EXPECT_DOUBLE_EQ(6.0, view->getFieldAs<double>(Dimension::Id::Z, 2));
}

TEST(RangeFilterTest, multipleDimensions)
{
    BOX3D srcBounds(0.0, 1.0, 1.0, 0.0, 10.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("limits", "Y[4.00e0:+6]");
    rangeOps.add("limits", "Z[4:6]");

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(3u, view->size());
    EXPECT_DOUBLE_EQ(4.0, view->getFieldAs<double>(Dimension::Id::Y, 0));
    EXPECT_DOUBLE_EQ(5.0, view->getFieldAs<double>(Dimension::Id::Y, 1));
    EXPECT_DOUBLE_EQ(6.0, view->getFieldAs<double>(Dimension::Id::Y, 2));
    EXPECT_DOUBLE_EQ(4.0, view->getFieldAs<double>(Dimension::Id::Z, 0));
    EXPECT_DOUBLE_EQ(5.0, view->getFieldAs<double>(Dimension::Id::Z, 1));
    EXPECT_DOUBLE_EQ(6.0, view->getFieldAs<double>(Dimension::Id::Z, 2));
}

TEST(RangeFilterTest, multipleDimsBusted)
{
    BOX3D srcBounds(1, 3, 5, 1, 3, 5);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 1);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps1;
    rangeOps1.add("limits", "X[1:1], Y[27:27]");

    RangeFilter f1;
    f1.setOptions(rangeOps1);
    f1.setInput(reader);

    PointTable t1;
    f1.prepare(t1);
    PointViewSet s1 = f1.execute(t1);
    PointViewPtr v1 = *s1.begin();

    Options rangeOps2;
    rangeOps2.add("limits", "Y[27:27], X[1:1]");

    RangeFilter f2;
    f2.setOptions(rangeOps2);
    f2.setInput(reader);

    PointTable t2;
    f2.prepare(t2);
    PointViewSet s2 = f2.execute(t2);
    PointViewPtr v2 = *s2.begin();

    EXPECT_EQ(v1->size(), v2->size());
}

TEST(RangeFilterTest, onlyMin)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("limits", "Z[6:]");

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(5u, view->size());
    EXPECT_DOUBLE_EQ(6.0, view->getFieldAs<double>(Dimension::Id::Z, 0));
    EXPECT_DOUBLE_EQ(7.0, view->getFieldAs<double>(Dimension::Id::Z, 1));
    EXPECT_DOUBLE_EQ(8.0, view->getFieldAs<double>(Dimension::Id::Z, 2));
    EXPECT_DOUBLE_EQ(9.0, view->getFieldAs<double>(Dimension::Id::Z, 3));
    EXPECT_DOUBLE_EQ(10.0, view->getFieldAs<double>(Dimension::Id::Z, 4));
}

TEST(RangeFilterTest, onlyMax)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 10);

    StageFactory f;
    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("limits", "Z[:5]");

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(5u, view->size());
    EXPECT_DOUBLE_EQ(1.0, view->getFieldAs<double>(Dimension::Id::Z, 0));
    EXPECT_DOUBLE_EQ(2.0, view->getFieldAs<double>(Dimension::Id::Z, 1));
    EXPECT_DOUBLE_EQ(3.0, view->getFieldAs<double>(Dimension::Id::Z, 2));
    EXPECT_DOUBLE_EQ(4.0, view->getFieldAs<double>(Dimension::Id::Z, 3));
    EXPECT_DOUBLE_EQ(5.0, view->getFieldAs<double>(Dimension::Id::Z, 4));
}

TEST(RangeFilterTest, negation)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 10);

    StageFactory f;
    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("limits", "Z![2:5]");

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(6u, view->size());
    EXPECT_DOUBLE_EQ(1.0, view->getFieldAs<double>(Dimension::Id::Z, 0));
    EXPECT_DOUBLE_EQ(6.0, view->getFieldAs<double>(Dimension::Id::Z, 1));
    EXPECT_DOUBLE_EQ(7.0, view->getFieldAs<double>(Dimension::Id::Z, 2));
    EXPECT_DOUBLE_EQ(8.0, view->getFieldAs<double>(Dimension::Id::Z, 3));
    EXPECT_DOUBLE_EQ(9.0, view->getFieldAs<double>(Dimension::Id::Z, 4));
    EXPECT_DOUBLE_EQ(10.0, view->getFieldAs<double>(Dimension::Id::Z, 5));
}

TEST(RangeFilterTest, equals)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("limits", "Z[5:5]");

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(1u, view->size());
    EXPECT_DOUBLE_EQ(5.0, view->getFieldAs<double>(Dimension::Id::Z, 0));
}

TEST(RangeFilterTest, negativeValues)
{
    BOX3D srcBounds(0.0, 0.0, -10.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 21);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("limits", "Z[-1:1)");

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(2u, view->size());
    EXPECT_DOUBLE_EQ(-1.0, view->getFieldAs<double>(Dimension::Id::Z, 0));
    EXPECT_DOUBLE_EQ(0.0, view->getFieldAs<double>(Dimension::Id::Z, 1));
}

TEST(RangeFilterTest, simple_logic)
{

    Options ops;
    ops.add("bounds", BOX3D(1, 101, 201, 10, 110, 210));
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("limits", "Y[108:109], X[2:5], Z[1:1000], X[7:9], Y[103:105]");

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(5u, view->size());
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::X, 0), 3);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::X, 1), 4);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::X, 2), 5);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::X, 3), 8);
    EXPECT_EQ(view->getFieldAs<int>(Dimension::Id::X, 4), 9);
}

// Make sure that dimension names containing digits works
TEST(RangeFilterTest, case_1659)
{
    TextReader reader;

    Options ops;
    ops.add("filename", Support::datapath("text/numeric_dim.txt"));
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("limits", "Eigenvalue0[:35]");

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(3u, view->size());
}

TEST(RangeFilterTest, stream_logic)
{
    Options ops;
    ops.add("bounds", BOX3D(1, 101, 201, 10, 110, 210));
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("limits", "Y[108:109], X[2:5], Z[1:1000], X[7:9], Y[103:105]");

    RangeFilter range;
    range.setOptions(rangeOps);
    range.setInput(reader);

    StreamCallbackFilter f;
    f.setInput(range);

    FixedPointTable table(20);
    f.prepare(table);

    auto cb = [](PointRef& point)
    {
        static int i = 0;
        int x = point.getFieldAs<int>(Dimension::Id::X);
        if (i == 0)
            EXPECT_EQ(x, 3);
        else if (i == 1)
            EXPECT_EQ(x, 4);
        else if (i == 2)
            EXPECT_EQ(x, 5);
        else if (i == 3)
            EXPECT_EQ(x, 8);
        else if (i == 4)
            EXPECT_EQ(x, 9);
        EXPECT_TRUE(i < 5);
        ++i;
        return true;
    };
    f.setCallback(cb);

    f.execute(table);
}

TEST(RangeFilterTest, nan)
{
    LasReader reader;

    Options options;
    options.add("filename", Support::datapath("las/gps-time-nan.las"));
    reader.setOptions(options);

    Options rangeOptions;
    rangeOptions.add("limits", "GpsTime[-1:1]");

    RangeFilter filter;
    filter.setOptions(rangeOptions);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(0u, view->size());
}

