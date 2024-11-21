/******************************************************************************
* Copyright (c) 2023, Howard Butler (info@hobu.co)
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
#include <filters/ExpressionFilter.hpp>
#include <filters/StreamCallbackFilter.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(ExpressionFilterTest, createStage)
{
    StageFactory f;
    Stage* filter(f.createStage("filters.expression"));
    EXPECT_TRUE(filter);
}

TEST(ExpressionFilterTest, noLimits)
{
    ExpressionFilter filter;

    PointTable table;
    EXPECT_THROW(filter.prepare(table), pdal_error);
}

TEST(ExpressionFilterTest, singleDimension)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("expression", "Z > 3.5 && Z <= 6");

    ExpressionFilter filter;
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

TEST(ExpressionFilterTest, multipleDimensions)
{
    BOX3D srcBounds(0.0, 1.0, 1.0, 0.0, 10.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("expression", "Y >= 4.0 && Y <= 6 && Z >= 4 && Z <= 6");

    ExpressionFilter filter;
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
TEST(ExpressionFilterTest, onlyMin)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("expression", "Z >= 6.0");

    ExpressionFilter filter;
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

TEST(ExpressionFilterTest, onlyMax)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);
//
    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 10);

    StageFactory f;
    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("expression", "Z <= 5.0");

    ExpressionFilter filter;
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

TEST(ExpressionFilterTest, negation)
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
    rangeOps.add("expression", "!(Z >=2 && Z <=5)");
//     rangeOps.add("limits", "Z![2:5]");

    ExpressionFilter filter;
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

TEST(ExpressionFilterTest, equals)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("expression", "Z == 5.0");
//     rangeOps.add("limits", "Z[5:5]");

    ExpressionFilter filter;
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

TEST(ExpressionFilterTest, negativeValues)
{
    BOX3D srcBounds(0.0, 0.0, -10.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 21);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("expression", "Z >= -1 && Z < 1");

    ExpressionFilter filter;
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

TEST(ExpressionFilterTest, NaNs)
{
    BOX3D srcBounds(0.0, 0.0, -10.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "invalid");
    ops.add("count", 21);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("expression", "OffsetTime == nan");

    ExpressionFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(3u, view->size());
    EXPECT_DOUBLE_EQ(std::numeric_limits<double>::quiet_NaN(), view->getFieldAs<double>(Dimension::Id::OffsetTime, 1));
}

TEST(ExpressionFilterTest, simple_logic)
{

    Options ops;
    ops.add("bounds", BOX3D(1, 101, 201, 10, 110, 210));
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
//     rangeOps.add("limits", "Y[108:109], X[2:5], Z[1:1000], X[7:9], Y[103:105]");
    rangeOps.add("expression", "((Y >= 108 && Y <= 109) || (Y >= 103 && Y <= 105)) && ((X >= 2 && X <= 5) || (X >= 7 && X <=9 )) && (Z >=1 && Z <= 1000) ");

    ExpressionFilter filter;
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
TEST(ExpressionFilterTest, case_1659)
{
    TextReader reader;

    Options ops;
    ops.add("filename", Support::datapath("text/numeric_dim.txt"));
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("expression", "Eigenvalue0 <= 35");

    ExpressionFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(3u, view->size());
}

TEST(ExpressionFilterTest, stream_logic)
{
    Options ops;
    ops.add("bounds", BOX3D(1, 101, 201, 10, 110, 210));
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
//     rangeOps.add("limits", "Y[108:109], X[2:5], Z[1:1000], X[7:9], Y[103:105]");
    rangeOps.add("expression", "((Y >= 108 && Y <= 109) || (Y >= 103 && Y <= 105)) && ((X >= 2 && X <= 5) || (X >= 7 && X <=9 )) && (Z >=1 && Z <= 1000) ");

    ExpressionFilter range;
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

TEST(ExpressionFilterTest, nan)
{
    LasReader reader;

    Options options;
    options.add("filename", Support::datapath("las/gps-time-nan.las"));
    reader.setOptions(options);

    Options rangeOptions;
    rangeOptions.add("expression", "GpsTime >= -1.0 && GpsTime <= 1");

    ExpressionFilter filter;
    filter.setOptions(rangeOptions);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(0u, view->size());
}

TEST(ExpressionFilterTest, nan2)
{
    LasReader reader;

    Options options;
    options.add("filename", Support::datapath("las/gps-time-nan.las"));
    reader.setOptions(options);

    Options rangeOptions;
    rangeOptions.add("expression", "GpsTime == nan()");

    ExpressionFilter filter;
    filter.setOptions(rangeOptions);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(0u, view->size());
}

TEST(ExpressionFilterTest, multipleExpressions)
{
    BOX3D srcBounds(0.0, 1.0, 1.0, 0.0, 10.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options rangeOps;
    rangeOps.add("expression", "Y >= 4.0");
    rangeOps.add("expression", "Y <= 6.0");
    rangeOps.add("expression", "Z >= 4.0");
    rangeOps.add("expression", "Z <= 6.0");

    ExpressionFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(4u, viewSet.size());

    static int total_cnt = 0;
    std::vector<PointViewPtr> views;
    for (auto v : viewSet)
    {
        views.push_back(v);
    }

    EXPECT_EQ(7u, views[0]->size());
    EXPECT_EQ(6u, views[1]->size());
    EXPECT_EQ(7u, views[2]->size());
    EXPECT_EQ(6u, views[3]->size());
}
