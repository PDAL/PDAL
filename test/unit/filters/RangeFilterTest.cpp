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
#include <FauxReader.hpp>
#include <RangeFilter.hpp>

using namespace pdal;

TEST(RangeFilterTest, createStage)
{
    StageFactory f;
    std::shared_ptr<Stage> filter(f.createStage("filters.range"));
    EXPECT_TRUE(filter.get());
}

TEST(RangeFilterTest, noDimension)
{
    RangeFilter filter;

    PointTable table;
    EXPECT_THROW(filter.prepare(table), pdal_error);
}

TEST(RangeFilterTest, noRange)
{
    Options rangeOps;
    rangeOps.add("dimension", "Z");

    RangeFilter filter;
    filter.setOptions(rangeOps);

    PointTable table;
    EXPECT_THROW(filter.prepare(table), pdal_error);
}

TEST(RangeFilterTest, singleDimension)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("num_points", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options range;
    range.add("min", 4);
    range.add("max", 6);

    Option dim("dimension", "Z");
    dim.setOptions(range);

    Options rangeOps;
    rangeOps.add(dim);

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(3u, view->size());
    EXPECT_FLOAT_EQ(4.0, view->getFieldAs<double>(Dimension::Id::Z, 0));
    EXPECT_FLOAT_EQ(5.0, view->getFieldAs<double>(Dimension::Id::Z, 1));
    EXPECT_FLOAT_EQ(6.0, view->getFieldAs<double>(Dimension::Id::Z, 2));
}

TEST(RangeFilterTest, multipleDimensions)
{
    BOX3D srcBounds(0.0, 1.0, 1.0, 0.0, 10.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("num_points", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options y_range;
    y_range.add("min", 4);
    y_range.add("max", 6);

    Option y_dim("dimension", "Y");
    y_dim.setOptions(y_range);

    Options z_range;
    z_range.add("min", 4);
    z_range.add("max", 6);

    Option z_dim("dimension", "Z");
    z_dim.setOptions(z_range);

    Options rangeOps;
    rangeOps.add(y_dim);
    rangeOps.add(z_dim);

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(3u, view->size());
    EXPECT_FLOAT_EQ(4.0, view->getFieldAs<double>(Dimension::Id::Y, 0));
    EXPECT_FLOAT_EQ(5.0, view->getFieldAs<double>(Dimension::Id::Y, 1));
    EXPECT_FLOAT_EQ(6.0, view->getFieldAs<double>(Dimension::Id::Y, 2));
    EXPECT_FLOAT_EQ(4.0, view->getFieldAs<double>(Dimension::Id::Z, 0));
    EXPECT_FLOAT_EQ(5.0, view->getFieldAs<double>(Dimension::Id::Z, 1));
    EXPECT_FLOAT_EQ(6.0, view->getFieldAs<double>(Dimension::Id::Z, 2));
}

TEST(RangeFilterTest, onlyMin)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("num_points", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options range;
    range.add("min", 6);

    Option dim("dimension", "Z");
    dim.setOptions(range);

    Options rangeOps;
    rangeOps.add(dim);

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(5u, view->size());
    EXPECT_FLOAT_EQ(6.0, view->getFieldAs<double>(Dimension::Id::Z, 0));
    EXPECT_FLOAT_EQ(7.0, view->getFieldAs<double>(Dimension::Id::Z, 1));
    EXPECT_FLOAT_EQ(8.0, view->getFieldAs<double>(Dimension::Id::Z, 2));
    EXPECT_FLOAT_EQ(9.0, view->getFieldAs<double>(Dimension::Id::Z, 3));
    EXPECT_FLOAT_EQ(10.0, view->getFieldAs<double>(Dimension::Id::Z, 4));
}

TEST(RangeFilterTest, onlyMax)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("num_points", 10);

    StageFactory f;
    FauxReader reader;
    reader.setOptions(ops);

    Options range;
    range.add("max", 5);

    Option dim("dimension", "Z");
    dim.setOptions(range);

    Options rangeOps;
    rangeOps.add(dim);

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(5u, view->size());
    EXPECT_FLOAT_EQ(1.0, view->getFieldAs<double>(Dimension::Id::Z, 0));
    EXPECT_FLOAT_EQ(2.0, view->getFieldAs<double>(Dimension::Id::Z, 1));
    EXPECT_FLOAT_EQ(3.0, view->getFieldAs<double>(Dimension::Id::Z, 2));
    EXPECT_FLOAT_EQ(4.0, view->getFieldAs<double>(Dimension::Id::Z, 3));
    EXPECT_FLOAT_EQ(5.0, view->getFieldAs<double>(Dimension::Id::Z, 4));
}


TEST(RangeFilterTest, equals)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("num_points", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options range;
    range.add("equals", 5);

    Option dim("dimension", "Z");
    dim.setOptions(range);

    Options rangeOps;
    rangeOps.add(dim);

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(1u, view->size());
    EXPECT_FLOAT_EQ(5.0, view->getFieldAs<double>(Dimension::Id::Z, 0));
}

TEST(RangeFilterTest, negativeValues)
{
    BOX3D srcBounds(0.0, 0.0, -10.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("num_points", 21);

    FauxReader reader;
    reader.setOptions(ops);

    Options range;
    range.add("min", -1);
    range.add("max", 1);

    Option dim("dimension", "Z");
    dim.setOptions(range);

    Options rangeOps;
    rangeOps.add(dim);

    RangeFilter filter;
    filter.setOptions(rangeOps);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(3u, view->size());
    EXPECT_FLOAT_EQ(-1.0, view->getFieldAs<double>(Dimension::Id::Z, 0));
    EXPECT_FLOAT_EQ(0.0, view->getFieldAs<double>(Dimension::Id::Z, 1));
    EXPECT_FLOAT_EQ(1.0, view->getFieldAs<double>(Dimension::Id::Z, 2));
}

