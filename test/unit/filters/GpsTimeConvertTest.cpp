/******************************************************************************
 * Copyright (c) 2021, Preston J. Hartzell (preston.hartzell@gmail.com)
 *
 * All rights reserved
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

#include <io/BufferReader.hpp>
#include <filters/GpsTimeConvert.hpp>

namespace pdal
{

void checkTime(PointViewPtr view, int index, double expected)
{
    double actual = view->getFieldAs<double>(Dimension::Id::GpsTime, index);
    EXPECT_DOUBLE_EQ(expected, actual);
}

TEST(gws2gtTest, HandlesWrappedWeekSeconds)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::GpsTime});

    PointViewPtr inView(new PointView(table));
    inView->setField(Id::GpsTime, 0, 604799.5);
    inView->setField(Id::GpsTime, 1, 0.5);

    BufferReader reader;
    reader.addView(inView);

    Options options;
    options.add("conversion", "gws2gt");
    options.add("start_date", "2020-12-12");
    options.add("wrapped", true);

    GpsTimeConvert filter;
    filter.setOptions(options);
    filter.setInput(reader);
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    checkTime(outView, 0, 1291852799.5);
    checkTime(outView, 1, 1291852800.5);
}

TEST(gws2gtTest, HandlesUnwrappedWeekSeconds)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::GpsTime});

    PointViewPtr view(new PointView(table));
    view->setField(Id::GpsTime, 0, 604799.5);
    view->setField(Id::GpsTime, 1, 604800.5);

    BufferReader reader;
    reader.addView(view);

    Options options;
    options.add("conversion", "gws2gt");
    options.add("start_date", "2020-12-12");
    options.add("wrapped", false);

    GpsTimeConvert filter;
    filter.setOptions(options);
    filter.setInput(reader);
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    checkTime(outView, 0, 1291852799.5);
    checkTime(outView, 1, 1291852800.5);
}

TEST(gws2gstTest, HandlesWrappedWeekSeconds)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::GpsTime});

    PointViewPtr inView(new PointView(table));
    inView->setField(Id::GpsTime, 0, 604799.5);
    inView->setField(Id::GpsTime, 1, 0.5);

    BufferReader reader;
    reader.addView(inView);

    Options options;
    options.add("conversion", "gws2gst");
    options.add("start_date", "2020-12-12");
    options.add("wrapped", true);

    GpsTimeConvert filter;
    filter.setOptions(options);
    filter.setInput(reader);
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    checkTime(outView, 0, 291852799.5);
    checkTime(outView, 1, 291852800.5);
}

TEST(gws2gstTest, HandlesUnwrappedWeekSeconds)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::GpsTime});

    PointViewPtr view(new PointView(table));
    view->setField(Id::GpsTime, 0, 604799.5);
    view->setField(Id::GpsTime, 1, 604800.5);

    BufferReader reader;
    reader.addView(view);

    Options options;
    options.add("conversion", "gws2gst");
    options.add("start_date", "2020-12-12");
    options.add("wrapped", false);

    GpsTimeConvert filter;
    filter.setOptions(options);
    filter.setInput(reader);
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    checkTime(outView, 0, 291852799.5);
    checkTime(outView, 1, 291852800.5);
}

TEST(gt2gwsTest, WrapsWeekSeconds)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::GpsTime});

    PointViewPtr view(new PointView(table));
    view->setField(Id::GpsTime, 0, 1291852799.5);
    view->setField(Id::GpsTime, 1, 1291852800.5);

    BufferReader reader;
    reader.addView(view);

    Options options;
    options.add("conversion", "gt2gws");
    options.add("wrap", true);

    GpsTimeConvert filter;
    filter.setOptions(options);
    filter.setInput(reader);
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    checkTime(outView, 0, 604799.5);
    checkTime(outView, 1, 0.5);
}

TEST(gt2gwsTest, DoesNotWrapWeekSeconds)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::GpsTime});

    PointViewPtr view(new PointView(table));
    view->setField(Id::GpsTime, 0, 1291852799.5);
    view->setField(Id::GpsTime, 1, 1291852800.5);

    BufferReader reader;
    reader.addView(view);

    Options options;
    options.add("conversion", "gt2gws");
    options.add("wrap", false);

    GpsTimeConvert filter;
    filter.setOptions(options);
    filter.setInput(reader);
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    checkTime(outView, 0, 604799.5);
    checkTime(outView, 1, 604800.5);
}

TEST(gst2gwsTest, WrapsWeekSeconds)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::GpsTime});

    PointViewPtr view(new PointView(table));
    view->setField(Id::GpsTime, 0, 291852799.5);
    view->setField(Id::GpsTime, 1, 291852800.5);

    BufferReader reader;
    reader.addView(view);

    Options options;
    options.add("conversion", "gst2gws");
    options.add("wrap", true);

    GpsTimeConvert filter;
    filter.setOptions(options);
    filter.setInput(reader);
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    checkTime(outView, 0, 604799.5);
    checkTime(outView, 1, 0.5);
}

TEST(gst2gwsTest, DoesNotWrapWeekSeconds)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::GpsTime});

    PointViewPtr view(new PointView(table));
    view->setField(Id::GpsTime, 0, 291852799.5);
    view->setField(Id::GpsTime, 1, 291852800.5);

    BufferReader reader;
    reader.addView(view);

    Options options;
    options.add("conversion", "gst2gws");
    options.add("wrap", false);

    GpsTimeConvert filter;
    filter.setOptions(options);
    filter.setInput(reader);
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    checkTime(outView, 0, 604799.5);
    checkTime(outView, 1, 604800.5);
}

TEST(gt2gstTest, ToGpsStandardTime)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::GpsTime});

    PointViewPtr view(new PointView(table));
    view->setField(Id::GpsTime, 0, 1291852799.5);
    view->setField(Id::GpsTime, 1, 1291852800.5);

    BufferReader reader;
    reader.addView(view);

    Options options;
    options.add("conversion", "gt2gst");

    GpsTimeConvert filter;
    filter.setOptions(options);
    filter.setInput(reader);
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    checkTime(outView, 0, 291852799.5);
    checkTime(outView, 1, 291852800.5);
}

TEST(gst2gtTest, FromGpsStandardTime)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::GpsTime});

    PointViewPtr view(new PointView(table));
    view->setField(Id::GpsTime, 0, 291852799.5);
    view->setField(Id::GpsTime, 1, 291852800.5);

    BufferReader reader;
    reader.addView(view);

    Options options;
    options.add("conversion", "gst2gt");

    GpsTimeConvert filter;
    filter.setOptions(options);
    filter.setInput(reader);
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    checkTime(outView, 0, 1291852799.5);
    checkTime(outView, 1, 1291852800.5);
}

} // namespace pdal
