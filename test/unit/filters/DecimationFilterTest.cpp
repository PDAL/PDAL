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
#include <pdal/StageFactory.hpp>
#include <io/FauxReader.hpp>
#include <filters/DecimationFilter.hpp>
#include <filters/StreamCallbackFilter.hpp>

using namespace pdal;

TEST(DecimationFilterTest, create)
{
    StageFactory f;
    Stage* filter(f.createStage("filters.decimation"));
    EXPECT_TRUE(filter);
}

TEST(DecimationFilterTest, DecimationFilterTest_test1)
{
    BOX3D srcBounds(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "random");
    ops.add("count", 30);
    FauxReader reader;
    reader.setOptions(ops);

    Options decimationOps;
    decimationOps.add("step", 10);

    DecimationFilter filter;
    filter.setOptions(decimationOps);
    filter.setInput(reader);

    PointTable table;

    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 3u);

    uint64_t t0 = view->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, 0);
    uint64_t t1 = view->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, 1);
    uint64_t t2 = view->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, 2);

    EXPECT_EQ(t0, 0u);
    EXPECT_EQ(t1, 10u);
    EXPECT_EQ(t2, 20u);
}

TEST(DecimationFilterTest, stream)
{
    BOX3D srcBounds(0.0, 0.0, 0.0, 99.0, 99.0, 99.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 100);
    FauxReader reader;
    reader.setOptions(ops);

    Options decimationOps;
    decimationOps.add("step", 10);
    decimationOps.add("offset", 10);
    decimationOps.add("limit", 90);

    DecimationFilter dec;
    dec.setOptions(decimationOps);
    dec.setInput(reader);

    StreamCallbackFilter filter;

    auto cb = [](PointRef& point)
    {
        static int i = 0;
        int x = point.getFieldAs<int>(Dimension::Id::X);
        int y = point.getFieldAs<int>(Dimension::Id::Y);
        EXPECT_EQ(x, (i + 1) * 10);
        EXPECT_EQ(y, (i + 1) * 10);
        EXPECT_TRUE(i < 8);
        i++;
        return true;
    };
    filter.setCallback(cb);
    filter.setInput(dec);

    FixedPointTable t(2);

    filter.prepare(t);
    filter.execute(t);
}

