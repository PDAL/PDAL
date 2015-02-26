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

#include <pdal/StageFactory.hpp>
#include <StatsFilter.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(StatsFilterTest, simple)
{
    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 1000);
    ops.add("mode", "constant");

    StageFactory f;
    std::unique_ptr<Stage> reader(f.createStage("readers.faux"));
    EXPECT_TRUE(reader.get());
    reader->setOptions(ops);

    StatsFilter filter;
    filter.setInput(reader.get());
    EXPECT_EQ(filter.getName(), "filters.stats");

    PointContext ctx;
    filter.prepare(ctx);
    filter.execute(ctx);

    const stats::Summary& statsX = filter.getStats(Dimension::Id::X);
    const stats::Summary& statsY = filter.getStats(Dimension::Id::Y);
    const stats::Summary& statsZ = filter.getStats(Dimension::Id::Z);

    EXPECT_EQ(statsX.count(), 1000u);
    EXPECT_EQ(statsY.count(), 1000u);
    EXPECT_EQ(statsZ.count(), 1000u);

    EXPECT_FLOAT_EQ(statsX.minimum(), 1.0);
    EXPECT_FLOAT_EQ(statsY.minimum(), 2.0);
    EXPECT_FLOAT_EQ(statsZ.minimum(), 3.0);

    EXPECT_FLOAT_EQ(statsX.maximum(), 1.0);
    EXPECT_FLOAT_EQ(statsY.maximum(), 2.0);
    EXPECT_FLOAT_EQ(statsZ.maximum(), 3.0);

    EXPECT_FLOAT_EQ(statsX.average(), 1.0);
    EXPECT_FLOAT_EQ(statsY.average(), 2.0);
    EXPECT_FLOAT_EQ(statsZ.average(), 3.0);
}

TEST(StatsFilterTest, dimset)
{
    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 1000);
    ops.add("mode", "constant");

    StageFactory f;
    std::unique_ptr<Stage> reader(f.createStage("readers.faux"));
    EXPECT_TRUE(reader.get());
    reader->setOptions(ops);

    Options filterOps;
    filterOps.add("dimensions", " , X, Z ");
    StatsFilter filter;
    filter.setInput(reader.get());
    filter.setOptions(filterOps);
    EXPECT_EQ(filter.getName(), "filters.stats");

    PointContext ctx;
    filter.prepare(ctx);
    filter.execute(ctx);

    const stats::Summary& statsX = filter.getStats(Dimension::Id::X);
    EXPECT_THROW(filter.getStats(Dimension::Id::Y), pdal_error);
    const stats::Summary& statsZ = filter.getStats(Dimension::Id::Z);

    EXPECT_EQ(statsX.count(), 1000u);
    EXPECT_EQ(statsZ.count(), 1000u);

    EXPECT_FLOAT_EQ(statsX.minimum(), 1.0);
    EXPECT_FLOAT_EQ(statsZ.minimum(), 3.0);

    EXPECT_FLOAT_EQ(statsX.maximum(), 1.0);
    EXPECT_FLOAT_EQ(statsZ.maximum(), 3.0);

    EXPECT_FLOAT_EQ(statsX.average(), 1.0);
    EXPECT_FLOAT_EQ(statsZ.average(), 3.0);
}


TEST(StatsFilterTest, metadata)
{
    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 1000);
    ops.add("mode", "constant");

    StageFactory f;
    std::unique_ptr<Stage> reader(f.createStage("readers.faux"));
    EXPECT_TRUE(reader.get());
    reader->setOptions(ops);

    Options filterOps;
    filterOps.add("dimensions", " , X, Z ");
    StatsFilter filter;
    filter.setInput(reader.get());
    filter.setOptions(filterOps);

    PointContext ctx;
    filter.prepare(ctx);
    filter.execute(ctx);
    MetadataNode m = filter.getMetadata();
    std::vector<MetadataNode> children = m.children("statistic");

    auto findNode = [](MetadataNode m,
        const std::string name, const std::string val)
    {
        auto findNameVal = [name, val](MetadataNode m)
            { return (m.name() == name && m.value() == val); };

        return m.find(findNameVal);
    };
 

    for (auto mi = children.begin(); mi != children.end(); ++mi)
    {
        if (findNode(*mi, "name", "X").valid())
        {
            EXPECT_EQ(mi->findChild("average").value(), "1");
            EXPECT_EQ(mi->findChild("minimum").value(), "1");
            EXPECT_EQ(mi->findChild("maximum").value(), "1");
            EXPECT_EQ(mi->findChild("count").value(), "1000");
        }
        if (findNode(*mi, "name", "Z").valid())
        {
            EXPECT_EQ(mi->findChild("average").value(), "3");
            EXPECT_EQ(mi->findChild("minimum").value(), "3");
            EXPECT_EQ(mi->findChild("maximum").value(), "3");
            EXPECT_EQ(mi->findChild("count").value(), "1000");
        }
    }
}

