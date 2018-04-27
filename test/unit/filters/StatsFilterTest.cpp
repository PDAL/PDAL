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

#include <pdal/PDALUtils.hpp>
#include <pdal/StageFactory.hpp>
#include <filters/StatsFilter.hpp>
#include <io/FauxReader.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(Stats, simple)
{
    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 1000);
    ops.add("mode", "ramp");

    StageFactory f;

    Stage* reader(f.createStage("readers.faux"));
    EXPECT_TRUE(reader);
    reader->setOptions(ops);

    StatsFilter filter;
    filter.setInput(*reader);
    EXPECT_EQ(filter.getName(), "filters.stats");

    PointTable table;
    filter.prepare(table);
    filter.execute(table);

    const stats::Summary& statsX = filter.getStats(Dimension::Id::X);
    const stats::Summary& statsY = filter.getStats(Dimension::Id::Y);
    const stats::Summary& statsZ = filter.getStats(Dimension::Id::Z);

    EXPECT_EQ(statsX.count(), 1000u);
    EXPECT_EQ(statsY.count(), 1000u);
    EXPECT_EQ(statsZ.count(), 1000u);

    EXPECT_FLOAT_EQ(statsX.minimum(), 1.0);
    EXPECT_FLOAT_EQ(statsY.minimum(), 2.0);
    EXPECT_FLOAT_EQ(statsZ.minimum(), 3.0);

    EXPECT_FLOAT_EQ(statsX.maximum(), 101.0);
    EXPECT_FLOAT_EQ(statsY.maximum(), 102.0);
    EXPECT_FLOAT_EQ(statsZ.maximum(), 103.0);

    EXPECT_FLOAT_EQ(statsX.average(), 51.0);
    EXPECT_FLOAT_EQ(statsY.average(), 52.0);
    EXPECT_FLOAT_EQ(statsZ.average(), 53.0);

    EXPECT_FLOAT_EQ(statsX.variance(), 837.09351);
    EXPECT_FLOAT_EQ(statsY.variance(), 837.0965);
    EXPECT_FLOAT_EQ(statsZ.variance(), 837.1015);

    EXPECT_DOUBLE_EQ(statsX.skewness(), 0.0);
    EXPECT_DOUBLE_EQ(statsY.skewness(), 0.0);
    EXPECT_DOUBLE_EQ(statsZ.skewness(), 0.0);

    EXPECT_DOUBLE_EQ(statsX.kurtosis(), 0.0);
    EXPECT_DOUBLE_EQ(statsY.kurtosis(), 0.0);
    EXPECT_DOUBLE_EQ(statsZ.kurtosis(), 0.0);
}

TEST(Stats, advanced)
{
    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 1000);
    ops.add("mode", "ramp");

    StageFactory f;

    Stage* reader(f.createStage("readers.faux"));
    EXPECT_TRUE(reader);
    reader->setOptions(ops);

    StatsFilter filter;
    Options so;
    so.add("advanced", true);

    filter.setInput(*reader);
    filter.setOptions(so);
    EXPECT_EQ(filter.getName(), "filters.stats");

    PointTable table;
    filter.prepare(table);
    filter.execute(table);

    const stats::Summary& statsX = filter.getStats(Dimension::Id::X);
    const stats::Summary& statsY = filter.getStats(Dimension::Id::Y);
    const stats::Summary& statsZ = filter.getStats(Dimension::Id::Z);

    EXPECT_EQ(statsX.count(), 1000u);
    EXPECT_EQ(statsY.count(), 1000u);
    EXPECT_EQ(statsZ.count(), 1000u);

    EXPECT_NEAR(statsX.skewness(), 7.6279972e+11, 10000);
    EXPECT_NEAR(statsY.skewness(), 6.1023649e+12, 100000);
    EXPECT_NEAR(statsZ.skewness(), 2.0595297e+13, 1000000);

    EXPECT_NEAR(statsX.kurtosis(), -527558696e+4, 10000);
    EXPECT_NEAR(statsY.kurtosis(), -422043928e+5, 100000);
    EXPECT_NEAR(statsZ.kurtosis(), -142438122e+6, 1000000);
}


TEST(Stats, stream)
{
    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 1000);
    ops.add("mode", "constant");

    StageFactory f;

    Stage* reader(f.createStage("readers.faux"));
    EXPECT_TRUE(reader);
    reader->setOptions(ops);

    StatsFilter filter;
    filter.setInput(*reader);
    EXPECT_EQ(filter.getName(), "filters.stats");

    FixedPointTable table(100);
    filter.prepare(table);
    filter.execute(table);

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


TEST(Stats, dimset)
{
    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 1000);
    ops.add("mode", "constant");

    StageFactory f;
    Stage* reader(f.createStage("readers.faux"));
    EXPECT_TRUE(reader);
    reader->setOptions(ops);

    Options filterOps;
    filterOps.add("dimensions", " , X, Z ");
    StatsFilter filter;
    filter.setInput(*reader);
    filter.setOptions(filterOps);
    EXPECT_EQ(filter.getName(), "filters.stats");

    PointTable table;
    filter.prepare(table);
    filter.execute(table);

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


TEST(Stats, metadata)
{
    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 1000);
    ops.add("mode", "constant");

    StageFactory f;
    Stage* reader(f.createStage("readers.faux"));
    EXPECT_TRUE(reader);
    reader->setOptions(ops);

    Options filterOps;
    filterOps.add("dimensions", " , X, Z ");
    StatsFilter filter;
    filter.setInput(*reader);
    filter.setOptions(filterOps);

    PointTable table;
    filter.prepare(table);
    filter.execute(table);
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
            EXPECT_DOUBLE_EQ(mi->findChild("average").value<double>(), 1.0);
            EXPECT_DOUBLE_EQ(mi->findChild("minimum").value<double>(), 1.0);
            EXPECT_DOUBLE_EQ(mi->findChild("maximum").value<double>(), 1.0);
            EXPECT_DOUBLE_EQ(mi->findChild("count").value<double>(), 1000.0);
        }
        if (findNode(*mi, "name", "Z").valid())
        {
            EXPECT_DOUBLE_EQ(mi->findChild("average").value<double>(), 3.0);
            EXPECT_DOUBLE_EQ(mi->findChild("minimum").value<double>(), 3.0);
            EXPECT_DOUBLE_EQ(mi->findChild("maximum").value<double>(), 3.0);
            EXPECT_DOUBLE_EQ(mi->findChild("count").value<double>(), 1000.0);
        }
    }
}


TEST(Stats, enum)
{
    BOX3D bounds(1.0, 0.0, 0.0, 10.0, 100.0, 1000.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 10);
    ops.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(ops);

    Options filterOps;
    filterOps.add("dimensions", "X, Y, Z");
    filterOps.add("enumerate", "X");
    filterOps.add("count", "Y");

    StatsFilter filter;
    filter.setInput(reader);
    filter.setOptions(filterOps);

    PointTable table;
    filter.prepare(table);
    filter.execute(table);

    const stats::Summary& statsX = filter.getStats(Dimension::Id::X);
    const stats::Summary::EnumMap& values = statsX.values();
    EXPECT_EQ(values.size(), 10U);
    double d = 1.0;
    for (auto& v : values)
    {
        EXPECT_DOUBLE_EQ(d, v.first);
        d += 1.0;
    }

    const stats::Summary& statsY = filter.getStats(Dimension::Id::Y);
    const stats::Summary::EnumMap& yValues = statsY.values();
    d = 0.0;
    for (auto& v : yValues)
    {
        EXPECT_DOUBLE_EQ(d, v.first);
        EXPECT_EQ(1u, v.second);
        d += (100.0 / 9);
    }
}

TEST(Stats, global)
{
    BOX3D bounds(1.0, 0.0, 0.0, 10.0, 100.0, 1000.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 10);
    ops.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(ops);

    Options filterOps;
    filterOps.add("dimensions", "X, Y, Z");
    filterOps.add("global", "Z, Y, X");
    filterOps.add("count", "Y");

    StatsFilter filter;
    filter.setInput(reader);
    filter.setOptions(filterOps);

    PointTable table;
    filter.prepare(table);
    filter.execute(table);

    const stats::Summary& statsZ = filter.getStats(Dimension::Id::Z);

    EXPECT_DOUBLE_EQ(statsZ.median(), 555.55555555555554);
	EXPECT_DOUBLE_EQ(statsZ.mad(), 333.33333333333331);
	EXPECT_DOUBLE_EQ(statsZ.minimum(), 0.0);
	EXPECT_DOUBLE_EQ(statsZ.maximum(), 1000.0);

}
