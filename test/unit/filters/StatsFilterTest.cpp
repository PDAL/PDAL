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

#include <random>

#include <pdal/PDALUtils.hpp>
#include <pdal/StageFactory.hpp>
#include <filters/StatsFilter.hpp>
#include <io/BufferReader.hpp>
#include <io/FauxReader.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(Stats, handcalc)
{
    PointTable table;
    table.layout()->registerDim(Dimension::Id::X);
    PointViewPtr v(new PointView(table));
    v->setField(Dimension::Id::X, 0, 1);
    v->setField(Dimension::Id::X, 1, 5);
    v->setField(Dimension::Id::X, 2, 8);
    v->setField(Dimension::Id::X, 3, 25);
    v->setField(Dimension::Id::X, 4, 20);
    v->setField(Dimension::Id::X, 5, 17);

    BufferReader r;
    r.addView(v);
    StatsFilter f;
    f.setInput(r);
    Options opts;
    opts.add("advanced", true);
    f.setOptions(opts);

    f.prepare(table);
    f.execute(table);

    const stats::Summary& xstats = f.getStats(Dimension::Id::X);

    EXPECT_EQ(xstats.minimum(), 1.0);
    EXPECT_EQ(xstats.maximum(), 25.0);
    EXPECT_DOUBLE_EQ(xstats.average(), 12.0 + 2.0 / 3.0);
    EXPECT_DOUBLE_EQ(xstats.sampleVariance(), 88.2 + .2 / 3);
    EXPECT_DOUBLE_EQ(xstats.sampleStddev(), std::sqrt(88.2 + .2 / 3));
    EXPECT_NEAR(xstats.populationSkewness(), 0.05589204039, 0.000000001);
    EXPECT_NEAR(xstats.sampleSkewness(), 0.07653332827, 0.000000001);

    EXPECT_NEAR(xstats.populationKurtosis(), 1.504435885, 0.000000001);
    EXPECT_NEAR(xstats.populationExcessKurtosis(), -1.495564114, 0.000000001);
    EXPECT_NEAR(xstats.sampleKurtosis(), 4.387937998, 0.000000001);
    EXPECT_NEAR(xstats.sampleExcessKurtosis(), -1.862062002, 0.000000001);
}


TEST(Stats, baseline)
{
    PointTable table;
    table.layout()->registerDim(Dimension::Id::X);
    PointViewPtr v(new PointView(table));

    for (PointId idx = 0; idx < 100; idx++)
        v->setField(Dimension::Id::X, idx, 55.2);

    BufferReader r;
    r.addView(v);
    StatsFilter f;
    f.setInput(r);
    Options opts;
    opts.add("advanced", true);
    f.setOptions(opts);

    f.prepare(table);
    f.execute(table);

    const stats::Summary& xstats = f.getStats(Dimension::Id::X);
    EXPECT_EQ(xstats.minimum(), 55.2);
    EXPECT_EQ(xstats.maximum(), 55.2);
    EXPECT_EQ(xstats.average(), 55.2);
    EXPECT_EQ(xstats.sampleVariance(), 0.0);
    EXPECT_EQ(xstats.populationVariance(), 0.0);
    EXPECT_EQ(xstats.sampleSkewness(), 0.0);
    EXPECT_EQ(xstats.populationSkewness(), 0.0);
    EXPECT_EQ(xstats.sampleKurtosis(), 0.0);
    EXPECT_EQ(xstats.sampleExcessKurtosis(), 0.0);
    EXPECT_EQ(xstats.populationKurtosis(), 0.0);
    EXPECT_EQ(xstats.populationExcessKurtosis(), 0.0);
}


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

    EXPECT_DOUBLE_EQ(statsX.minimum(), 1.0);
    EXPECT_DOUBLE_EQ(statsY.minimum(), 2.0);
    EXPECT_DOUBLE_EQ(statsZ.minimum(), 3.0);

    EXPECT_DOUBLE_EQ(statsX.maximum(), 101.0);
    EXPECT_DOUBLE_EQ(statsY.maximum(), 102.0);
    EXPECT_DOUBLE_EQ(statsZ.maximum(), 103.0);

    EXPECT_DOUBLE_EQ(statsX.average(), 51.0);
    EXPECT_DOUBLE_EQ(statsY.average(), 52.0);
    EXPECT_DOUBLE_EQ(statsZ.average(), 53.0);

    EXPECT_NEAR(statsX.sampleVariance(), 835.8375, .0001);
    EXPECT_NEAR(statsY.sampleVariance(), 835.8375, .0001);
    EXPECT_NEAR(statsZ.sampleVariance(), 835.8375, .0001);

    EXPECT_DOUBLE_EQ(statsX.sampleSkewness(), 0.0);
    EXPECT_DOUBLE_EQ(statsY.sampleSkewness(), 0.0);
    EXPECT_DOUBLE_EQ(statsZ.sampleSkewness(), 0.0);

    EXPECT_DOUBLE_EQ(statsX.sampleExcessKurtosis(), 0.0);
    EXPECT_DOUBLE_EQ(statsY.sampleExcessKurtosis(), 0.0);
    EXPECT_DOUBLE_EQ(statsZ.sampleExcessKurtosis(), 0.0);
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

    EXPECT_NEAR(statsX.sampleSkewness(), -5.2235397e-16, 1e-23);
    EXPECT_NEAR(statsY.sampleSkewness(), -5.7098153e-16, 1e-23);
    EXPECT_NEAR(statsZ.sampleSkewness(), -5.5176534e-16, 1e-23);

    EXPECT_NEAR(statsX.sampleExcessKurtosis(), -1.2, .00001);
    EXPECT_NEAR(statsY.sampleExcessKurtosis(), -1.2, .00001);
    EXPECT_NEAR(statsZ.sampleExcessKurtosis(), -1.2, .00001);
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

    EXPECT_DOUBLE_EQ(statsX.minimum(), 1.0);
    EXPECT_DOUBLE_EQ(statsY.minimum(), 2.0);
    EXPECT_DOUBLE_EQ(statsZ.minimum(), 3.0);

    EXPECT_DOUBLE_EQ(statsX.maximum(), 1.0);
    EXPECT_DOUBLE_EQ(statsY.maximum(), 2.0);
    EXPECT_DOUBLE_EQ(statsZ.maximum(), 3.0);

    EXPECT_DOUBLE_EQ(statsX.average(), 1.0);
    EXPECT_DOUBLE_EQ(statsY.average(), 2.0);
    EXPECT_DOUBLE_EQ(statsZ.average(), 3.0);
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

    EXPECT_DOUBLE_EQ(statsX.minimum(), 1.0);
    EXPECT_DOUBLE_EQ(statsZ.minimum(), 3.0);

    EXPECT_DOUBLE_EQ(statsX.maximum(), 1.0);
    EXPECT_DOUBLE_EQ(statsZ.maximum(), 3.0);

    EXPECT_DOUBLE_EQ(statsX.average(), 1.0);
    EXPECT_DOUBLE_EQ(statsZ.average(), 3.0);
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

TEST(Stats, merge)
{
    std::mt19937 gen(314159);
    {
        std::uniform_real_distribution<double> dis(0, 100000);
        using SummaryPtr = std::unique_ptr<stats::Summary>;
        std::array<SummaryPtr, 10> parts;

        for (SummaryPtr& part : parts)
            part.reset(new stats::Summary("test", stats::Summary::NoEnum, true));
        stats::Summary whole("test", stats::Summary::NoEnum, true);

        stats::Summary* part = parts[0].get();
        for (size_t i = 0; i < 10000; ++i)
        {
            stats::Summary& part = *(parts[i / 1000].get());

            double d = dis(gen);
            whole.insert(d);
            part.insert(d);
        }

        for (size_t i = 1; i < 10; ++i)
            parts[0]->merge(*parts[i]);

        stats::Summary& p = *parts[0];

        EXPECT_DOUBLE_EQ(whole.minimum(), p.minimum());
        EXPECT_DOUBLE_EQ(whole.maximum(), p.maximum());
        EXPECT_FLOAT_EQ((float)whole.average(), (float)p.average());
        EXPECT_FLOAT_EQ((float)whole.populationVariance(), (float)p.populationVariance());
        EXPECT_FLOAT_EQ((float)whole.skewness(), (float)p.skewness());
        EXPECT_FLOAT_EQ((float)whole.kurtosis(), (float)p.kurtosis());
    }

    {
        std::uniform_int_distribution<int> dis(0, 100);
        using SummaryPtr = std::unique_ptr<stats::Summary>;
        std::array<SummaryPtr, 10> parts;

        for (SummaryPtr& part : parts)
            part.reset(new stats::Summary("test", stats::Summary::Enumerate, false));
        stats::Summary whole("test", stats::Summary::Enumerate, false);

        stats::Summary* part = parts[0].get();
        for (size_t i = 0; i < 10000; ++i)
        {
            stats::Summary& part = *(parts[i / 1000].get());

            double d = dis(gen);
            whole.insert(d);
            part.insert(d);
        }

        for (size_t i = 1; i < 10; ++i)
            parts[0]->merge(*parts[i]);

        stats::Summary& p = *parts[0];

        stats::Summary::EnumMap wm = whole.values();
        stats::Summary::EnumMap pm = p.values();
        EXPECT_EQ(wm.size(), pm.size());
        for (size_t i = 0; i < 100; ++i)
            EXPECT_EQ(wm[(double)i], pm[(double)i]);
    }
}
