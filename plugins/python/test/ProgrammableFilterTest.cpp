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

#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <stats/StatsFilter.hpp>
#include <faux/FauxReader.hpp>

#include "Support.hpp"

using namespace pdal;
#include <pdal/plang/Environment.hpp>

class ProgrammableFilterTest : public ::testing::Test
{
public:
    virtual void SetUp()
    {
        pdal::plang::Environment::get();
    }

};
TEST_F(ProgrammableFilterTest, ProgrammableFilterTest_test1)
{
    StageFactory f;

    BOX3D bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    Options ops;
    ops.add("bounds", bounds);
    ops.add("num_points", 10);
    ops.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(ops);

    Option source("source", "import numpy as np\n"
        "def myfunc(ins,outs):\n"
        "  X = ins['X']\n"
        "  Y = ins['Y']\n"
        "  Z = ins['Z']\n"
        "  #print ins['X']\n"
        "  X = X + 10.0\n"
        "  # Y: leave as-is, don't export back out\n"
        "  # Z: goofiness to make it a numpy array of a constant\n"
        "  Z = np.zeros(X.size) + 3.14\n"
        "  outs['X'] = X\n"
        "  #print outs['X']\n"
        "  outs['Z'] = Z\n"
        "  return True\n"
    );
    Option module("module", "MyModule");
    Option function("function", "myfunc");
    Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);

    std::unique_ptr<Stage> filter(f.createStage("filters.programmable"));
    filter->setOptions(opts);
    filter->setInput(reader);

    std::unique_ptr<StatsFilter> stats(new StatsFilter);
    stats->setInput(*filter);

    PointTable table;

    stats->prepare(table);
    PointViewSet viewSet = stats->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    const stats::Summary& statsX = stats->getStats(Dimension::Id::X);
    const stats::Summary& statsY = stats->getStats(Dimension::Id::Y);
    const stats::Summary& statsZ = stats->getStats(Dimension::Id::Z);

    EXPECT_DOUBLE_EQ(statsX.minimum(), 10.0);
    EXPECT_DOUBLE_EQ(statsX.maximum(), 11.0);

    EXPECT_DOUBLE_EQ(statsY.minimum(), 0.0);
    EXPECT_DOUBLE_EQ(statsY.maximum(), 1.0);

    EXPECT_DOUBLE_EQ(statsZ.minimum(), 3.14);
    EXPECT_DOUBLE_EQ(statsZ.maximum(), 3.14);
}

TEST_F(ProgrammableFilterTest, pipeline)
{
    PipelineManager manager;
    PipelineReader reader(manager);

    reader.readPipeline(
        Support::configuredpath("plang/programmable-update-y-dims.xml"));
    manager.execute();
    PointViewSet viewSet = manager.views();
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    for (PointId idx = 0; idx < 10; ++idx)
    {
        int32_t y = view->getFieldAs<int32_t>(Dimension::Id::Y, idx);
        EXPECT_EQ(y, 314);
    }
}


TEST_F(ProgrammableFilterTest, add_dimension)
{
    StageFactory f;

    BOX3D bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    Options ops;
    ops.add("bounds", bounds);
    ops.add("num_points", 10);
    ops.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(ops);

    Option source("source", "import numpy\n"
        "def myfunc(ins,outs):\n"
        "  outs['AddedIntensity'] = np.zeros(ins['X'].size, dtype=numpy.double) + 1\n"
        "  outs['AddedPointSourceId'] = np.zeros(ins['X'].size, dtype=numpy.double) + 2\n"
        "  return True\n"
    );
    Option module("module", "MyModule");
    Option function("function", "myfunc");
    Option intensity("add_dimension", "AddedIntensity");
    Option scanDirection("add_dimension", "AddedPointSourceId");
    Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);
    opts.add(intensity);
    opts.add(scanDirection);

    std::unique_ptr<Stage> filter(f.createStage("filters.programmable"));
    filter->setOptions(opts);
    filter->setInput(reader);

    PointTable table;
    filter->prepare(table);
    PointViewSet viewSet = filter->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    PointLayoutPtr layout(table.layout());

    Dimension::Id::Enum int_id = layout->findDim("AddedIntensity");
    Dimension::Id::Enum psid_id = layout->findDim("AddedPointSourceId");

    for (unsigned int i = 0; i < view->size(); ++i)
    {
        EXPECT_EQ(view->getFieldAs<uint16_t>(int_id, i), 1);
        EXPECT_EQ(view->getFieldAs<uint16_t>(psid_id, i), 2);
    }
}


TEST_F(ProgrammableFilterTest, metadata)
{
    StageFactory f;

    BOX3D bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    Options ops;
    ops.add("bounds", bounds);
    ops.add("num_points", 10);
    ops.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(ops);

    Option source("source", "import numpy\n"
        "def myfunc(ins,outs,inmeta,outmeta):\n"
        "  t = ('name', 'value', '', '', [])\n"
        "  outmeta.append(t)\n"
        "  return True\n"
    );
    Option module("module", "MyModule");
    Option function("function", "myfunc");
    Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);

    std::unique_ptr<Stage> filter(f.createStage("filters.programmable"));
    filter->setOptions(opts);
    filter->setInput(reader);

    PointTable table;
    filter->prepare(table);
    PointViewSet viewSet = filter->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    PointLayoutPtr layout(table.layout());
    MetadataNode m = table.metadata();
    m = m.findChild("filters.programmable");
    MetadataNodeList l = m.children();
    EXPECT_EQ(l.size(), 1u);
//     EXPECT_EQ(l[0].name(), "name");
//     EXPECT_EQ(l[0].value(), "value");
}
