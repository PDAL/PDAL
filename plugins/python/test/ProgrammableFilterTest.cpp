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

#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <stats/StatsFilter.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(ProgrammableFilterTest, ProgrammableFilterTest_test1)
{
    StageFactory f;

    BOX3D bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    Options ops;
    ops.add("bounds", bounds);
    ops.add("num_points", 10);
    ops.add("mode", "ramp");

    std::unique_ptr<Stage> reader(f.createStage("readers.faux"));
    reader->setOptions(ops);

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
    filter->setInput(reader.get());

    StatsFilter stats;
    stats.setInput(filter.get());

    PointContext ctx;

    stats.prepare(ctx);
    PointBufferSet pbSet = stats.execute(ctx);
    EXPECT_EQ(pbSet.size(), 1u);
    PointBufferPtr buf = *pbSet.begin();

    const stats::Summary& statsX = stats.getStats(Dimension::Id::X);
    const stats::Summary& statsY = stats.getStats(Dimension::Id::Y);
    const stats::Summary& statsZ = stats.getStats(Dimension::Id::Z);

    EXPECT_FLOAT_EQ(statsX.minimum(), 10.0);
    EXPECT_FLOAT_EQ(statsX.maximum(), 11.0);

    EXPECT_FLOAT_EQ(statsY.minimum(), 0.0);
    EXPECT_FLOAT_EQ(statsY.maximum(), 1.0);

    EXPECT_FLOAT_EQ(statsZ.minimum(), 3.14);
    EXPECT_FLOAT_EQ(statsZ.maximum(), 3.14);
}

TEST(ProgrammableFilterTest, pipeline)
{
    PipelineManager manager;
    PipelineReader reader(manager);

    reader.readPipeline(Support::configuredpath("plang/programmable-update-y-dims.xml"));
    manager.execute();
    PointBufferSet pbSet = manager.buffers();
    EXPECT_EQ(pbSet.size(), 1u);
    PointBufferPtr buf = *pbSet.begin();

    for (PointId idx = 0; idx < 10; ++idx)
    {
        int32_t y = buf->getFieldAs<int32_t>(Dimension::Id::Y, idx);
        EXPECT_EQ(y, 314);
    }
}


TEST(ProgrammableFilterTest, add_dimension)
{
    StageFactory f;

    BOX3D bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    Options ops;
    ops.add("bounds", bounds);
    ops.add("num_points", 10);
    ops.add("mode", "ramp");

    std::unique_ptr<Stage> reader(f.createStage("readers.faux"));
    reader->setOptions(ops);

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
    filter->setInput(reader.get());

    PointContext ctx;
    filter->prepare(ctx);
    PointBufferSet pbSet = filter->execute(ctx);
    EXPECT_EQ(pbSet.size(), 1u);
    PointBufferPtr buf = *pbSet.begin();

    pdal::Dimension::Id::Enum int_id = ctx.findDim("AddedIntensity");
    pdal::Dimension::Id::Enum psid_id = ctx.findDim("AddedPointSourceId");

    for (unsigned int i = 0; i < buf->size(); ++i)
    {
        EXPECT_EQ(buf->getFieldAs<uint16_t>(int_id, i), 1);
        EXPECT_EQ(buf->getFieldAs<uint16_t>(psid_id, i), 2);
    }
}
