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

#include <pdal/pdal_internal.hpp>
#ifdef PDAL_HAVE_PYTHON

#include <boost/test/unit_test.hpp>

#include <pdal/filters/Programmable.hpp>
#include <pdal/filters/Stats.hpp>
#include <pdal/drivers/faux/Reader.hpp>

#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(ProgrammableFilterTest)

using namespace pdal;

BOOST_AUTO_TEST_CASE(ProgrammableFilterTest_test1)
{
    Bounds<double> bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    Options ops;
    ops.add("bounds", bounds);
    ops.add("num_points", 10);
    ops.add("mode", "ramp");

    drivers::faux::Reader reader(ops);

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

    filters::Programmable filter(opts);
    filter.setInput(&reader);
    BOOST_CHECK(filter.getDescription() == "Programmable Filter");

    Options filterOps;
    filters::Stats stats(filterOps);
    stats.setInput(&filter);

    PointContext ctx;

    stats.prepare(ctx);
    PointBufferSet pbSet = stats.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();

    const filters::stats::Summary& statsX = stats.getStats(Dimension::Id::X);
    const filters::stats::Summary& statsY = stats.getStats(Dimension::Id::Y);
    const filters::stats::Summary& statsZ = stats.getStats(Dimension::Id::Z);

    BOOST_CHECK_CLOSE(statsX.minimum(), 10.0, 0.001);
    BOOST_CHECK_CLOSE(statsX.maximum(), 11.0, 0.001);

    BOOST_CHECK_CLOSE(statsY.minimum(), 0.0, 0.001);
    BOOST_CHECK_CLOSE(statsY.maximum(), 1.0, 0.001);

    BOOST_CHECK_CLOSE(statsZ.minimum(), 3.14, 0.001);
    BOOST_CHECK_CLOSE(statsZ.maximum(), 3.14, 0.001);
}

BOOST_AUTO_TEST_CASE(pipeline)
{
    PipelineManager manager;
    PipelineReader reader(manager);

    reader.readPipeline(Support::datapath("pipeline/create-dimension.xml"));
    manager.execute();
    PointBufferSet pbSet = manager.buffers();
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();

    for (PointId idx = 0; idx < 10; ++idx)
    {
        int32_t y = buf->getFieldAs<int32_t>(Dimension::Id::Y, idx);
        BOOST_CHECK_EQUAL(y, 314);
    }
}


BOOST_AUTO_TEST_CASE(add_dimension)
{
    Bounds<double> bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    Options ops;
    ops.add("bounds", bounds);
    ops.add("num_points", 10);
    ops.add("mode", "ramp");

    drivers::faux::Reader reader(ops);

    Option source("source", "import numpy\n"
        "def myfunc(ins,outs):\n"
        "  outs['Intensity'] = np.zeros(ins['X'].size, dtype=numpy.uint16) + 1\n"
        "  outs['PointSourceId'] = np.zeros(ins['X'].size, dtype=numpy.uint16) + 2\n"
        "  return True\n"
    );
    Option module("module", "MyModule");
    Option function("function", "myfunc");
    Option intensity("add_dimension", "Intensity");
    Option scanDirection("add_dimension", "PointSourceId");
    Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);
    opts.add(intensity);
    opts.add(scanDirection);

    filters::Programmable filter(opts);
    filter.setInput(&reader);

    PointContext ctx;
    filter.prepare(ctx);
    PointBufferSet pbSet = filter.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();

    for (unsigned int i = 0; i < buf->size(); ++i)
    {
        BOOST_CHECK_EQUAL(buf->getFieldAs<uint16_t>(Dimension::Id::Intensity, i), 1);
        BOOST_CHECK_EQUAL(buf->getFieldAs<uint16_t>(Dimension::Id::PointSourceId, i), 2);
    }
}


BOOST_AUTO_TEST_SUITE_END()

#endif  // PDAL_HAVE_PYTHON
