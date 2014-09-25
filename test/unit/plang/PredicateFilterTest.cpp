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

#include <pdal/filters/Predicate.hpp>
#include <pdal/filters/Stats.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>

#include "../StageTester.hpp"
#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(PredicateFilterTest)

using namespace pdal;

BOOST_AUTO_TEST_CASE(PredicateFilterTest_test1)
{
    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOps;
    readerOps.add("bounds", bounds);
    readerOps.add("num_points", 1000);
    readerOps.add("mode", "ramp");
    drivers::faux::Reader reader(readerOps);

    // keep all points where x less than 1.0
    const pdal::Option source("source",
        // "X < 1.0"
        "import numpy as np\n"
        "def yow1(ins,outs):\n"
        "  X = ins['X']\n"
        "  Mask = np.less(X, 1.0)\n"
        "  #print X\n"
        "  #print Mask\n"
        "  outs['Mask'] = Mask\n"
        "  return True\n"
    );
    const Option module("module", "MyModule1");
    const Option function("function", "yow1");
    Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);

    filters::Predicate filter(opts);
    filter.setInput(&reader);
    BOOST_CHECK(filter.getDescription() == "Predicate Filter");

    Options statOpts;
    filters::Stats stats(statOpts);
    stats.setInput(&filter);

    PointContext ctx;

    stats.prepare(ctx);
    PointBufferSet pbSet = stats.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);

    const filters::stats::Summary& statsX = stats.getStats(Dimension::Id::X);
    const filters::stats::Summary& statsY = stats.getStats(Dimension::Id::Y);
    const filters::stats::Summary& statsZ = stats.getStats(Dimension::Id::Z);

    BOOST_CHECK(Utils::compare_approx<double>(statsX.minimum(), 0.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsY.minimum(), 0.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsZ.minimum(), 0.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsX.maximum(), 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsY.maximum(), 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsZ.maximum(), 1.0, 0.01));
}


BOOST_AUTO_TEST_CASE(PredicateFilterTest_test2)
{
    // same as above, but with 'Y >' instead of 'X <'

    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOps;
    readerOps.add("bounds", bounds);
    readerOps.add("num_points", 1000);
    readerOps.add("mode", "ramp");

    drivers::faux::Reader reader(readerOps);

    Option source("source",
        // "Y > 1.0"
        "import numpy as np\n"
        "def yow2(ins,outs):\n"
        "  Y = ins['Y']\n"
        "  Mask = np.greater(Y, 1.0)\n"
        "  #print Mask\n"
        "  outs['Mask'] = Mask\n"
        "  return True\n"
    );
    Option module("module", "MyModule1");
    Option function("function", "yow2");
    Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);

    filters::Predicate filter(opts);
    filter.setInput(&reader);
    BOOST_CHECK(filter.getDescription() == "Predicate Filter");

    Options statOpts;
    filters::Stats stats(statOpts);
    stats.setInput(&filter);

    PointContext ctx;

    stats.prepare(ctx);
    PointBufferSet pbSet = stats.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);

    const filters::stats::Summary& statsX = stats.getStats(Dimension::Id::X);
    const filters::stats::Summary& statsY = stats.getStats(Dimension::Id::Y);
    const filters::stats::Summary& statsZ = stats.getStats(Dimension::Id::Z);

    BOOST_CHECK(Utils::compare_approx<double>(statsX.minimum(), 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsY.minimum(), 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsZ.minimum(), 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsX.maximum(), 2.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsY.maximum(), 2.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsZ.maximum(), 2.0, 0.01));
}

BOOST_AUTO_TEST_CASE(PredicateFilterTest_test3)
{
    // can we make a pipeline with TWO python filters in it?

    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOpts;
    readerOpts.add("bounds", bounds);
    readerOpts.add("num_points", 1000);
    readerOpts.add("mode", "ramp");
    drivers::faux::Reader reader(readerOpts);

    // keep all points where x less than 1.0
    const Option source1("source",
        // "X < 1.0"
        "import numpy as np\n"
        "def yow1(ins,outs):\n"
        "  X = ins['X']\n"
        "  Mask = np.less(X, 1.0)\n"
        "  #print X\n"
        "  #print Mask\n"
        "  outs['Mask'] = Mask\n"
        "  return True\n"
    );
    const Option module1("module", "MyModule1");
    const Option function1("function", "yow1");
    Options opts1;
    opts1.add(source1);
    opts1.add(module1);
    opts1.add(function1);

    filters::Predicate filter1(opts1);
    filter1.setInput(&reader);

    // keep all points where y greater than 0.5
    const Option source2("source",
        // "Y > 0.5"
        "import numpy as np\n"
        "def yow2(ins,outs):\n"
        "  Y = ins['Y']\n"
        "  Mask = np.greater(Y, 0.5)\n"
        "  #print X\n"
        "  #print Mask\n"
        "  outs['Mask'] = Mask\n"
        "  return True\n"
    );
    const Option module2("module", "MyModule2");
    const Option function2("function", "yow2");
    Options opts2;
    opts2.add(source2);
    opts2.add(module2);
    opts2.add(function2);

    filters::Predicate filter2(opts2);
    filter2.setInput(&filter1);

    Options statOpts;
    filters::Stats stats(statOpts);
    stats.setInput(&filter2);

    PointContext ctx;
    stats.prepare(ctx);
    stats.execute(ctx);

    const filters::stats::Summary& statsX = stats.getStats(Dimension::Id::X);
    const filters::stats::Summary& statsY = stats.getStats(Dimension::Id::Y);
    const filters::stats::Summary& statsZ = stats.getStats(Dimension::Id::Z);

    BOOST_CHECK(Utils::compare_approx<double>(statsX.minimum(), 0.5, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsY.minimum(), 0.5, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsZ.minimum(), 0.5, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsX.maximum(), 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsY.maximum(), 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(statsZ.maximum(), 1.0, 0.01));
}

BOOST_AUTO_TEST_CASE(PredicateFilterTest_test4)
{
    // test the point counters in the Predicate's iterator

    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOpts;
    readerOpts.add("bounds", bounds);
    readerOpts.add("num_points", 1000);
    readerOpts.add("mode", "ramp");
    drivers::faux::Reader reader(readerOpts);

    const Option source("source",
        // "Y > 0.5"
        "import numpy as np\n"
        "def yow2(ins,outs):\n"
        "  Y = ins['Y']\n"
        "  Mask = np.greater(Y, 0.5)\n"
        "  #print Mask\n"
        "  outs['Mask'] = Mask\n"
        "  return True\n"
    );
    const Option module("module", "MyModule1");
    const Option function("function", "yow2");
    Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);

    filters::Predicate filter(opts);
    filter.setInput(&reader);

    PointContext ctx;
    PointBufferPtr buf(new PointBuffer(ctx));

    filter.prepare(ctx);

    StageTester::ready(&reader, ctx);
    PointBufferSet pbSet = StageTester::run(&reader, buf);
    StageTester::done(&reader, ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 1000);

    StageTester::ready(&filter, ctx);
    pbSet = StageTester::run(&filter, buf);
    StageTester::done(&filter, ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 750);
}

BOOST_AUTO_TEST_CASE(PredicateFilterTest_test5)
{
    // test error handling if missing Mask

    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOpts;
    readerOpts.add("bounds", bounds);
    readerOpts.add("num_points", 1000);
    readerOpts.add("mode", "ramp");
    drivers::faux::Reader reader(readerOpts);

    const Option source("source",
        // "Y > 0.5"
        "import numpy as np\n"
        "def yow2(ins,outs):\n"
        "  Y = ins['Y']\n"
        "  Mask = np.greater(Y, 0.5)\n"
        "  #print Mask\n"
        "  outs['xxxMaskxxx'] = Mask # delierbately rong\n"
        "  return True\n"
    );
    const Option module("module", "MyModule1");
    const Option function("function", "yow2");
    Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);

    filters::Predicate filter(opts);
    filter.setInput(&reader);

    PointContext ctx;
    filter.prepare(ctx);

    BOOST_REQUIRE_THROW(filter.execute(ctx), pdal::python_error);
}

BOOST_AUTO_TEST_CASE(PredicateFilterTest_Pipeline)
{
    PipelineManager mgr;
    PipelineReader reader(mgr);

    reader.readPipeline(Support::datapath("plang/from-module.xml"));
    point_count_t cnt = mgr.execute();
    BOOST_CHECK_EQUAL(cnt, 1);
}

BOOST_AUTO_TEST_CASE(PredicateFilterTest_Embed)
{
    PipelineManager mgr;
    PipelineReader reader(mgr);

    reader.readPipeline(Support::datapath("plang/predicate-embed.xml"));
    point_count_t cnt = mgr.execute();
    BOOST_CHECK_EQUAL(cnt, 1);
}

BOOST_AUTO_TEST_SUITE_END()

#endif  // PDAL_HAVE_PYTHON
