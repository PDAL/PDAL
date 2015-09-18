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

#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/StageWrapper.hpp>
#include <stats/StatsFilter.hpp>
#include <faux/FauxReader.hpp>


#include "Support.hpp"


using namespace pdal;

#include <pdal/plang/Environment.hpp>

class PredicateFilterTest : public ::testing::Test
{
public:
    virtual void SetUp()
    {
        pdal::plang::Environment::get();
    }

};

TEST_F(PredicateFilterTest, PredicateFilterTest_test1)
{
    StageFactory f;


    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOps;
    readerOps.add("bounds", bounds);
    readerOps.add("num_points", 1000);
    readerOps.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(readerOps);

    // keep all points where x less than 1.0
    const Option source("source",
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

    std::unique_ptr<Stage> filter(f.createStage("filters.predicate"));
    filter->setOptions(opts);
    filter->setInput(reader);

    Options statOpts;
    std::unique_ptr<StatsFilter> stats(new StatsFilter);
    stats->setOptions(statOpts);
    stats->setInput(*filter);

    PointTable table;

    stats->prepare(table);
    PointViewSet viewSet = stats->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);

    const stats::Summary& statsX = stats->getStats(Dimension::Id::X);
    const stats::Summary& statsY = stats->getStats(Dimension::Id::Y);
    const stats::Summary& statsZ = stats->getStats(Dimension::Id::Z);

    EXPECT_TRUE(Utils::compare_approx<double>(statsX.minimum(), 0.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsY.minimum(), 0.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsZ.minimum(), 0.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsX.maximum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsY.maximum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsZ.maximum(), 1.0, 0.01));
}

TEST_F(PredicateFilterTest, PredicateFilterTest_test2)
{
    StageFactory f;
    // same as above, but with 'Y >' instead of 'X <'

    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOps;
    readerOps.add("bounds", bounds);
    readerOps.add("num_points", 1000);
    readerOps.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(readerOps);

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

    std::unique_ptr<Stage> filter(f.createStage("filters.predicate"));
    filter->setOptions(opts);
    filter->setInput(reader);

    Options statOpts;
    std::unique_ptr<StatsFilter> stats(new StatsFilter);
    stats->setOptions(statOpts);
    stats->setInput(*filter);

    PointTable table;

    stats->prepare(table);
    PointViewSet viewSet = stats->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);

    const stats::Summary& statsX = stats->getStats(Dimension::Id::X);
    const stats::Summary& statsY = stats->getStats(Dimension::Id::Y);
    const stats::Summary& statsZ = stats->getStats(Dimension::Id::Z);

    EXPECT_TRUE(Utils::compare_approx<double>(statsX.minimum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsY.minimum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsZ.minimum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsX.maximum(), 2.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsY.maximum(), 2.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsZ.maximum(), 2.0, 0.01));
}

TEST_F(PredicateFilterTest, PredicateFilterTest_test3)
{
    StageFactory f;
    // can we make a pipeline with TWO python filters in it?

    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOpts;
    readerOpts.add("bounds", bounds);
    readerOpts.add("num_points", 1000);
    readerOpts.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(readerOpts);

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

    std::unique_ptr<Stage> filter1(f.createStage("filters.predicate"));
    filter1->setOptions(opts1);
    filter1->setInput(reader);

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

    std::unique_ptr<Stage> filter2(f.createStage("filters.predicate"));
    filter2->setOptions(opts2);
    filter2->setInput(*filter1);

    Options statOpts;
    std::unique_ptr<StatsFilter> stats(new StatsFilter);
    stats->setOptions(statOpts);
    stats->setInput(*filter2);

    PointTable table;
    stats->prepare(table);
    stats->execute(table);

    const stats::Summary& statsX = stats->getStats(Dimension::Id::X);
    const stats::Summary& statsY = stats->getStats(Dimension::Id::Y);
    const stats::Summary& statsZ = stats->getStats(Dimension::Id::Z);

    EXPECT_TRUE(Utils::compare_approx<double>(statsX.minimum(), 0.5, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsY.minimum(), 0.5, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsZ.minimum(), 0.5, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsX.maximum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsY.maximum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx<double>(statsZ.maximum(), 1.0, 0.01));
}

TEST_F(PredicateFilterTest, PredicateFilterTest_test4)
{
    StageFactory f;
    // test the point counters in the Predicate's iterator

    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOpts;
    readerOpts.add("bounds", bounds);
    readerOpts.add("num_points", 1000);
    readerOpts.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(readerOpts);

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

    std::unique_ptr<Stage> filter(f.createStage("filters.predicate"));
    filter->setOptions(opts);
    filter->setInput(reader);

    PointTable table;
    PointViewPtr buf(new PointView(table));

    filter->prepare(table);

    StageWrapper::ready(reader, table);
    PointViewSet viewSet = StageWrapper::run(reader, buf);
    StageWrapper::done(reader, table);
    EXPECT_EQ(viewSet.size(), 1u);
    buf = *viewSet.begin();
    EXPECT_EQ(buf->size(), 1000u);

    StageWrapper::ready(*filter, table);
    viewSet = StageWrapper::run(*filter, buf);
    StageWrapper::done(*filter, table);
    EXPECT_EQ(viewSet.size(), 1u);
    buf = *viewSet.begin();
    EXPECT_EQ(buf->size(), 750u);
}

TEST_F(PredicateFilterTest, PredicateFilterTest_test5)
{
    StageFactory f;
    // test error handling if missing Mask

    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOpts;
    readerOpts.add("bounds", bounds);
    readerOpts.add("num_points", 1000);
    readerOpts.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(readerOpts);

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

    std::unique_ptr<Stage> filter(f.createStage("filters.predicate"));
    filter->setOptions(opts);
    filter->setInput(reader);

    PointTable table;
    filter->prepare(table);

    ASSERT_THROW(filter->execute(table), pdal::pdal_error);
}

TEST_F(PredicateFilterTest, PredicateFilterTest_Pipeline)
{
    PipelineManager mgr;
    PipelineReader reader(mgr);

    reader.readPipeline(Support::configuredpath("plang/from-module.xml"));
    point_count_t cnt = mgr.execute();
    EXPECT_EQ(cnt, 1u);
}

TEST_F(PredicateFilterTest, PredicateFilterTest_Embed)
{
    PipelineManager mgr;
    PipelineReader reader(mgr);

    reader.readPipeline(Support::configuredpath("plang/predicate-embed.xml"));
    point_count_t cnt = mgr.execute();
    EXPECT_EQ(cnt, 1u);
}
