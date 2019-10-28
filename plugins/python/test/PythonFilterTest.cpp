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
#include <pdal/StageFactory.hpp>
#include <io/FauxReader.hpp>
#include <filters/StatsFilter.hpp>

#include "../plang/Invocation.hpp"
#include "../plang/Environment.hpp"

#include <pdal/StageWrapper.hpp>

#include "Support.hpp"

using namespace pdal;
using namespace pdal::plang;


class PythonFilterTest : public ::testing::Test
{
public:
    virtual void SetUp()
    {
        pdal::plang::Environment::get();
    }

};

TEST_F(PythonFilterTest, PythonFilterTest_test1)
{
    StageFactory f;

    BOX3D bounds1(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    BOX3D bounds2(10.0, 10.0, 10.0, 11.0, 11.0, 11.0);
    FauxReader reader1;
    FauxReader reader2;

    Options ops1;
    ops1.add("bounds", bounds1);
    ops1.add("count", 10);
    ops1.add("mode", "ramp");
    reader1.setOptions(ops1);

    Options ops2;
    ops2.add("bounds", bounds2);
    ops2.add("count", 10);
    ops2.add("mode", "ramp");
    reader2.setOptions(ops2);

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

    Stage* filter(f.createStage("filters.python"));
    filter->setOptions(opts);
    filter->setInput(reader1);
    filter->setInput(reader2);

    std::unique_ptr<StatsFilter> stats(new StatsFilter);
    stats->setInput(*filter);

    PointTable table;

    stats->prepare(table);
    PointViewSet viewSet = stats->execute(table);
    EXPECT_EQ(viewSet.size(), 2u);

    const stats::Summary& statsX = stats->getStats(Dimension::Id::X);
    const stats::Summary& statsY = stats->getStats(Dimension::Id::Y);
    const stats::Summary& statsZ = stats->getStats(Dimension::Id::Z);

    EXPECT_DOUBLE_EQ(statsX.minimum(), 10.0);
    EXPECT_DOUBLE_EQ(statsX.maximum(), 21.0);

    EXPECT_DOUBLE_EQ(statsY.minimum(), 0.0);
    EXPECT_DOUBLE_EQ(statsY.maximum(), 11.0);

    EXPECT_DOUBLE_EQ(statsZ.minimum(), 3.14);
    EXPECT_DOUBLE_EQ(statsZ.maximum(), 3.14);
}

TEST_F(PythonFilterTest, pipelineJSON)
{
    PipelineManager manager;

    manager.readPipeline(
        Support::configuredpath("plang/programmable-update-y-dims.json"));
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

TEST_F(PythonFilterTest, add_dimension)
{
    StageFactory f;

    BOX3D bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 10);
    ops.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(ops);

    Option source("source", "import numpy as np\n"
        "def myfunc(ins,outs):\n"
        "  outs['AddedIntensity'] = np.zeros(ins['X'].size, dtype=np.double) + 1\n"
        "  outs['AddedPointSourceId'] = np.zeros(ins['X'].size, dtype=np.double) + 2\n"
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

    Stage* filter(f.createStage("filters.python"));
    filter->setOptions(opts);
    filter->setInput(reader);

    PointTable table;
    filter->prepare(table);
    PointViewSet viewSet = filter->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    PointLayoutPtr layout(table.layout());

    Dimension::Id int_id = layout->findDim("AddedIntensity");
    Dimension::Id psid_id = layout->findDim("AddedPointSourceId");

    for (unsigned int i = 0; i < view->size(); ++i)
    {
        EXPECT_EQ(view->getFieldAs<uint16_t>(int_id, i), 1);
        EXPECT_EQ(view->getFieldAs<uint16_t>(psid_id, i), 2);
    }
}

TEST_F(PythonFilterTest, metadata)
{
    StageFactory f;

    BOX3D bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 10);
    ops.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(ops);

    Option source("source", "import numpy\n"
        "import sys\n"
        "import redirector\n"
        "def myfunc(ins,outs):\n"
        "  global metadata\n"
        "  #print('before', globals(),  file=sys.stderr,)\n"
        "  metadata = {'name': 'root', 'value': 'a string', 'type': 'string', 'description': 'a description', 'children': [{'name': 'filters.python', 'value': 52, 'type': 'integer', 'description': 'a filter description', 'children': []}, {'name': 'readers.faux', 'value': 'another string', 'type': 'string', 'description': 'a reader description', 'children': []}]}\n"
        " # print ('schema', schema, file=sys.stderr,)\n"
        "  return True\n"
    );
    Option module("module", "MyModule");
    Option function("function", "myfunc");
    Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);

    Stage* filter(f.createStage("filters.python"));
    filter->setOptions(opts);
    filter->setInput(reader);

    PointTable table;
    filter->prepare(table);
    PointViewSet viewSet = filter->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    PointLayoutPtr layout(table.layout());
    MetadataNode m = table.metadata();
    m = m.findChild("filters.python");
    MetadataNodeList l = m.children();
    EXPECT_EQ(l.size(), 3u);
    EXPECT_EQ(l[0].name(), "filters.python");
    EXPECT_EQ(l[0].value(), "52");
    EXPECT_EQ(l[0].description(), "a filter description");
}

TEST_F(PythonFilterTest, pdalargs)
{
    StageFactory f;

    BOX3D bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 10);
    ops.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(ops);

    Option source("source", "import numpy\n"
        "import sys\n"
        "import redirector\n"
        "def myfunc(ins,outs):\n"
        "  pdalargs['name']\n"
        "# print ('pdalargs', pdalargs, file=sys.stderr,)\n"
        "  return True\n"
    );
    Option module("module", "MyModule");
    Option function("function", "myfunc");
    Option args("pdalargs", "{\"name\":\"Howard\",\"something\":42, \"another\": \"True\"}");
    Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);
    opts.add(args);

    Stage* filter(f.createStage("filters.python"));
    filter->setOptions(opts);
    filter->setInput(reader);

    PointTable table;
    filter->prepare(table);
    PointViewSet viewSet = filter->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();

    // Not throwing anything is success for now
}


class PredicateFilterTest : public ::testing::Test
{
public:
    virtual void SetUp()
    {
        pdal::plang::Environment::get();
    }

};


TEST_F(PredicateFilterTest, PredicateFilterTest_test_programmable)
{
    StageFactory f;

    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOps;
    readerOps.add("bounds", bounds);
    readerOps.add("count", 1000);
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

    Stage* filter(f.createStage("filters.python"));
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

    EXPECT_TRUE(Utils::compare_approx(statsX.minimum(), 0.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsY.minimum(), 0.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsZ.minimum(), 0.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsX.maximum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsY.maximum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsZ.maximum(), 1.0, 0.01));
}

TEST_F(PredicateFilterTest, PredicateFilterTest_test_programmable_2)
{
    StageFactory f;
    // same as above, but with 'Y >' instead of 'X <'

    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOps;
    readerOps.add("bounds", bounds);
    readerOps.add("count", 1000);
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

    Stage* filter(f.createStage("filters.python"));
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

    EXPECT_TRUE(Utils::compare_approx(statsX.minimum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsY.minimum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsZ.minimum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsX.maximum(), 2.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsY.maximum(), 2.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsZ.maximum(), 2.0, 0.01));
}

TEST_F(PredicateFilterTest, PredicateFilterTest_test_programmable_3)
{
    StageFactory f;
    // can we make a pipeline with TWO python filters in it?

    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOpts;
    readerOpts.add("bounds", bounds);
    readerOpts.add("count", 1000);
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

    Stage* filter1(f.createStage("filters.python"));
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

    Stage* filter2(f.createStage("filters.python"));
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

    EXPECT_TRUE(Utils::compare_approx(statsX.minimum(), 0.5, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsY.minimum(), 0.5, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsZ.minimum(), 0.5, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsX.maximum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsY.maximum(), 1.0, 0.01));
    EXPECT_TRUE(Utils::compare_approx(statsZ.maximum(), 1.0, 0.01));
}

TEST_F(PredicateFilterTest, PredicateFilterTest_test_programmable_4)
{
    StageFactory f;
    // test the point counters in the Predicate's iterator

    BOX3D bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    Options readerOpts;
    readerOpts.add("bounds", bounds);
    readerOpts.add("count", 1000);
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

    Stage* filter(f.createStage("filters.python"));
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


TEST_F(PredicateFilterTest, PredicateFilterTest_PipelineJSON)
{
    PipelineManager mgr;

    mgr.readPipeline(Support::configuredpath("plang/from-module.json"));
    point_count_t cnt = mgr.execute();
    EXPECT_EQ(cnt, 1u);
}

TEST_F(PredicateFilterTest, PredicateFilterTest_EmbedJSON)
{
    PipelineManager mgr;

    mgr.readPipeline(Support::configuredpath("plang/predicate-embed.json"));
    point_count_t cnt = mgr.execute();
    EXPECT_EQ(cnt, 1u);
}


TEST(PLangTest, PLangTest_basic)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  #print 'hi'\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script, MetadataNode(), "");

    PointTable table;
    PointViewPtr view(new PointView(table));
    meth.execute(view, MetadataNode());
}

//---------------------------------------------------------------------------
//
// Error tests
//
//---------------------------------------------------------------------------

TEST(PLangTest, PLangTest_compile_error)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "return True\n"
        ;
    Script script(source, "MyTest", "yow");
    ASSERT_THROW(Invocation meth(script, MetadataNode(), ""), pdal::pdal_error);
}


TEST(PLangTest, PLangTest_runtime_error)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  z['s'] = 9\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script, MetadataNode(), "");

    PointTable table;
    PointViewPtr view(new PointView(table));
    ASSERT_THROW(meth.execute(view, MetadataNode()), pdal::pdal_error);
}


TEST(PLangTest, PLangTest_returnvoid)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs,mids):\n"
        "  #print 'foo'\n"
        "  return\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script, MetadataNode(), "");

    PointTable table;
    PointViewPtr view(new PointView(table));
    ASSERT_THROW(meth.execute(view, MetadataNode()), pdal::pdal_error);
}


TEST(PLangTest, PLangTest_returnint)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs,mids):\n"
        "  #print 'foo'\n"
        "  return 7\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script, MetadataNode(), "");

    PointTable table;
    PointViewPtr view(new PointView(table));
    ASSERT_THROW(meth.execute(view, MetadataNode()), pdal::pdal_error);
}

//---------------------------------------------------------------------------
//
// PARAM tests
//
//---------------------------------------------------------------------------

TEST(PLangTest, PLangTest_aliases)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  \n"
        "  #print ins['X']\n"
        "  #print ins['prefix.X']\n"
        "  \n"
        "  X = ins['X']\n"
        "  prefixX = ins['prefix.X']\n"
        "  \n"
        "  #print X\n"
        "  #print prefixX\n"
        "  \n"
        "  Y = X + prefixX\n"
        "  prefixY = Y\n"
        "  \n"
        "  #print Y\n"
        "  #print prefixY\n"
        "  \n"
        "  outs['Y'] = Y\n"
        "  outs['prefix.Y'] = prefixY\n"
        "  \n"
        "  #print outs['Y']\n"
        "  #print outs['prefix.Y']\n"
        "  return True\n"
        ;

    PointTable table;
    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    Dimension::Id prefixX = table.layout()->assignDim("prefix.X",
        Dimension::Type::Double);
    Dimension::Id prefixY = table.layout()->assignDim("prefix.Y",
        Dimension::Type::Double);
    PointViewPtr view(new PointView(table));
    view->setField(Dimension::Id::X, 0, 1);
    view->setField(Dimension::Id::X, 1, 2);
    view->setField(Dimension::Id::X, 2, 3);
    view->setField(Dimension::Id::X, 3, 4);
    view->setField(Dimension::Id::X, 4, 5);

    view->setField(prefixX, 0, 1);
    view->setField(prefixX, 1, 2);
    view->setField(prefixX, 2, 3);
    view->setField(prefixX, 3, 4);
    view->setField(prefixX, 4, 5);

    Script script(source, "MyTest", "yow");
    Invocation meth(script, MetadataNode(), "");
    meth.execute(view, MetadataNode());

    double ck = 2;
    for (PointId idx = 0; idx < 5; ++idx)
    {
        EXPECT_DOUBLE_EQ(view->getFieldAs<double>(Dimension::Id::Y, idx), ck);
        EXPECT_DOUBLE_EQ(view->getFieldAs<double>(prefixY, idx), ck);
        ck += 2;
    }
}


TEST(PLangTest, PLangTest_returntrue)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script, MetadataNode(), "");

    PointTable table;
    PointViewPtr view(new PointView(table));

    EXPECT_TRUE(meth.execute(view, MetadataNode()));
}


TEST(PLangTest, PLangTest_returnfalse)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  return False\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script, MetadataNode(), "");

    PointTable table;
    PointViewPtr view(new PointView(table));
    EXPECT_FALSE(meth.execute(view, MetadataNode()));
}


//---------------------------------------------------------------------------
//
// MISC tests
//
//---------------------------------------------------------------------------

TEST(PLangTest, log)
{
    // verify we can redirect the stdout inside the python script

    Options reader_opts;
    {
        BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
        Option opt1("bounds", bounds);
        Option opt2("count", 750);
        Option opt3("mode", "constant");

        reader_opts.add(opt1);
        reader_opts.add(opt2);
        reader_opts.add(opt3);

        Option optlog("log", Support::temppath("mylog_three.txt"));
        reader_opts.add(optlog);
    }

    Options xfilter_opts;
    {
        const Option source("source",
            "import numpy as np\n"
            "import sys\n"
            "def xfunc(ins,outs):\n"
            "  X = ins['X']\n"
            "  print (\"Testing log output through python script.\")\n"
            "  X = X + 1.0\n"
            "  outs['X'] = X\n"
            "  sys.stdout.flush()\n"
            "  return True\n"
            );
        const Option module("module", "xModule");
        const Option function("function", "xfunc");
        xfilter_opts.add("log", Support::temppath("mylog_three.txt"));
        xfilter_opts.add(source);
        xfilter_opts.add(module);
        xfilter_opts.add(function);
    }

    StageFactory f;
    {
        FauxReader reader;

        reader.setOptions(reader_opts);

        Stage* xfilter(f.createStage("filters.python"));
        xfilter->setOptions(xfilter_opts);
        xfilter->setInput(reader);

        PointTable table;
        xfilter->prepare(table);
        PointViewSet pvSet = xfilter->execute(table);
        EXPECT_EQ(pvSet.size(), 1u);
        PointViewPtr view = *pvSet.begin();
        EXPECT_EQ(view->size(), 750u);
    }

    bool ok = Support::compare_text_files(
        Support::temppath("mylog_three.txt"),
        Support::datapath("logs/log_py.txt"));

    // TODO: fails on Windows
    // unknown file: error: C++ exception with description "pdalboost::filesystem::remove:
    // The process cannot access the file because it is being used by another process:
    // "C:/projects/pdal/test/data/../temp/mylog_three.txt"" thrown in the test body.
    //if (ok)
    //    FileUtils::deleteFile(Support::temppath("mylog_three.txt"));

    EXPECT_TRUE(ok);
}


TEST_F(PythonFilterTest, PythonFilterTest_modify)
{
    StageFactory f;

    BOX3D bounds(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 10);
    ops.add("mode", "ramp");

    FauxReader reader;
    reader.setOptions(ops);

    Option source("source", "import numpy as np\n"
        "def myfunc(ins,outs):\n"
        "  X = ins['X']\n"
        "  Y = ins['Y']\n"
        "  Z = ins['Z']\n"
        "  X = np.delete(X, (9), axis=0)\n"
        "  Y = np.delete(Y, (9), axis=0)\n"
        "  Z = np.delete(Z, (9), axis=0)\n"
        "  Z = np.append(Z,100)\n"
        "  Y = np.append(Y,200)\n"
        "#  print (Z)\n"
        "#  print (X)\n"
        "  outs['Z'] = Z\n"
        "  outs['Y'] = Y\n"
        "  outs['X'] = X\n"
        "#  print (len(X), len(Y), len(Z))\n"
        "  return True\n"
    );
    Option module("module", "MyModule");
    Option function("function", "myfunc");
    Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);

    Stage* filter(f.createStage("filters.python"));
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

    EXPECT_EQ(view->size(), 10u);

    EXPECT_DOUBLE_EQ(statsX.minimum(), 0.0);
    EXPECT_DOUBLE_EQ(statsX.maximum(), 1.0);

    EXPECT_DOUBLE_EQ(statsY.minimum(), 0.0);
    EXPECT_DOUBLE_EQ(statsY.maximum(), 200.0);

    EXPECT_DOUBLE_EQ(statsZ.minimum(), 0.0);
    EXPECT_DOUBLE_EQ(statsZ.maximum(), 100);
}


// most pipelines (those with a writer) will be invoked via `pdal pipeline`
static void run_pipeline(std::string const& pipeline)
{
    const std::string cmd = Support::binpath(Support::exename("pdal") +
        " pipeline");

    std::string output;
    std::string file(Support::configuredpath(pipeline));
    int stat = pdal::Utils::run_shell_command(cmd + " " + file, output);
    EXPECT_EQ(0, stat);
    if (stat)
    {
        std::cerr << "error message in run_pipeline: " << output << std::endl;
    }
}

class jsonWithProgrammable : public testing::TestWithParam<const char*> {};

TEST_P(jsonWithProgrammable, pipeline)
{
    pdal::plang::Environment::get();
    pdal::StageFactory f;
    pdal::Stage* s = f.createStage("filters.python");
    if (s)
        run_pipeline(GetParam());
    else
        std::cerr << "WARNING: could not create filters.programmable, skipping test" << std::endl;
}

INSTANTIATE_TEST_CASE_P(plugins, jsonWithProgrammable,
                        testing::Values(
                            // "pipeline/programmable-hag.json",
                            "pipeline/programmable-update-y-dims.json"
                        ));

class jsonWithPredicate : public testing::TestWithParam<const char*> {};

TEST_P(jsonWithPredicate, pipeline)
{
    pdal::plang::Environment::get();
    pdal::StageFactory f;
    pdal::Stage* s = f.createStage("filters.python");
    if (s)
        run_pipeline(GetParam());
    else
        std::cerr << "WARNING: could not create filters.python, skipping test" << std::endl;
}

INSTANTIATE_TEST_CASE_P(plugins, jsonWithPredicate,
                        testing::Values(
                            "pipeline/crop_wkt_2d_classification.json",
                            "pipeline/from-module.json",
                            "pipeline/predicate-embed.json",
                            "pipeline/predicate-keep-ground-and-unclass.json",
                            "pipeline/predicate-keep-last-return.json",
                            "pipeline/predicate-keep-specified-returns.json",
                            "pipeline/reproject.json"
                        ));
