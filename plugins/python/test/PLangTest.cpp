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

#include <pdal/plang/Invocation.hpp>
#include <pdal/plang/Array.hpp>

#include <Support.hpp>
#include <faux/FauxReader.hpp>
#include <pdal/StageFactory.hpp>

using namespace pdal;
using namespace pdal::plang;

TEST(PLangTest, PLangTest_basic)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  #print 'hi'\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();
    meth.execute();
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
    Invocation meth(script);

    ASSERT_THROW(meth.compile(), pdal::pdal_error);
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
    Invocation meth(script);
    meth.compile();

    ASSERT_THROW(meth.execute(), pdal::pdal_error);
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
    Invocation meth(script);
    meth.compile();

    ASSERT_THROW(meth.execute(), pdal::pdal_error);
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
    Invocation meth(script);
    meth.compile();

    ASSERT_THROW(meth.execute(), pdal::pdal_error);
}


//---------------------------------------------------------------------------
//
// PARAM tests
//
//---------------------------------------------------------------------------


TEST(PLangTest, PLangTest_ins)
{
    double data[5] = {1.0, 2.0, 3.0, 4.0, 5.0};

    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  #print ins['X']\n"
        "  X = ins['X']\n"
        "  #print X\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();
    meth.insertArgument("X", (uint8_t*)data, Dimension::Type::Double, 5);
    meth.execute();
}


TEST(PLangTest, PLangTest_outs)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  #print outs['X']\n"
        "  X = np.ones(5)\n"
        "  #print X\n"
        "  outs['X'] = X\n"
        "  #print outs['X']\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();
    meth.execute();
    EXPECT_TRUE(meth.hasOutputVariable("X"));
    void *output = meth.extractResult("X", Dimension::Type::Double);

    double *d = (double *)output;
    EXPECT_FLOAT_EQ(*d++, 1.0);
    EXPECT_FLOAT_EQ(*d++, 1.0);
    EXPECT_FLOAT_EQ(*d++, 1.0);
    EXPECT_FLOAT_EQ(*d++, 1.0);
    EXPECT_FLOAT_EQ(*d++, 1.0);
}


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
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();

    {
        double data[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
        meth.insertArgument("X", (uint8_t*)data, Dimension::Type::Double, 5);
        meth.insertArgument("prefix.X", (uint8_t*)data,
            Dimension::Type::Double, 5);
    }
    meth.execute();

    {
        EXPECT_TRUE(meth.hasOutputVariable("Y"));
        EXPECT_TRUE(meth.hasOutputVariable("prefix.Y"));

        void *output = meth.extractResult("Y", Dimension::Type::Double);
        double *d = (double *)output;
        EXPECT_FLOAT_EQ(*d++, 2.0);
        EXPECT_FLOAT_EQ(*d++, 4.0);
        EXPECT_FLOAT_EQ(*d++, 6.0);
        EXPECT_FLOAT_EQ(*d++, 8.0);
        EXPECT_FLOAT_EQ(*d++, 10.0);

        output = meth.extractResult("prefix.Y", Dimension::Type::Double);
        d = (double *)output;
        EXPECT_FLOAT_EQ(*d++, 2.0);
        EXPECT_FLOAT_EQ(*d++, 4.0);
        EXPECT_FLOAT_EQ(*d++, 6.0);
        EXPECT_FLOAT_EQ(*d++, 8.0);
        EXPECT_FLOAT_EQ(*d++, 10.0);
    }

    {
        std::vector<std::string> names;
        meth.getOutputNames(names);

        // We're getting stuff from a hash, so it
        // isn't stable
        std::sort(names.begin(), names.end());
        EXPECT_EQ(names.size(), 2u);
        EXPECT_EQ(names[0], "Y");
        EXPECT_EQ(names[1], "prefix.Y");
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
    Invocation meth(script);
    meth.compile();

    bool sts = meth.execute();
    EXPECT_TRUE(sts);
}


TEST(PLangTest, PLangTest_returnfalse)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  return False\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();

    bool sts = meth.execute();
    EXPECT_TRUE(!sts);
}


//---------------------------------------------------------------------------
//
// MISC tests
//
//---------------------------------------------------------------------------

TEST(PLangTest, PLangTest_reentry)
{
    const char* source =
        "import numpy as np\n"
        "def yow(ins,outs):\n"
        "  X = ins['X']\n"
        "  Y = X + 1.0\n"
        "  #print Y\n"
        "  outs['Y'] = Y\n"
        "  return True\n"
        ;
    Script script(source, "MyTest", "yow");
    Invocation meth(script);
    meth.compile();

    {
        double indata1[5] = {0.0, 1.0, 2.0, 3.0, 4.0};
        meth.insertArgument("X", (uint8_t*)indata1, Dimension::Type::Double, 5);
        meth.execute();
        void *output = meth.extractResult("Y", Dimension::Type::Double);

        double *d = (double *)output;
        EXPECT_FLOAT_EQ(*d++, 1.0);
        EXPECT_FLOAT_EQ(*d++, 2.0);
        EXPECT_FLOAT_EQ(*d++, 3.0);
        EXPECT_FLOAT_EQ(*d++, 4.0);
        EXPECT_FLOAT_EQ(*d++, 5.0);
    }

    {
        double indata2[5] = {10.0, 20.0, 30.0, 40.0, 50.0};
        meth.insertArgument("X", (uint8_t*)indata2, Dimension::Type::Double, 5);
        meth.execute();
        void *output = meth.extractResult("Y", Dimension::Type::Double);

        double *d = (double *)output;
        EXPECT_FLOAT_EQ(*d++, 11.0);
        EXPECT_FLOAT_EQ(*d++, 21.0);
        EXPECT_FLOAT_EQ(*d++, 31.0);
        EXPECT_FLOAT_EQ(*d++, 41.0);
        EXPECT_FLOAT_EQ(*d++, 51.0);
    }
}

TEST(PLangTest, log)
{
    StageFactory f;
    // verify we can redirect the stdout inside the python script

    Options reader_opts;
    {
        BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
        Option opt1("bounds", bounds);
        Option opt2("num_points", 750);
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
            "def xfunc(ins,outs):\n"
            "  X = ins['X']\n"
            "  print (\"Testing log output through python script.\")\n"
            "  X = X + 1.0\n"
            "  outs['X'] = X\n"
            "  return True\n"
            );
        const Option module("module", "xModule");
        const Option function("function", "xfunc");
        xfilter_opts.add("log", Support::temppath("mylog_three.txt"));
        xfilter_opts.add(source);
        xfilter_opts.add(module);
        xfilter_opts.add(function);
    }

    {
        FauxReader reader;

        reader.setOptions(reader_opts);

        std::unique_ptr<Stage> xfilter(f.createStage("filters.programmable"));
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

    if (ok)
        FileUtils::deleteFile(Support::temppath("mylog_three.txt"));

    EXPECT_TRUE(ok);
}



PointViewPtr makeTestView(PointTableRef table, point_count_t cnt = 17)
{
    PointLayoutPtr layout(table.layout());

    layout->registerDim(Dimension::Id::Classification);
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);

    PointViewPtr view(new PointView(table));

    // write the data into the view
    for (PointId i = 0; i < cnt; i++)
    {
        const uint8_t x = (uint8_t)(i + 1);
        const int32_t y = i * 10;
        const double z = i * 100;

        view->setField(Dimension::Id::Classification, i, x);
        view->setField(Dimension::Id::X, i, y);
        view->setField(Dimension::Id::Y, i, z);
    }
    EXPECT_EQ(view->size(), cnt);
    return view;
}

void verifyTestView(const PointView& view, point_count_t cnt = 17)
{
    // read the view back out
    for (PointId i = 0; i < cnt; i++)
    {
        uint8_t x = view.getFieldAs<uint8_t>(
            Dimension::Id::Classification, i);
        int32_t y = view.getFieldAs<uint32_t>(Dimension::Id::X, i);
        double z = view.getFieldAs<double>(Dimension::Id::Y, i);

        EXPECT_EQ(x, (uint8_t)(i + 1));
        EXPECT_EQ(y, (int32_t)(i * 10));
        EXPECT_TRUE(Utils::compare_approx(z, static_cast<double>(i) * 100.0,
            (std::numeric_limits<double>::min)()));
    }
}

TEST(PLangTest, PLangTest_array)
{
    PointTable table;
    PointViewPtr view = makeTestView(table, 40);

    plang::Array array;
    array.update(view);
    verifyTestView(*view.get(), 4);

}


