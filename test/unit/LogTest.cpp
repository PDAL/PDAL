/******************************************************************************
* Copyright (c) 2012, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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
#include <pdal/Options.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <FauxReader.hpp>
#include "Support.hpp"

using namespace pdal;

TEST(LogTest, test_one)
{
    StageFactory f;

    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);

    Option opt1("bounds", bounds);
    Option opt2("log", Support::temppath("mylog_one.txt"));
    Option opt3("num_points", 750);
    Option opt4("mode", "constant");

    Options opts;
    opts.add(opt1);
    opts.add(opt2);
    opts.add(opt3);
    opts.add(opt4);

    {
        PointTable table;

        FauxReader reader;
        reader.setOptions(opts);
        reader.prepare(table);

        EXPECT_EQ(reader.log()->getLevel(), LogLevel::Error);
        reader.log()->setLevel(LogLevel::Debug5);
        EXPECT_EQ(reader.log()->getLevel(), LogLevel::Debug5);

        PointViewSet viewSet = reader.execute(table);
        EXPECT_EQ(viewSet.size(), 1u);
        PointViewPtr view = *viewSet.begin();
        EXPECT_EQ(view->size(), 750u);
    }
    bool ok = Support::compare_text_files(
        Support::temppath("mylog_one.txt"),
        Support::datapath("logs/logtest.txt"));

    if (ok)
        FileUtils::deleteFile(Support::temppath("mylog_one.txt"));

    EXPECT_TRUE(ok);
}

#ifdef PDAL_HAVE_PYTHON

TEST(LogTest, test_two_a)
{
    StageFactory f;

    Options reader_opts;
    {
        BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
        Option opt1("bounds", bounds);
        Option opt2("log", Support::temppath("logtest_123.txt"));
        Option opt3("num_points", 750);
        Option opt4("mode", "constant");

        reader_opts.add(opt1);
        reader_opts.add(opt2);
        reader_opts.add(opt3);
        reader_opts.add(opt4);
    }

    Options xfilter_opts;
    {
        const Option source("source",
            "import numpy as np\n"
            "def xfunc(ins,outs):\n"
            "  X = ins['X']\n"
            "  #print ins['X']\n"
            "  X = X + 1.0\n"
            "  outs['X'] = X\n"
            "  return True\n"
            );
        const Option module("module", "xModule");
        const Option function("function", "xfunc");
        xfilter_opts.add(source);
        xfilter_opts.add(module);
        xfilter_opts.add(function);
    }

    Options yfilter_opts;
    {
        const Option source("source",
            "import numpy as np\n"
            "def yfunc(ins,outs):\n"
            "  Y = ins['Y']\n"
            "  #print ins['Y']\n"
            "  Y = Y + 1.0\n"
            "  outs['Y'] = Y\n"
            "  return True\n"
            );
        const Option module("module", "yModule");
        const Option function("function", "yfunc");
        yfilter_opts.add(source);
        yfilter_opts.add(module);
        yfilter_opts.add(function);
    }

    {
        FauxReader reader;
        reader.setOptions(reader_opts);

        std::unique_ptr<Stage> xfilter(f.createStage("filters.programmable"));
        xfilter->setOptions(xfilter_opts);
        xfilter->setInput(reader);

        std::unique_ptr<Stage> yfilter(f.createStage("filters.programmable"));
        yfilter->setOptions(yfilter_opts);
        yfilter->setInput(*xfilter);

        PointTable table;
        yfilter->prepare(table);
        EXPECT_TRUE(true);

        reader.log()->setLevel(LogLevel::Debug5);
        xfilter->log()->setLevel(LogLevel::Debug5);
        yfilter->log()->setLevel(LogLevel::Debug5);
        EXPECT_TRUE(true);

        PointViewSet viewSet = yfilter->execute(table);
        EXPECT_TRUE(true);
        EXPECT_EQ(viewSet.size(), 1u);
        PointViewPtr view = *viewSet.begin();
        EXPECT_EQ(view->size(), 750u);
    }

    bool ok1 = Support::compare_text_files(
        Support::temppath("logtest_123.txt"),
        Support::datapath("logs/logtest_123.txt"));

    if (ok1)
      FileUtils::deleteFile(Support::temppath("logtest_123.txt"));

    EXPECT_TRUE(ok1);
}


TEST(LogTest, test_two_b)
{
    StageFactory f;

    Options reader_opts;
    {
        BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
        Option opt1("bounds", bounds);
        Option opt2("log", Support::temppath("logtest_test_two_b_1.txt"));
        Option opt3("num_points", 750);
        Option opt4("mode", "constant");

        reader_opts.add(opt1);
        reader_opts.add(opt2);
        reader_opts.add(opt3);
        reader_opts.add(opt4);
    }

    Options xfilter_opts;
    {
        const Option source("source",
            "import numpy as np\n"
            "def xfunc(ins,outs):\n"
            "  X = ins['X']\n"
            "  #print ins['X']\n"
            "  X = X + 1.0\n"
            "  outs['X'] = X\n"
            "  return True\n"
            );
        const Option module("module", "xModule");
        const Option function("function", "xfunc");
        xfilter_opts.add(source);
        xfilter_opts.add(module);
        xfilter_opts.add(function);

        Option optlog("log", Support::temppath("logtest_test_two_b_2.txt"));
        xfilter_opts.add(optlog);
    }

    Options yfilter_opts;
    {
        const Option source("source",
            "import numpy as np\n"
            "def yfunc(ins,outs):\n"
            "  Y = ins['Y']\n"
            "  #print ins['Y']\n"
            "  Y = Y + 1.0\n"
            "  outs['Y'] = Y\n"
            "  return True\n"
            );
        const Option module("module", "yModule");
        const Option function("function", "yfunc");
        yfilter_opts.add(source);
        yfilter_opts.add(module);
        yfilter_opts.add(function);

        Option optlog("log", Support::temppath("logtest_test_two_b_3.txt"));
        yfilter_opts.add(optlog);
    }

    {
        FauxReader reader;
        reader.setOptions(reader_opts);

        std::unique_ptr<Stage> xfilter(f.createStage("filters.programmable"));
        xfilter->setOptions(xfilter_opts);
        xfilter->setInput(reader);

        std::unique_ptr<Stage> yfilter(f.createStage("filters.programmable"));
        yfilter->setOptions(yfilter_opts);
        yfilter->setInput(*xfilter);

        PointTable table;
        yfilter->prepare(table);

        reader.log()->setLevel(LogLevel::Debug5);
        xfilter->log()->setLevel(LogLevel::Debug5);
        yfilter->log()->setLevel(LogLevel::Debug5);

        PointViewSet viewSet = yfilter->execute(table);
        EXPECT_EQ(viewSet.size(), 1u);
        PointViewPtr view = *viewSet.begin();
        EXPECT_EQ(view->size(), 750u);
    }

    bool ok1 = Support::compare_text_files(
        Support::temppath("logtest_test_two_b_1.txt"),
        Support::datapath("logs/logtest_1.txt"));
    bool ok2 = Support::compare_text_files(
        Support::temppath("logtest_test_two_b_2.txt"),
        Support::datapath("logs/logtest_2.txt"));
    bool ok3 = Support::compare_text_files(
        Support::temppath("logtest_test_two_b_3.txt"),
        Support::datapath("logs/logtest_3.txt"));

    if (ok1)
        FileUtils::deleteFile(Support::temppath("logtest_test_two_b_1.txt"));

    if (ok2)
        FileUtils::deleteFile(Support::temppath("logtest_test_two_b_2.txt"));

    if (ok3)
        FileUtils::deleteFile(Support::temppath("logtest_test_two_b_3.txt"));

    EXPECT_TRUE(ok1);
    EXPECT_TRUE(ok2);
    EXPECT_TRUE(ok3);
}


TEST(LogTest, test_three)
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
        PointViewSet viewSet = xfilter->execute(table);
        EXPECT_EQ(viewSet.size(), 1u);
        PointViewPtr view = *viewSet.begin();
        EXPECT_EQ(view->size(), 750u);
    }

    bool ok = Support::compare_text_files(
        Support::temppath("mylog_three.txt"),
        Support::datapath("logs/log_py.txt"));

    if (ok)
        FileUtils::deleteFile(Support::temppath("mylog_three.txt"));

    EXPECT_TRUE(ok);
}

#endif
