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

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/filters/Programmable.hpp>
#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(LogTest)

BOOST_AUTO_TEST_CASE(test_one)
{
    const Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);

    Option opt1("bounds", bounds);
    Option opt2("log", Support::temppath("mylog.txt"));
    Option opt3("num_points", 1000);
    Option opt4("mode", "constant");

    Options opts;
    opts.add(opt1);
    opts.add(opt2);
    opts.add(opt3);
    opts.add(opt4);

    {
        pdal::drivers::faux::Reader reader(opts);
        reader.initialize();

        BOOST_CHECK_EQUAL(reader.log()->getLevel(), logERROR);
        reader.log()->setLevel(logDEBUG5);
        BOOST_CHECK_EQUAL(reader.log()->getLevel(), logDEBUG5);

        const Schema& schema = reader.getSchema();
        PointBuffer data(schema, 750);

        StageSequentialIterator* iter = reader.createSequentialIterator(data);
        boost::uint32_t numRead = iter->read(data);

        BOOST_CHECK_EQUAL(numRead, 750u);

        delete iter;
    }

    bool ok = Support::compare_text_files(Support::temppath("mylog.txt"), Support::datapath("logtest.txt"));
    BOOST_CHECK(ok);

    FileUtils::deleteFile(Support::temppath("mylog.txt"));

    return;
}


BOOST_AUTO_TEST_CASE(test_two)
{
    // test commented out for now because it doesn't seem to do what mpg thought it was supposed to do
#if 0
    Options reader_opts;
    {
        const Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
        Option opt1("bounds", bounds);
        Option opt2("log", Support::temppath("logtest_1.txt"));
        Option opt3("num_points", 1000);
        Option opt4("mode", "constant");

        reader_opts.add(opt1);
        reader_opts.add(opt2);
        reader_opts.add(opt3);
        reader_opts.add(opt4);
    }

    Options xfilter_opts;
    {
        const pdal::Option source("source",
            "import numpy as np\n"
            "def xfunc(ins,outs):\n"
            "  X = ins['X']\n"
            "  #print ins['X']\n"
            "  X = X + 1.0\n"
            "  outs['X'] = X\n"
            "  return True\n"
            );
        const pdal::Option module("module", "xModule");
        const pdal::Option function("function", "xfunc");
        xfilter_opts.add(source);
        xfilter_opts.add(module);
        xfilter_opts.add(function);
    
        Option optlog("log", Support::temppath("logtest_2.txt"));
        xfilter_opts.add(optlog);
    }

    Options yfilter_opts;
    {
        const pdal::Option source("source",
            "import numpy as np\n"
            "def yfunc(ins,outs):\n"
            "  Y = ins['Y']\n"
            "  #print ins['Y']\n"
            "  Y = Y + 1.0\n"
            "  outs['Y'] = Y\n"
            "  return True\n"
            );
        const pdal::Option module("module", "yModule");
        const pdal::Option function("function", "yfunc");
        yfilter_opts.add(source);
        yfilter_opts.add(module);
        yfilter_opts.add(function);
    
        Option optlog("log", Support::temppath("logtest_3.txt"));
        yfilter_opts.add(optlog);
    }

    {
        drivers::faux::Reader reader(reader_opts);
        filters::Programmable xfilter(reader, xfilter_opts);
        filters::Programmable yfilter(xfilter, yfilter_opts);

        yfilter.initialize();

        reader.log()->setLevel(logDEBUG5);
        xfilter.log()->setLevel(logDEBUG5);
        yfilter.log()->setLevel(logDEBUG5);

        const Schema& schema = yfilter.getSchema();
        PointBuffer data(schema, 750);

        StageSequentialIterator* iter = yfilter.createSequentialIterator(data);
        boost::uint32_t numRead = iter->read(data);

        BOOST_CHECK_EQUAL(numRead, 750u);

        delete iter;
    }

    //bool ok = Support::compare_text_files(Support::temppath("mylog.txt"), Support::datapath("logtest.txt"));
    //BOOST_CHECK(ok);

    FileUtils::deleteFile(Support::temppath("logtest_1.txt"));
    FileUtils::deleteFile(Support::temppath("logtest_2.txt"));
    FileUtils::deleteFile(Support::temppath("logtest_3.txt"));
#endif
    return;
}


BOOST_AUTO_TEST_SUITE_END()
