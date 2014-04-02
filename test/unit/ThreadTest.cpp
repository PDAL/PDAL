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
#include <sstream>
#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(ThreadTest)


#ifdef PDAL_HAVE_PYTHON

Options makeFilterOptions(int threadNum, int filterNum, bool log)
{
    using namespace std;

    ostringstream id;
    id << "T" << threadNum << "F" << filterNum;

    ostringstream function_id;
    function_id << "function_" << id.str();
    ostringstream module_id;
    function_id << "module_" << id.str();

    ostringstream script;
    script <<
        "import numpy as np\n"
        "def " << function_id.str() << "(ins,outs):\n"
        "  #print 'hi1 " << id.str() << "'\n"
        "  X = ins['X']\n"
        "  #print 'hi2 " << id.str() << "'\n"
        "  X = X + 1.0\n"
        "  #print 'hi2 " << id.str() << "'\n"
        "  outs['X'] = X\n"
        "  #print 'hi4 " << id.str() << "'\n"
        "  return True\n"
        ;

    Options opts;
    {
        const pdal::Option source("source", script.str());
        const pdal::Option module("module", module_id.str());
        const pdal::Option function("function", function_id.str());
        opts.add(source);
        opts.add(module);
        opts.add(function);

        if (log)
        {
            ostringstream log_id;
            log_id << "log_" << id.str() << ".txt";
            Option optlog("log", Support::temppath(log_id.str()));
            opts.add(optlog);
        }
    }

    return opts;
}


Options makeReaderOptions()
{
    Options opts;
    {
        const Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
        Option opt1("bounds", bounds);
        Option opt2("log", Support::temppath("logtest_1.txt"));
        Option opt3("num_points", 1000);
        Option opt4("mode", "constant");

        opts.add(opt1);
        opts.add(opt2);
        opts.add(opt3);
        opts.add(opt4);
    }

    return opts;
}


BOOST_AUTO_TEST_CASE(test_parallel)
{
    //return;

#define TEN 10
    Options opts[TEN][TEN];
    Stage* stages[TEN][TEN];
    StageSequentialIterator* iters[TEN];
    PointBuffer* datas[TEN];

    for (int threadId=0; threadId<TEN; threadId++)
    {
        opts[threadId][0] = makeReaderOptions();
        stages[threadId][0] = new drivers::faux::Reader(opts[threadId][0]);

        for (int stageId=1; stageId<TEN; stageId++)
        {
            opts[threadId][stageId] = makeFilterOptions(threadId, stageId, true);

            stages[threadId][stageId] = new filters::Programmable(*stages[threadId][stageId-1], opts[threadId][stageId]);
        }
    }

    for (int threadId=0; threadId<TEN; threadId++)
    {
        stages[threadId][TEN-1]->initialize();
    }

    //reader.log()->setLevel(logDEBUG5);
    //xfilter.log()->setLevel(logDEBUG5);
    //yfilter.log()->setLevel(logDEBUG5);

    for (int threadId=0; threadId<TEN; threadId++)
    {
        const Schema& schema = stages[threadId][9]->getSchema();
        datas[threadId] = new PointBuffer(schema, 750);

        iters[threadId] = stages[threadId][9]->createSequentialIterator(*datas[threadId]);
    }

    for (int threadId=0; threadId<TEN; threadId++)
    {
        boost::uint32_t numRead = iters[threadId]->read(*datas[threadId]);

        BOOST_CHECK_EQUAL(numRead, 750u);
    }

    for (int threadId=0; threadId<TEN; threadId++)
    {
        delete iters[threadId];
        delete datas[threadId];

        for (int filterId=0; filterId<TEN; filterId++)
        {
            delete stages[threadId][filterId];
        }
    }

    //    bool ok = Support::compare_text_files(Support::temppath("mylog.txt"), Support::datapath("logtest.txt"));
  //  BOOST_CHECK(ok);

    FileUtils::deleteFile(Support::temppath("logtest_1.txt"));
    FileUtils::deleteFile(Support::temppath("logtest_2.txt"));
    FileUtils::deleteFile(Support::temppath("logtest_3.txt"));
    
    return;
}


#endif

BOOST_AUTO_TEST_SUITE_END()
