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

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/filters/Cache.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(CacheFilterTest)

BOOST_AUTO_TEST_CASE(test1)
{
    Bounds<double> srcBounds(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);
    pdal::drivers::faux::Reader reader(srcBounds, 10000, pdal::drivers::faux::Reader::Constant);

    pdal::filters::Cache cache(reader, 2, 1024);
    BOOST_CHECK(cache.getDescription() == "Cache Filter");
    cache.initialize();

    const Schema& schema = reader.getSchema();

    PointBuffer dataBig(schema, 1024);
    PointBuffer dataSmall(schema, 1);

    StageSequentialIterator* iter1 = cache.createSequentialIterator(dataBig);

    //BOOST_CHECK(cache.getIndex() == 0);
    BOOST_CHECK(cache.getNumPointsRequested() == 0);
    BOOST_CHECK(cache.getNumPointsRead() == 0);

    iter1->read(dataBig);
    BOOST_CHECK(dataBig.getField<boost::uint64_t>(dataBig.getSchema().getDimension("Time"), 0) == 0);
    //BOOST_CHECK(cache.getIndex() == 1024);
    BOOST_CHECK(cache.getNumPointsRequested() == 1024);
    BOOST_CHECK(cache.getNumPointsRead() == 1024);

    iter1->read(dataBig);
    BOOST_CHECK(dataBig.getField<boost::uint64_t>(dataBig.getSchema().getDimension("Time"), 0) == 1024);
    // BOOST_CHECK(cache.getIndex() == 2048);
    BOOST_CHECK(cache.getNumPointsRequested() == 2048);
    BOOST_CHECK(cache.getNumPointsRead() == 2048);

    StageSequentialIterator* iter2 = cache.createSequentialIterator(dataSmall);

    iter2->skip(42);
    iter2->read(dataSmall);
    BOOST_CHECK(dataSmall.getField<boost::uint64_t>(dataSmall.getSchema().getDimension("Time"), 0) == 42);
    //BOOST_CHECK(cache.getIndex() == 43);
    BOOST_CHECK(cache.getNumPointsRequested() == 2048+1);
    BOOST_CHECK(cache.getNumPointsRead() == 2048);

    delete iter1;
    delete iter2;

    return;
}


BOOST_AUTO_TEST_CASE(CacheFilterTest_test_options)
{
    Bounds<double> srcBounds(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);
    pdal::drivers::faux::Reader reader(srcBounds, 10000, pdal::drivers::faux::Reader::Constant);

    Option opt1("max_cache_blocks", 2);
    Option opt2("cache_block_size", 1024);
    Options opts;
    opts.add(opt1);
    opts.add(opt2);
    pdal::filters::Cache cache(reader, opts);
    BOOST_CHECK(cache.getDescription() == "Cache Filter");
    cache.initialize();

    const Schema& schema = reader.getSchema();

    PointBuffer dataBig(schema, 1024);
    PointBuffer dataSmall(schema, 1);

    StageSequentialIterator* iter1 = cache.createSequentialIterator(dataBig);

    //BOOST_CHECK(cache.getIndex() == 0);
    BOOST_CHECK(cache.getNumPointsRequested() == 0);
    BOOST_CHECK(cache.getNumPointsRead() == 0);

    iter1->read(dataBig);
    BOOST_CHECK(dataBig.getField<boost::uint64_t>(dataBig.getSchema().getDimension("Time"), 0) == 0);
    //BOOST_CHECK(cache.getIndex() == 1024);
    BOOST_CHECK(cache.getNumPointsRequested() == 1024);
    BOOST_CHECK(cache.getNumPointsRead() == 1024);

    iter1->read(dataBig);
    BOOST_CHECK(dataBig.getField<boost::uint64_t>(dataBig.getSchema().getDimension("Time"), 0) == 1024);
    // BOOST_CHECK(cache.getIndex() == 2048);
    BOOST_CHECK(cache.getNumPointsRequested() == 2048);
    BOOST_CHECK(cache.getNumPointsRead() == 2048);

    StageSequentialIterator* iter2 = cache.createSequentialIterator(dataSmall);

    iter2->skip(42);
    iter2->read(dataSmall);
    BOOST_CHECK(dataSmall.getField<boost::uint64_t>(dataBig.getSchema().getDimension("Time"), 0) == 42);
    //BOOST_CHECK(cache.getIndex() == 43);
    BOOST_CHECK(cache.getNumPointsRequested() == 2048+1);
    BOOST_CHECK(cache.getNumPointsRead() == 2048);

    delete iter1;
    delete iter2;

    return;
}

BOOST_AUTO_TEST_SUITE_END()
