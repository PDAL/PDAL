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

#include <pdal/SchemaLayout.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Iterator.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/filters/CacheFilter.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(CacheFilterTest)

BOOST_AUTO_TEST_CASE(test1)
{
    Bounds<double> srcBounds(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);
    pdal::drivers::faux::Reader reader(srcBounds, 10000, pdal::drivers::faux::Reader::Constant);

    pdal::filters::CacheFilter cache(reader, 2, 1024);
    BOOST_CHECK(cache.getDescription() == "Cache Filter");

    const Schema& schema = reader.getSchema();
    SchemaLayout layout(schema);
    const int offsetT = schema.getDimensionIndex(Dimension::Field_Time, Dimension::Uint64);

    PointBuffer dataBig(layout, 1024);
    PointBuffer dataSmall(layout, 1);

    SequentialIterator* iter1 = cache.createSequentialIterator();

    //BOOST_CHECK(cache.getIndex() == 0);
    BOOST_CHECK(cache.getNumPointsRequested() == 0);
    BOOST_CHECK(cache.getNumPointsRead() == 0);

    iter1->read(dataBig);
    BOOST_CHECK(dataBig.getField<boost::uint64_t>(0, offsetT) == 0);
    //BOOST_CHECK(cache.getIndex() == 1024);
    BOOST_CHECK(cache.getNumPointsRequested() == 1024);
    BOOST_CHECK(cache.getNumPointsRead() == 1024);

    iter1->read(dataBig);
    BOOST_CHECK(dataBig.getField<boost::uint64_t>(0, offsetT) == 1024);
   // BOOST_CHECK(cache.getIndex() == 2048);
    BOOST_CHECK(cache.getNumPointsRequested() == 2048);
    BOOST_CHECK(cache.getNumPointsRead() == 2048);

    SequentialIterator* iter2 = cache.createSequentialIterator();

    iter2->skip(42);
    iter2->read(dataSmall);
    BOOST_CHECK(dataSmall.getField<boost::uint64_t>(0, offsetT) == 42);
    //BOOST_CHECK(cache.getIndex() == 43);
    BOOST_CHECK(cache.getNumPointsRequested() == 2048+1);
    BOOST_CHECK(cache.getNumPointsRead() == 2048);

    delete iter1;
    delete iter2;

    return;
}

BOOST_AUTO_TEST_SUITE_END()
