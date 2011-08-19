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
#include <boost/scoped_ptr.hpp>

#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/filters/StatsFilter.hpp>
#include <pdal/filters/StatsFilterIterator.hpp>

#ifdef PDAL_COMPILER_GCC
#pragma GCC diagnostic ignored "-Wfloat-equal"
#endif

using namespace pdal;

BOOST_AUTO_TEST_SUITE(StatsFilterTest)

BOOST_AUTO_TEST_CASE(StatsFilterTest_test1)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Constant);

    pdal::filters::StatsFilter filter(reader, Options::none());
    BOOST_CHECK_EQUAL(filter.getName(), "filters.stats");
    BOOST_CHECK_EQUAL(filter.getDescription(), "Statistics Filter");
    filter.initialize();

    const Schema& schema = filter.getSchema();
    SchemaLayout layout(schema);
    PointBuffer data(layout, 1000);
    
    boost::scoped_ptr<pdal::StageSequentialIterator> iter(filter.createSequentialIterator());
    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 1000);
    }
    
    const pdal::filters::StatsCollector& statsX = filter.getStats(Dimension::Field_X);
    const pdal::filters::StatsCollector& statsY = filter.getStats(Dimension::Field_Y);
    const pdal::filters::StatsCollector& statsZ = filter.getStats(Dimension::Field_Z);

    BOOST_CHECK_EQUAL(statsX.count(), 1000u);
    BOOST_CHECK_EQUAL(statsY.count(), 1000u);
    BOOST_CHECK_EQUAL(statsZ.count(), 1000u);

    BOOST_CHECK_CLOSE(statsX.minimum(), 1.0, 0.0001);
    BOOST_CHECK_CLOSE(statsY.minimum(), 2.0, 0.0001);
    BOOST_CHECK_CLOSE(statsZ.minimum(), 3.0, 0.0001);

    BOOST_CHECK_CLOSE(statsX.maximum(), 1.0, 0.0001);
    BOOST_CHECK_CLOSE(statsY.maximum(), 2.0, 0.0001);
    BOOST_CHECK_CLOSE(statsZ.maximum(), 3.0, 0.0001);

    BOOST_CHECK_CLOSE(statsX.average(), 1.0, 0.0001);
    BOOST_CHECK_CLOSE(statsY.average(), 2.0, 0.0001);
    BOOST_CHECK_CLOSE(statsZ.average(), 3.0, 0.0001);

    return;
}

BOOST_AUTO_TEST_SUITE_END()
