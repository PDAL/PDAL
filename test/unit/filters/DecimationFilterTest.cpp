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

#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/filters/Decimation.hpp>

#include "../StageTester.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(DecimationFilterTest)

BOOST_AUTO_TEST_CASE(DecimationFilterTest_test1)
{
    BOX3D srcBounds(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "random");
    ops.add("num_points", 30);
    drivers::faux::Reader reader(ops);

    Options decimationOps;
    decimationOps.add("step", 10);
    filters::Decimation filter(decimationOps);
    filter.setInput(&reader);
    BOOST_CHECK(filter.getDescription() == "Decimation Filter");

    PointContext ctx;

    filter.prepare(ctx);
    PointBufferSet pbSet = filter.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 3);

    uint64_t t0 = buf->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, 0);
    uint64_t t1 = buf->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, 1);
    uint64_t t2 = buf->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, 2);

    BOOST_CHECK_EQUAL(t0, 0);
    BOOST_CHECK_EQUAL(t1, 10);
    BOOST_CHECK_EQUAL(t2, 20);
}

BOOST_AUTO_TEST_SUITE_END()
