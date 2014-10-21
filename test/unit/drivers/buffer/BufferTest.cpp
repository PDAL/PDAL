/******************************************************************************
* Copyright (c) 2014, Hobu Inc. (hobu@hobu.inc)
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

#include "UnitTest.hpp"

#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/buffer/BufferReader.hpp>
#include <pdal/filters/Stats.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(BufferTest)

using namespace pdal;

namespace
{

BOOST_AUTO_TEST_CASE(test_basic)
{
    PointContext ctx;

    PointBufferPtr buf(new PointBuffer(ctx));

    ctx.registerDim(Dimension::Id::X);
    ctx.registerDim(Dimension::Id::Y);
    ctx.registerDim(Dimension::Id::Z);

    for (int i = 0; i < 20; ++i)
    {
        buf->setField(Dimension::Id::X, i, i);
        buf->setField(Dimension::Id::Y, i, 2 * i);
        buf->setField(Dimension::Id::Z, i, -i);
    }

    Options ops;
    drivers::buffer::BufferReader r;
    r.setOptions(ops);
    r.addBuffer(buf);

    filters::Stats s;
    s.setOptions(ops);
    s.setInput(&r);

    s.prepare(ctx);
    PointBufferSet pbSet = s.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 20);

    filters::stats::Summary xSummary = s.getStats(Dimension::Id::X);
    BOOST_CHECK_CLOSE(xSummary.minimum(), 0, .0001);
    BOOST_CHECK_CLOSE(xSummary.maximum(), 19, .0001);
    BOOST_CHECK_EQUAL(xSummary.count(), 20);
    BOOST_CHECK_CLOSE(xSummary.average(), 9.5, .0001);

    filters::stats::Summary ySummary = s.getStats(Dimension::Id::Y);
    BOOST_CHECK_CLOSE(ySummary.minimum(), 0, .0001);
    BOOST_CHECK_CLOSE(ySummary.maximum(), 38, .0001);
    BOOST_CHECK_EQUAL(ySummary.count(), 20);
    BOOST_CHECK_CLOSE(ySummary.average(), 19, .0001);

    filters::stats::Summary zSummary = s.getStats(Dimension::Id::Z);
    BOOST_CHECK_CLOSE(zSummary.minimum(), -19, .0001);
    BOOST_CHECK_CLOSE(zSummary.maximum(), 0, .0001);
    BOOST_CHECK_EQUAL(zSummary.count(), 20);
    BOOST_CHECK_CLOSE(zSummary.average(), -9.5, .0001);
}

}

BOOST_AUTO_TEST_SUITE_END()
