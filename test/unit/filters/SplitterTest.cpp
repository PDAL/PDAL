/******************************************************************************
* Copyright (c) 2014, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/filters/Splitter.hpp>
#include <pdal/drivers/las/Reader.hpp>

#include "../StageTester.hpp"
#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(SplitterTest)

BOOST_AUTO_TEST_CASE(test_tile_filter)
{
    // create the reader
    drivers::las::Reader r(Support::datapath("1.2-with-color.las"));

    Options o;
    Option length("length", 1000, "length");
    o.add(length);

    // create the tile filter and prepare
    pdal::filters::Splitter s(o);
    s.setInput(&r);

    PointContext ctx;
    PointBufferPtr buf(new PointBuffer(ctx));
    s.prepare(ctx);

    StageTester::ready(&r, ctx);
    PointBufferSet pbSet = StageTester::run(&r, buf);
    StageTester::done(&r, ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    buf = *pbSet.begin();

    StageTester::ready(&s, ctx);
    pbSet = StageTester::run(&s, buf);
    StageTester::done(&s, ctx);

    std::vector<PointBufferPtr> buffers;
    for (auto it = pbSet.begin(); it != pbSet.end(); ++it)
        buffers.push_back(*it);

    auto sorter = [](PointBufferPtr p1, PointBufferPtr p2)
    {
        Bounds<double> b1 = p1->calculateBounds();
        Bounds<double> b2 = p2->calculateBounds();

        return b1.getMinimum(0) < b2.getMinimum(0) ?  true :
            b1.getMinimum(0) > b2.getMinimum(0) ? false :
            b1.getMinimum(1) < b2.getMinimum(1);
    };
    std::sort(buffers.begin(), buffers.end(), sorter);

    BOOST_CHECK_EQUAL(buffers.size(), 15);
    int counts[] = {24, 27, 26, 27, 10, 166, 142, 76, 141, 132, 63, 70, 67,
        34, 60 };
    for (size_t i = 0; i < buffers.size(); ++i)
    {
        PointBufferPtr& buf = buffers[i];
        BOOST_CHECK_EQUAL(buf->size(), counts[i]);
    }
}

BOOST_AUTO_TEST_SUITE_END()

