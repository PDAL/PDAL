/******************************************************************************
* Copyright (c) 2014, Hobu Inc. (hobu@hobu.co)
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

#include "gtest/gtest.h"

#include <random>

#include <SortFilter.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>
#include "Support.hpp"
#include "StageTester.hpp"

using namespace pdal;

TEST(SortFilterTest, simple)
{
    Options opts;

    opts.add("dimension", "X");

    point_count_t count = 10000;

    SortFilter filter;
    filter.setOptions(opts);

    PointContext ctx;
    PointBuffer buf(ctx);

    ctx.registerDim(Dimension::Id::X);

    std::default_random_engine generator;
    std::uniform_real_distribution<double> dist(0.0, (double)count);

    for (PointId i = 0; i < count; ++i)
        buf.setField(Dimension::Id::X, i, dist(generator));

    filter.prepare(ctx);
    FilterTester::ready(&filter, ctx);
    FilterTester::filter(&filter, buf);
    FilterTester::done(&filter, ctx);

    for (PointId i = 1; i < count; ++i)
    {
        double d1 = buf.getFieldAs<double>(Dimension::Id::X, i - 1);
        double d2 = buf.getFieldAs<double>(Dimension::Id::X, i);
        EXPECT_TRUE(d1 <= d2);
    }
}

TEST(SortFilterTest, pipeline)
{
    PipelineManager mgr;
    PipelineReader reader(mgr);

    reader.readPipeline(Support::datapath("filters/sort.xml"));
    mgr.execute();

    PointBufferSet pbSet = mgr.buffers();

    EXPECT_EQ(pbSet.size(), 1u);
    PointBufferPtr buf = *pbSet.begin();

    for (PointId i = 1; i < buf->size(); ++i)
    {
        double d1 = buf->getFieldAs<double>(Dimension::Id::X, i - 1);
        double d2 = buf->getFieldAs<double>(Dimension::Id::X, i);
        EXPECT_TRUE(d1 <= d2);
    }
}
