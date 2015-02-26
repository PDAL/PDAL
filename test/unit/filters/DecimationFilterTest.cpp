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

#include "gtest/gtest.h"

#include <pdal/PointBuffer.hpp>
#include <pdal/StageFactory.hpp>
#include <DecimationFilter.hpp>

using namespace pdal;

TEST(DecimationFilterTest, DecimationFilterTest_test1)
{
    BOX3D srcBounds(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "random");
    ops.add("num_points", 30);
    StageFactory f;
    std::unique_ptr<Stage> reader(f.createStage("readers.faux"));
    EXPECT_TRUE(reader.get());
    reader->setOptions(ops);

    Options decimationOps;
    decimationOps.add("step", 10);
    
    std::unique_ptr<Stage> filter(f.createStage("filters.decimation"));
    EXPECT_TRUE(filter.get());
    filter->setOptions(decimationOps);
    filter->setInput(reader.get());

    PointContext ctx;

    filter->prepare(ctx);
    PointBufferSet pbSet = filter->execute(ctx);
    EXPECT_EQ(pbSet.size(), 1u);
    PointBufferPtr buf = *pbSet.begin();
    EXPECT_EQ(buf->size(), 3u);

    uint64_t t0 = buf->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, 0);
    uint64_t t1 = buf->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, 1);
    uint64_t t2 = buf->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, 2);

    EXPECT_EQ(t0, 0u);
    EXPECT_EQ(t1, 10u);
    EXPECT_EQ(t2, 20u);
}
