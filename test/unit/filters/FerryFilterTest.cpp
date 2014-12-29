/******************************************************************************
* Copyright (c) 2014, Howard Butler (howard@hobu.co)
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
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>
#include "Support.hpp"

using namespace pdal;

TEST(FerryFilterTest, test_ferry_copy)
{
    PipelineManager mgr;
    PipelineReader specReader(mgr);
    specReader.readPipeline(Support::configuredpath("filters/ferry.xml"));

    Stage *stage = mgr.getStage();
    mgr.execute();
    PointContext ctx = mgr.context();

    PointBufferSet pbSet = mgr.buffers();

    EXPECT_EQ(pbSet.size(), 1u);
    PointBufferPtr buf = *pbSet.begin();
    EXPECT_EQ(buf->size(), 1065u);

    Dimension::Id::Enum state_plane_x = ctx.findDim("StatePlaneX");
    Dimension::Id::Enum state_plane_y = ctx.findDim("StatePlaneY");

    double lon = buf->getFieldAs<double>(Dimension::Id::X, 0);
    double lat = buf->getFieldAs<double>(Dimension::Id::Y, 0);

    double x = buf->getFieldAs<double>(state_plane_x, 0);
    double y = buf->getFieldAs<double>(state_plane_y, 0);

    EXPECT_FLOAT_EQ(-117.2501328350574, lon);
    EXPECT_FLOAT_EQ(49.341077824192915, lat);
    EXPECT_FLOAT_EQ(637012.24, x);
    EXPECT_FLOAT_EQ(849028.31, y);
}

TEST(FerryFilterTest, test_ferry_invalid)
{
    Options ops1;
    ops1.add("filename", Support::datapath("las/1.2-with-color.las"));
    StageFactory f;
    ReaderPtr reader(f.createReader("readers.las"));
    EXPECT_TRUE(reader.get());
    reader->setOptions(ops1);

    Options options;

    Option x("dimension", "X", "");
    Option toX("to","X", "");
    Options xO;
    xO.add(toX);
    x.setOptions(xO);
    options.add(x);

    FilterPtr ferry(f.createFilter("filters.ferry"));
    EXPECT_TRUE(ferry.get());
    ferry->setInput(reader.get());
    ferry->setOptions(options);

    PointContext ctx;

    EXPECT_THROW(ferry->prepare(ctx), pdal_error );
}
