/******************************************************************************
* Copyright (c) 2015, Howard Butler (howard@hobu.co)
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

#include <pdal/pdal_test_main.hpp>


#include <pdal/PointView.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/StageFactory.hpp>
#include <las/LasWriter.hpp>

#include "Support.hpp"

#include <iostream>

#ifdef PDAL_COMPILER_GCC
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif


using namespace pdal;

TEST(MrsidReaderTest, test_one)
{
    StageFactory f;

    Options sid_opts;
    sid_opts.add("filename", Support::datapath("mrsid/Tetons_200k.las.sid"));
    sid_opts.add("count", 750);

    PointTable table;

    std::shared_ptr<Stage> sid_reader(f.createStage("readers.mrsid"));
    EXPECT_TRUE(sid_reader.get());
    sid_reader->setOptions(sid_opts);
    sid_reader->prepare(table);
    PointViewSet pbSet = sid_reader->execute(table);
    EXPECT_EQ(pbSet.size(), 1u);
    PointViewPtr view = *pbSet.begin();
    EXPECT_EQ(view->size(), 750u);
    int32_t sid_x = view->getFieldAs<int32_t>(Dimension::Id::X, 10);
    int32_t sid_y = view->getFieldAs<int32_t>(Dimension::Id::Y, 10);
    int32_t sid_z = view->getFieldAs<int32_t>(Dimension::Id::Z, 10);
    EXPECT_EQ(sid_x, 504577);
    EXPECT_EQ(sid_y, 4795801);
    EXPECT_EQ(sid_z, 2505);

}


