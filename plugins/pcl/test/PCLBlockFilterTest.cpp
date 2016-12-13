/******************************************************************************
* Copyright (c) 2013, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/pdal_test_main.hpp>

#include <pdal/PipelineManager.hpp>
#include <pdal/PluginManager.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(PCLBlockFilterTest, PCLBlockFilterTest_example_passthrough_json)
{
    StageFactory f;
    Stage* filter(f.createStage("filters.pclblock"));
    EXPECT_TRUE(filter);

    PipelineManager pipeline;

    pipeline.readPipeline(
        Support::configuredpath("filters/pcl/passthrough.json"));
    pipeline.execute();

    PointViewSet viewSet = pipeline.views();
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 795u);
}

static void test_filter(const std::string& jsonFile,
                        size_t expectedPointCount)
{
    StageFactory f;
    Options options;

    const std::string& autzen = "autzen/autzen-point-format-3.las";
    options.add("filename", Support::datapath(autzen));

    Stage* reader(f.createStage("readers.las"));
    EXPECT_TRUE(reader);
    reader->setOptions(options);

    Options filter_options;
    filter_options.add("filename", Support::datapath(jsonFile));

    Stage* pcl_block(f.createStage("filters.pclblock"));
    EXPECT_TRUE(pcl_block);
    pcl_block->setOptions(filter_options);
    pcl_block->setInput(*reader);

    PointTable table;
    pcl_block->prepare(table);
    PointViewSet viewSet = pcl_block->execute(table);

    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), expectedPointCount);
}


TEST(PCLBlockFilterTest, PCLBlockFilterTest_example_PassThrough_1)
{
    test_filter("filters/pcl/example_PassThrough_1.json", 81);
}


TEST(PCLBlockFilterTest, PCLBlockFilterTest_example_PassThrough_2)
{
    test_filter("filters/pcl/example_PassThrough_2.json", 50);
}

//
// For the filter tests, we attempt to verify that each parameter "works" (as
// defined by affecting at least one point, using a setting other than the
// default).
//

TEST(PCLBlockFilterTest, PCLBlockFilterTest_filter_GridMinimum)
{
    test_filter("filters/pcl/filter_GridMinimum.json", 19);
}

TEST(PCLBlockFilterTest, PCLBlockFilterTest_filter_PassThrough)
{
    // test FilterLimits for Z
    test_filter("filters/pcl/filter_PassThrough_1.json", 81);

    // test FilterLimits for X
    test_filter("filters/pcl/filter_PassThrough_2.json", 33);
}

TEST(PCLBlockFilterTest, PCLBlockFilterTest_filter_VoxelGrid)
{
    // test LeafSize
    test_filter("filters/pcl/filter_VoxelGrid.json", 81);
}
