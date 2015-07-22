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
#include <pdal/PipelineReader.hpp>
#include <pdal/PluginManager.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"

#undef RUN_SLOW_TESTS

using namespace pdal;

TEST(PCLBlockFilterTest, PCLBlockFilterTest_example_passthrough_xml)
{
    StageFactory f;
    std::unique_ptr<Stage> filter(f.createStage("filters.pclblock"));
    EXPECT_TRUE(filter.get());

    PipelineManager pipeline;
    PipelineReader pipelineReader(pipeline);
    pipelineReader.readPipeline(
        Support::datapath("filters/pcl/passthrough.xml"));
    pipeline.execute();

    PointViewSet viewSet = pipeline.views();
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 81u);
}


static void test_filter(const std::string& jsonFile,
                        size_t expectedPointCount,
                        bool useThin=false)
{
    StageFactory f;
    Options options;

    const std::string& autzenThick = "autzen/autzen-point-format-3.las";
    const std::string& autzenThin = "autzen/autzen-thin.las";
    const std::string& autzen = useThin ? autzenThin : autzenThick;

    Option filename("filename", Support::datapath(autzen));
    Option debug("debug", true, "");
    Option verbose("verbose", 9, "");

    options.add(filename);
    options.add(debug);
    options.add(verbose);

    std::unique_ptr<Stage> reader(f.createStage("readers.las"));
    EXPECT_TRUE(reader.get());
    reader->setOptions(options);

    Option fname("filename", Support::datapath(jsonFile));
    Options filter_options;
    filter_options.add(fname);

    std::shared_ptr<Stage> pcl_block(f.createStage("filters.pclblock"));
    EXPECT_TRUE(pcl_block.get());
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

TEST(PCLBlockFilterTest, PCLBlockFilterTest_example_PMF_1)
{
    test_filter("filters/pcl/example_PMF_1.json", 93);
}


TEST(PCLBlockFilterTest, PCLBlockFilterTest_example_PMF_2)
{
    test_filter("filters/pcl/example_PMF_2.json", 94);
}

//
// For the filter tests, we attempt to verify that each parameter "works" (as
// defined by affecting at least one point, using a setting other than the
// default).
//

TEST(PCLBlockFilterTest, PCLBlockFilterTest_filter_APMF)
{
    // BUG: tests still to be developed (seems to either hang or crash inside
    // Eigen)

    //test_filter("filters/pcl/filter_APMF_1.json", 106, false);
}

/*
TEST(PCLBlockFilterTest, PCLBlockFilterTest_filter_ConditionalRemoval)
{
    // NormalEstimation: KSearch 0, RadiusSearch 50
    // ConditionalRemoval: (0.0, 0.087156)
    test_filter("filters/pcl/filter_ConditionalRemoval_1.json", 158, true);

    // NormalEstimation: KSearch 0, RadiusSearch 50
    // ConditionalRemoval: (0.01, 0.10)
    test_filter("filters/pcl/filter_ConditionalRemoval_2.json", 160, true);
}
*/

TEST(PCLBlockFilterTest, PCLBlockFilterTest_filter_GridMinimum)
{
    test_filter("filters/pcl/filter_GridMinimum.json", 19);
}

/*
TEST(PCLBlockFilterTest, PCLBlockFilterTest_filter_NormalEstimation)
{
    // NormalEstimation: KSearch default (0), RadiusSearch 50
    test_filter("filters/pcl/filter_NormalEstimation_1.json", 158, true);

    // NormalEstimation: KSearch default (0), RadiusSearch 51
    test_filter("filters/pcl/filter_NormalEstimation_2.json", 162, true);

    // BUG: need to test KSearch values
}
*/

TEST(PCLBlockFilterTest, PCLBlockFilterTest_filter_PassThrough)
{
    // test FilterLimits for Z
    test_filter("filters/pcl/filter_PassThrough_1.json", 81);

    // test FilterLimits for X
    test_filter("filters/pcl/filter_PassThrough_2.json", 33);
}

TEST(PCLBlockFilterTest, PCLBlockFilterTest_filter_PMF)
{
    // explicitly with all defaults
    // (with the default autzen file, this isn't a meaningful test: it will just
    // verify the filter is minimally functioning)
    test_filter("filters/pcl/filter_PMF_1.json", 106, false);

#if RUN_SLOW_TESTS
    // explicitly with all defaults
    test_filter("filters/pcl/filter_PMF_1.json", 9223, true);

    // with CellSize=3
    test_filter("filters/pcl/filter_PMF_2.json", 8298, true);

    // with WindowSize=50
    test_filter("filters/pcl/filter_PMF_3.json", 7970, true);

    // with Slope=0.25
    test_filter("filters/pcl/filter_PMF_4.json", 9206, true);

    // with MaxDistance=5
    test_filter("filters/pcl/filter_PMF_5.json", 9373, true);

    // with InitialDistance=0.25
    test_filter("filters/pcl/filter_PMF_6.json", 9229, true);

    // with Base=3
    test_filter("filters/pcl/filter_PMF_7.json", 8298, true);

    // with Exponential=false
    test_filter("filters/pcl/filter_PMF_8.json", 9138, true);

    // with Negative=true
    test_filter("filters/pcl/filter_PMF_9.json", 1430, true);
#endif
}

TEST(PCLBlockFilterTest, PCLBlockFilterTest_filter_RadiusOutlierRemoval)
{
    // test MinNeighbors=1 and RadiusSearch=200
    test_filter("filters/pcl/filter_RadiusOutlierRemoval_1.json", 60);

    // test MinNeighbors=2 and RadiusSearch=100
    test_filter("filters/pcl/filter_RadiusOutlierRemoval_2.json", 3);
}

TEST(PCLBlockFilterTest, PCLBlockFilterTest_filter_StatisticalOutlierRemoval)
{
    // test StdDev=2, MeanK=1.5
    test_filter("filters/pcl/filter_StatisticalOutlierRemoval_1.json", 96);

    // test StdDev=5, MeanK=0(default)
    test_filter("filters/pcl/filter_StatisticalOutlierRemoval_2.json", 63);
}

TEST(PCLBlockFilterTest, PCLBlockFilterTest_filter_VoxelGrid)
{
    // test LeafSize
    test_filter("filters/pcl/filter_VoxelGrid.json", 81);
}
