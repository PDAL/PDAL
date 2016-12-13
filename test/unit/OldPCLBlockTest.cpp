/******************************************************************************
* Copyright (c) 2016, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <string>

#include <pdal/pdal_test_main.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(OldPCLBlockTests, StatisticalOutliers1)
{
    StageFactory f;
    
    Options ro;
    ro.add("filename", Support::datapath("autzen/autzen-point-format-3.las"));
    
    Stage* r(f.createStage("readers.las"));
    EXPECT_TRUE(r);
    r->setOptions(ro);
    
    Options fo;
    fo.add("method", "statistical");
    fo.add("multiplier", 1.5);
    fo.add("mean_k", 2);
    fo.add("extract", true);
    
    Stage* outlier(f.createStage("filters.outlier"));
    EXPECT_TRUE(outlier);
    outlier->setOptions(fo);
    outlier->setInput(*r);
    
    PointTable table;
    outlier->prepare(table);
    PointViewSet viewSet = outlier->execute(table);
    
    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(96u, view->size());
}

TEST(OldPCLBlockTests, StatisticalOutliers2)
{
    StageFactory f;
    
    Options ro;
    ro.add("filename", Support::datapath("autzen/autzen-point-format-3.las"));
    
    Stage* r(f.createStage("readers.las"));
    EXPECT_TRUE(r);
    r->setOptions(ro);
    
    Options fo;
    fo.add("method", "statistical");
    fo.add("multiplier", 0.0);
    fo.add("mean_k", 5);
    fo.add("extract", true);
    
    Stage* outlier(f.createStage("filters.outlier"));
    EXPECT_TRUE(outlier);
    outlier->setOptions(fo);
    outlier->setInput(*r);
    
    PointTable table;
    outlier->prepare(table);
    PointViewSet viewSet = outlier->execute(table);
    
    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(63u, view->size());
}

TEST(OldPCLBlockTests, RadiusOutliers1)
{
    StageFactory f;
    
    Options ro;
    ro.add("filename", Support::datapath("autzen/autzen-point-format-3.las"));
    
    Stage* r(f.createStage("readers.las"));
    EXPECT_TRUE(r);
    r->setOptions(ro);
    
    Options fo;
    fo.add("method", "radius");
    fo.add("radius", 200.0);
    fo.add("min_k", 1);
    fo.add("extract", true);
    
    Stage* outlier(f.createStage("filters.outlier"));
    EXPECT_TRUE(outlier);
    outlier->setOptions(fo);
    outlier->setInput(*r);
    
    PointTable table;
    outlier->prepare(table);
    PointViewSet viewSet = outlier->execute(table);
    
    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(60u, view->size());
}

TEST(OldPCLBlockTests, RadiusOutliers2)
{
    StageFactory f;
    
    Options ro;
    ro.add("filename", Support::datapath("autzen/autzen-point-format-3.las"));
    
    Stage* r(f.createStage("readers.las"));
    EXPECT_TRUE(r);
    r->setOptions(ro);
    
    Options fo;
    fo.add("method", "radius");
    fo.add("radius", 100.0);
    fo.add("min_k", 2);
    fo.add("extract", true);
    
    Stage* outlier(f.createStage("filters.outlier"));
    EXPECT_TRUE(outlier);
    outlier->setOptions(fo);
    outlier->setInput(*r);
    
    PointTable table;
    outlier->prepare(table);
    PointViewSet viewSet = outlier->execute(table);
    
    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(3u, view->size());
}

TEST(OldPCLBlockTests, PMF1)
{
    StageFactory f;
    
    Options ro;
    ro.add("filename", Support::datapath("autzen/autzen-point-format-3.las"));
    
    Stage* r(f.createStage("readers.las"));
    EXPECT_TRUE(r);
    r->setOptions(ro);
    
    Options fo;
    fo.add("max_window_size", 200);
    fo.add("extract", true);
    
    Stage* outlier(f.createStage("filters.pmf"));
    EXPECT_TRUE(outlier);
    outlier->setOptions(fo);
    outlier->setInput(*r);
    
    PointTable table;
    outlier->prepare(table);
    PointViewSet viewSet = outlier->execute(table);
    
    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(93u, view->size());
}

TEST(OldPCLBlockTests, PMF2)
{
    StageFactory f;
    
    Options ro;
    ro.add("filename", Support::datapath("autzen/autzen-point-format-3.las"));
    
    Stage* r(f.createStage("readers.las"));
    EXPECT_TRUE(r);
    r->setOptions(ro);
    
    Options fo;
    fo.add("max_window_size", 200);
    fo.add("cell_size", 1.0);
    fo.add("slope", 1.0);
    fo.add("initial_distance", 0.05);
    fo.add("max_distance", 3.0);
    fo.add("extract", true);
    
    Stage* outlier(f.createStage("filters.pmf"));
    EXPECT_TRUE(outlier);
    outlier->setOptions(fo);
    outlier->setInput(*r);
    
    PointTable table;
    outlier->prepare(table);
    PointViewSet viewSet = outlier->execute(table);
    
    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(94u, view->size());
}

TEST(OldPCLBlockTests, PMF3)
{
    StageFactory f;
    
    Options ro;
    ro.add("filename", Support::datapath("autzen/autzen-point-format-3.las"));
    
    Stage* r(f.createStage("readers.las"));
    EXPECT_TRUE(r);
    r->setOptions(ro);
    
    Options fo;
    fo.add("max_window_size", 33);
    fo.add("cell_size", 1.0);
    fo.add("slope", 1.0);
    fo.add("initial_distance", 0.15);
    fo.add("max_distance", 2.5);
    fo.add("extract", true);
    
    Stage* outlier(f.createStage("filters.pmf"));
    EXPECT_TRUE(outlier);
    outlier->setOptions(fo);
    outlier->setInput(*r);
    
    PointTable table;
    outlier->prepare(table);
    PointViewSet viewSet = outlier->execute(table);
    
    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(106u, view->size());
}

/*
// Test these with "autzen/autzen-thin.las"
TEST(PCLBlockFilterTest, PCLBlockFilterTest_filter_PMF)
{
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
*/
