/******************************************************************************
 * Copyright (c) 2016-2017, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/StageFactory.hpp>
#include <pdal/pdal_test_main.hpp>

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

    Stage* outlier(f.createStage("filters.outlier"));
    EXPECT_TRUE(outlier);
    outlier->setOptions(fo);
    outlier->setInput(*r);

    Options rangeOpts;
    rangeOpts.add("limits", "Classification![7:7]");

    Stage* range(f.createStage("filters.range"));
    EXPECT_TRUE(range);
    range->setOptions(rangeOpts);
    range->setInput(*outlier);

    PointTable table;
    range->prepare(table);
    PointViewSet viewSet = range->execute(table);

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

    Stage* outlier(f.createStage("filters.outlier"));
    EXPECT_TRUE(outlier);
    outlier->setOptions(fo);
    outlier->setInput(*r);

    Options rangeOpts;
    rangeOpts.add("limits", "Classification![7:7]");

    Stage* range(f.createStage("filters.range"));
    EXPECT_TRUE(range);
    range->setOptions(rangeOpts);
    range->setInput(*outlier);

    PointTable table;
    range->prepare(table);
    PointViewSet viewSet = range->execute(table);

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

    Stage* outlier(f.createStage("filters.outlier"));
    EXPECT_TRUE(outlier);
    outlier->setOptions(fo);
    outlier->setInput(*r);

    Options rangeOpts;
    rangeOpts.add("limits", "Classification![7:7]");

    Stage* range(f.createStage("filters.range"));
    EXPECT_TRUE(range);
    range->setOptions(rangeOpts);
    range->setInput(*outlier);

    PointTable table;
    range->prepare(table);
    PointViewSet viewSet = range->execute(table);

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

    Stage* outlier(f.createStage("filters.outlier"));
    EXPECT_TRUE(outlier);
    outlier->setOptions(fo);
    outlier->setInput(*r);

    Options rangeOpts;
    rangeOpts.add("limits", "Classification![7:7]");

    Stage* range(f.createStage("filters.range"));
    EXPECT_TRUE(range);
    range->setOptions(rangeOpts);
    range->setInput(*outlier);

    PointTable table;
    range->prepare(table);
    PointViewSet viewSet = range->execute(table);

    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(3u, view->size());
}

TEST(OldPCLBlockTests, PMF)
{
    StageFactory f;

    Options ro;
    ro.add("filename", Support::datapath("autzen/autzen-point-format-3.las"));

    Stage* r(f.createStage("readers.las"));
    EXPECT_TRUE(r);
    r->setOptions(ro);

    Options ao;
    ao.add("assignment", "Classification[:]=0");

    Stage* assign(f.createStage("filters.assign"));
    EXPECT_TRUE(assign);
    assign->setOptions(ao);
    assign->setInput(*r);

    Options fo;
    fo.add("max_window_size", 33.0);
    fo.add("cell_size", 10.0);
    fo.add("slope", 1.0);
    fo.add("initial_distance", 0.15);
    fo.add("max_distance", 2.5);
    fo.add("returns", "first");
    fo.add("returns", "intermediate");
    fo.add("returns", "last");
    fo.add("returns", "only");

    Stage* outlier(f.createStage("filters.pmf"));
    EXPECT_TRUE(outlier);
    outlier->setOptions(fo);
    outlier->setInput(*assign);

    Options rangeOpts;
    rangeOpts.add("limits", "Classification[2:2]");

    Stage* range(f.createStage("filters.range"));
    EXPECT_TRUE(range);
    range->setOptions(rangeOpts);
    range->setInput(*outlier);

    PointTable table;
    range->prepare(table);
    PointViewSet viewSet = range->execute(table);

    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(79u, view->size());
}
