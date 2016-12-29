/******************************************************************************
* Copyright (c) 2016, Bradley J. Chambers (brad.chambers@gmail.com)
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

#include <pdal/Dimension.hpp>
#include <pdal/pdal_test_main.hpp>
#include <pdal/pdal_defines.h>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>

#include <pdal/Segmentation.hpp>

#include <vector>

using namespace pdal;

TEST(SegmentationTest, BasicClustering)
{
    using namespace Segmentation;

    std::vector<std::vector<PointId>> clusters;

    PointTable table;
    PointLayoutPtr layout(table.layout());

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);

    PointViewPtr src(new PointView(table));

    // Single point, single cluster
    src->setField(Dimension::Id::X, 0, 0.0);
    src->setField(Dimension::Id::Y, 0, 0.0);
    src->setField(Dimension::Id::Z, 0, 0.0);
    clusters = extractClusters(*src, 1, 10, 1.0);
    EXPECT_EQ(1u, clusters.size());
    EXPECT_EQ(1u, clusters[0].size());

    // Two separate clusters, both with single point
    src->setField(Dimension::Id::X, 1, 10.0);
    src->setField(Dimension::Id::Y, 1, 10.0);
    src->setField(Dimension::Id::Z, 1, 10.0);
    clusters = extractClusters(*src, 1, 10, 1.0);
    EXPECT_EQ(2u, clusters.size());
    EXPECT_EQ(1u, clusters[0].size());
    EXPECT_EQ(1u, clusters[1].size());

    // Still two clusters, one with two points
    src->setField(Dimension::Id::X, 2, 0.5);
    src->setField(Dimension::Id::Y, 2, 0.5);
    src->setField(Dimension::Id::Z, 2, 0.5);
    clusters = extractClusters(*src, 1, 10, 1.0);
    EXPECT_EQ(2u, clusters.size());
    EXPECT_EQ(2u, clusters[0].size());
    EXPECT_EQ(1u, clusters[1].size());

    // Reject the cluster with only one point
    clusters = extractClusters(*src, 2, 10, 1.0);
    EXPECT_EQ(1u, clusters.size());
    EXPECT_EQ(2u, clusters[0].size());

    // Reject the cluster with two points
    clusters = extractClusters(*src, 1, 1, 1.0);
    EXPECT_EQ(1u, clusters.size());
    EXPECT_EQ(1u, clusters[0].size());
}
