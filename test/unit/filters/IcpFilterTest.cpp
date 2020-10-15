/******************************************************************************
 * Copyright (c) 2017, Peter J. Gadomski <pete@gadom.ski>
 *
 * All rights reserved
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

#include "Support.hpp"
#include <Eigen/Dense>
#include <filters/TransformationFilter.hpp>
#include <io/BufferReader.hpp>
#include <io/LasReader.hpp>
#include <memory>
#include <pdal/pdal_test_main.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/private/MathUtils.hpp>

namespace pdal
{
namespace
{

std::unique_ptr<LasReader> newReader()
{
    Options options;
    options.add("filename", Support::datapath("las/100-points.las"));
    std::unique_ptr<LasReader> reader(new LasReader());
    reader->setOptions(options);
    return reader;
}

Stage* newFilter()
{
    static StageFactory f;

    return f.createStage("filters.icp");
}

void checkPointsEqualReader(const PointViewSet& pointViewSet, double tolerance)
{
    auto reader = newReader();
    PointTable table;
    reader->prepare(table);
    PointViewSet readerPointViewSet = reader->execute(table);
    ASSERT_EQ(1u, readerPointViewSet.size());
    PointViewPtr expected = *readerPointViewSet.begin();
    ASSERT_EQ(1u, pointViewSet.size());
    PointViewPtr actual = *pointViewSet.begin();

    ASSERT_EQ(expected->size(), actual->size());

    for (PointId i = 0; i < expected->size(); ++i)
    {
        ASSERT_NEAR(expected->getFieldAs<double>(Dimension::Id::X, i),
                    actual->getFieldAs<double>(Dimension::Id::X, i), tolerance);
        ASSERT_NEAR(expected->getFieldAs<double>(Dimension::Id::Y, i),
                    actual->getFieldAs<double>(Dimension::Id::Y, i), tolerance);
        ASSERT_NEAR(expected->getFieldAs<double>(Dimension::Id::Z, i),
                    actual->getFieldAs<double>(Dimension::Id::Z, i), tolerance);
    }
}
}

TEST(IcpFilterTest, DefaultIdentity)
{
    auto reader1 = newReader();
    auto reader2 = newReader();
    auto filter = newFilter();
    filter->setInput(*reader1);
    filter->setInput(*reader2);

    PointTable table;
    filter->prepare(table);
    PointViewSet viewSet = filter->execute(table);
    EXPECT_EQ(1u, viewSet.size());
    EXPECT_EQ(100u, (*viewSet.begin())->size());

    MetadataNode root = filter->getMetadata();
    MetadataNode transform = root.findChild("transform");
    EXPECT_EQ("string", transform.type());
    Eigen::MatrixXd transformMatrix = transform.value<Eigen::MatrixXd>();
    EXPECT_TRUE(transformMatrix.isApprox(Eigen::MatrixXd::Identity(4, 4), 1.0));
}

TEST(IcpFilterTest, RecoverTranslation)
{
    auto reader1 = newReader();
    auto reader2 = newReader();
    TransformationFilter transformationFilter;
    Options transformationOptions;
    transformationOptions.add("matrix", "1 0 0 1\n0 1 0 2\n0 0 1 3\n0 0 0 1");
    transformationFilter.setOptions(transformationOptions);
    transformationFilter.setInput(*reader2);

    auto filter = newFilter();
    filter->setInput(*reader1);
    filter->setInput(transformationFilter);

    PointTable table;
    filter->prepare(table);
    PointViewSet pointViewSet = filter->execute(table);

    MetadataNode root = filter->getMetadata();
    Eigen::MatrixXd transform =
        root.findChild("transform").value<Eigen::MatrixXd>();
    double tolerance = 1.5;
    EXPECT_NEAR(-1.0, transform(0, 3), tolerance);
    EXPECT_NEAR(-2.0, transform(1, 3), tolerance);
    EXPECT_NEAR(-3.0, transform(2, 3), tolerance);
    checkPointsEqualReader(pointViewSet, tolerance);
}

TEST(IcpFilterTest, RecoverTranslationWithNoise)
{
    // Create two views, the second being translated by (1, 2, 3), but with a
    // large amount of noise added to the second X value. Test that max_dist
    // argument rejects this match and we come up with the correct translation.
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader reader1;
    BufferReader reader2;

    PointViewPtr view1(new PointView(table));
    view1->setField(Id::X, 0, 0);
    view1->setField(Id::X, 1, 10);
    view1->setField(Id::X, 2, 0);
    view1->setField(Id::Y, 0, 0);
    view1->setField(Id::Y, 1, 0);
    view1->setField(Id::Y, 2, 10);
    view1->setField(Id::Z, 0, 0);
    view1->setField(Id::Z, 1, 0);
    view1->setField(Id::Z, 2, 0);
    reader1.addView(view1);

    PointViewPtr view2(new PointView(table));
    view2->setField(Id::X, 0, 1);
    view2->setField(Id::X, 1, 16);
    view2->setField(Id::X, 2, 1);
    view2->setField(Id::Y, 0, 2);
    view2->setField(Id::Y, 1, 2);
    view2->setField(Id::Y, 2, 12);
    view2->setField(Id::Z, 0, 3);
    view2->setField(Id::Z, 1, 3);
    view2->setField(Id::Z, 2, 3);
    reader2.addView(view2);
    
    auto filter = newFilter();
    Options icpOptions;
    icpOptions.add("max_dist", 5.0);
    filter->setInput(reader1);
    filter->setInput(reader2);
    filter->setOptions(icpOptions);

    filter->prepare(table);
    PointViewSet pointViewSet = filter->execute(table);

    MetadataNode root = filter->getMetadata();
    Eigen::MatrixXd transform =
        root.findChild("transform").value<Eigen::MatrixXd>();
    double tolerance = 1.5;
    EXPECT_NEAR(-1.0, transform(0, 3), tolerance);
    EXPECT_NEAR(-2.0, transform(1, 3), tolerance);
    EXPECT_NEAR(-3.0, transform(2, 3), tolerance);
}

TEST(IcpFilterTest, RecoverTranslationWithNoise2)
{
    // Create two views, the second being translated by (1, 2, 3), but with a
    // large amount of noise added to the second X value. Test that without the
    // max_dist argument the X component of the translation is incorrect.
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader reader1;
    BufferReader reader2;

    PointViewPtr view1(new PointView(table));
    view1->setField(Id::X, 0, 0);
    view1->setField(Id::X, 1, 10);
    view1->setField(Id::X, 2, 0);
    view1->setField(Id::Y, 0, 0);
    view1->setField(Id::Y, 1, 0);
    view1->setField(Id::Y, 2, 10);
    view1->setField(Id::Z, 0, 0);
    view1->setField(Id::Z, 1, 0);
    view1->setField(Id::Z, 2, 0);
    reader1.addView(view1);

    PointViewPtr view2(new PointView(table));
    view2->setField(Id::X, 0, 1);
    view2->setField(Id::X, 1, 16);
    view2->setField(Id::X, 2, 1);
    view2->setField(Id::Y, 0, 2);
    view2->setField(Id::Y, 1, 2);
    view2->setField(Id::Y, 2, 12);
    view2->setField(Id::Z, 0, 3);
    view2->setField(Id::Z, 1, 3);
    view2->setField(Id::Z, 2, 3);
    reader2.addView(view2);
    
    auto filter = newFilter();
    filter->setInput(reader1);
    filter->setInput(reader2);

    filter->prepare(table);
    PointViewSet pointViewSet = filter->execute(table);

    MetadataNode root = filter->getMetadata();
    Eigen::MatrixXd transform =
        root.findChild("transform").value<Eigen::MatrixXd>();
    double tolerance = 1.5;
    EXPECT_GT(-1.0, transform(0, 3));
    EXPECT_NEAR(-2.0, transform(1, 3), tolerance);
    EXPECT_NEAR(-3.0, transform(2, 3), tolerance);
}

TEST(IcpFilterTest, RecoverTranslationWithGuess)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader reader1;
    BufferReader reader2;

    PointViewPtr view1(new PointView(table));
    view1->setField(Id::X, 0, 0);
    view1->setField(Id::X, 1, 10);
    view1->setField(Id::X, 2, 0);
    view1->setField(Id::Y, 0, 0);
    view1->setField(Id::Y, 1, 0);
    view1->setField(Id::Y, 2, 10);
    view1->setField(Id::Z, 0, 0);
    view1->setField(Id::Z, 1, 0);
    view1->setField(Id::Z, 2, 0);
    reader1.addView(view1);

    PointViewPtr view2(new PointView(table));
    view2->setField(Id::X, 0, 1);
    view2->setField(Id::X, 1, 11);
    view2->setField(Id::X, 2, 1);
    view2->setField(Id::Y, 0, 2);
    view2->setField(Id::Y, 1, 2);
    view2->setField(Id::Y, 2, 12);
    view2->setField(Id::Z, 0, 3);
    view2->setField(Id::Z, 1, 3);
    view2->setField(Id::Z, 2, 3);
    reader2.addView(view2);
    
    auto filter = newFilter();
    Options icpOptions;
    // Start with the actual transformation for the initial guess.
    icpOptions.add("init", "1 0 0 -1 0 1 0 -2 0 0 1 -3 0 0 0 1");
    filter->setInput(reader1);
    filter->setInput(reader2);
    filter->setOptions(icpOptions);

    filter->prepare(table);
    PointViewSet pointViewSet = filter->execute(table);

    MetadataNode root = filter->getMetadata();
    Eigen::MatrixXd transform =
        root.findChild("transform").value<Eigen::MatrixXd>();
    double tolerance = 1.5;
    EXPECT_NEAR(-1.0, transform(0, 3), tolerance);
    EXPECT_NEAR(-2.0, transform(1, 3), tolerance);
    EXPECT_NEAR(-3.0, transform(2, 3), tolerance);
}

TEST(IcpFilterTest, RecoverTranslationWithBadGuess)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader reader1;
    BufferReader reader2;

    PointViewPtr view1(new PointView(table));
    view1->setField(Id::X, 0, 0);
    view1->setField(Id::X, 1, 10);
    view1->setField(Id::X, 2, 0);
    view1->setField(Id::Y, 0, 0);
    view1->setField(Id::Y, 1, 0);
    view1->setField(Id::Y, 2, 10);
    view1->setField(Id::Z, 0, 0);
    view1->setField(Id::Z, 1, 0);
    view1->setField(Id::Z, 2, 0);
    reader1.addView(view1);

    PointViewPtr view2(new PointView(table));
    view2->setField(Id::X, 0, 1);
    view2->setField(Id::X, 1, 11);
    view2->setField(Id::X, 2, 1);
    view2->setField(Id::Y, 0, 2);
    view2->setField(Id::Y, 1, 2);
    view2->setField(Id::Y, 2, 12);
    view2->setField(Id::Z, 0, 3);
    view2->setField(Id::Z, 1, 3);
    view2->setField(Id::Z, 2, 3);
    reader2.addView(view2);
    
    auto filter = newFilter();
    Options icpOptions;
    // Provide a bad initial guess for the tranformation.
    icpOptions.add("init", "0.996 0 0.087 -50 0 1 0 100 0.996 0 0.087 -300 0 0 0 1");
    filter->setInput(reader1);
    filter->setInput(reader2);
    filter->setOptions(icpOptions);

    filter->prepare(table);
    PointViewSet pointViewSet = filter->execute(table);

    MetadataNode root = filter->getMetadata();
    Eigen::MatrixXd transform =
        root.findChild("transform").value<Eigen::MatrixXd>();
    double tolerance = 1.5;
    // No reason to check exact values, only that they are LT/GT the expected values.
    EXPECT_GT(-1.0, transform(0, 3));
    EXPECT_LT(-2.0, transform(1, 3));
    EXPECT_GT(-3.0, transform(2, 3));
}

TEST(IcpFilterTest, RecoverTranslationWithMalformedGuess)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader reader1;
    BufferReader reader2;

    PointViewPtr view1(new PointView(table));
    view1->setField(Id::X, 0, 0);
    view1->setField(Id::X, 1, 10);
    view1->setField(Id::X, 2, 0);
    view1->setField(Id::Y, 0, 0);
    view1->setField(Id::Y, 1, 0);
    view1->setField(Id::Y, 2, 10);
    view1->setField(Id::Z, 0, 0);
    view1->setField(Id::Z, 1, 0);
    view1->setField(Id::Z, 2, 0);
    reader1.addView(view1);

    PointViewPtr view2(new PointView(table));
    view2->setField(Id::X, 0, 1);
    view2->setField(Id::X, 1, 11);
    view2->setField(Id::X, 2, 1);
    view2->setField(Id::Y, 0, 2);
    view2->setField(Id::Y, 1, 2);
    view2->setField(Id::Y, 2, 12);
    view2->setField(Id::Z, 0, 3);
    view2->setField(Id::Z, 1, 3);
    view2->setField(Id::Z, 2, 3);
    reader2.addView(view2);
    
    auto filter = newFilter();
    Options icpOptions;
    // Provide a malformed initial guess (too many entries).
    icpOptions.add("init", "7 1 0 0 -1 0 1 0 -2 0 0 1 -3 0 0 0 1");
    filter->setInput(reader1);
    filter->setInput(reader2);
    filter->setOptions(icpOptions);

    EXPECT_THROW(filter->prepare(table), pdal_error);
}

TEST(IcpFilterTest, RecoverRotation)
{
    auto reader1 = newReader();
    auto reader2 = newReader();
    TransformationFilter transformationFilter;
    Options transformationOptions;
    transformationOptions.add("matrix", "0.996 0 0.087 0\n0 1 0 0\n-0.087 0 0.996 0\n0 0 0 1");
    transformationFilter.setOptions(transformationOptions);
    transformationFilter.setInput(*reader2);

    auto filter = newFilter();
    filter->setInput(*reader1);
    filter->setInput(transformationFilter);

    PointTable table;
    filter->prepare(table);
    PointViewSet pointViewSet = filter->execute(table);

    MetadataNode root = filter->getMetadata();
    Eigen::MatrixXd transform =
        root.findChild("transform").value<Eigen::MatrixXd>();
    double tolerance = 0.001;
    EXPECT_NEAR(0.996, transform(0, 0), tolerance);
    EXPECT_NEAR(-0.087, transform(0, 2), tolerance);
    EXPECT_NEAR(0.087, transform(2, 0), tolerance);
    EXPECT_NEAR(0.996, transform(2, 2), tolerance);
    double tolerance2 = 0.5;
    checkPointsEqualReader(pointViewSet, tolerance2);
}

TEST(IcpFilterTest, TooFewInputs)
{
    auto reader = newReader();
    auto filter = newFilter();
    filter->setInput(*reader);

    PointTable table;
    filter->prepare(table);
    ASSERT_THROW(filter->execute(table), pdal_error);
}

TEST(IcpFilterTest, ThreeInputs)
{
    auto reader1 = newReader();
    auto reader2 = newReader();
    auto reader3 = newReader();
    auto filter = newFilter();
    filter->setInput(*reader1);
    filter->setInput(*reader2);
    filter->setInput(*reader3);

    PointTable table;
    filter->prepare(table);
    PointViewSet pointViewSet = filter->execute(table);
    EXPECT_EQ(2ul, pointViewSet.size());
}
}
