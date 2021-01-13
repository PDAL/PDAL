/******************************************************************************
 * Copyright (c) 2020 Bradley J Chambers (brad.chambers@gmail.com)
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
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_test_main.hpp>
#include <pdal/private/MathUtils.hpp>

namespace pdal
{
using namespace Eigen;

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

    return f.createStage("filters.teaser");
}

void checkPointsEqualReader(const PointViewSet& pointViewSet, double tolerance)
{
    using namespace Dimension;

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
        ASSERT_NEAR(expected->getFieldAs<double>(Id::X, i),
                    actual->getFieldAs<double>(Id::X, i), tolerance);
        ASSERT_NEAR(expected->getFieldAs<double>(Id::Y, i),
                    actual->getFieldAs<double>(Id::Y, i), tolerance);
        ASSERT_NEAR(expected->getFieldAs<double>(Id::Z, i),
                    actual->getFieldAs<double>(Id::Z, i), tolerance);
    }
}
} // namespace

TEST(TeaserFilterTest, DefaultIdentity)
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
    MatrixXd transformMatrix = transform.value<MatrixXd>();
    EXPECT_TRUE(transformMatrix.isApprox(MatrixXd::Identity(4, 4), 1.0));
}

TEST(TeaserFilterTest, RecoverTranslation)
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
    MatrixXd transform = root.findChild("transform").value<MatrixXd>();
    double tolerance = 1.5;
    EXPECT_NEAR(-1.0, transform(0, 3), tolerance);
    EXPECT_NEAR(-2.0, transform(1, 3), tolerance);
    EXPECT_NEAR(-3.0, transform(2, 3), tolerance);
    checkPointsEqualReader(pointViewSet, tolerance);
}

TEST(TeaserFilterTest, RecoverRotation)
{
    StageFactory f;

    Options rOpts;
    rOpts.add("filename", Support::datapath("las/sample_c_thin.las"));

    Stage* fixed(f.createStage("readers.las"));
    fixed->setOptions(rOpts);

    Stage* moving(f.createStage("readers.las"));
    moving->setOptions(rOpts);

    Options translate1Opts;
    translate1Opts.add(
        "matrix",
        "1 0 0 -674568.4487 0 1 0 -1206773.638 0 0 1 -650.9486969 0 0 0 1");

    Stage* translate1(f.createStage("filters.transformation"));
    translate1->setInput(*moving);
    translate1->setOptions(translate1Opts);

    Options rotateOpts;
    rotateOpts.add("matrix",
                   "0.985 -0.174 0 0 0.174 0.985 0 0 0 0 1 0 0 0 0 1");
    Stage* rotate(f.createStage("filters.transformation"));
    rotate->setInput(*translate1);
    rotate->setOptions(rotateOpts);

    Options translate2Opts;
    translate2Opts.add(
        "matrix",
        "1 0 0 674568.4487 0 1 0 1206773.638 0 0 1 650.9486969 0 0 0 1");

    Stage* translate2(f.createStage("filters.transformation"));
    translate2->setInput(*rotate);
    translate2->setOptions(translate2Opts);

    Stage* filter(f.createStage("filters.teaser"));
    filter->setInput(*fixed);
    filter->setInput(*translate2);

    PointTable t;
    filter->prepare(t);
    PointViewSet s = filter->execute(t);

    MetadataNode root = filter->getMetadata();
    MatrixXd transform = root.findChild("transform").value<MatrixXd>();
    double mse = root.findChild("fitness").value<double>();
    double tolerance = 0.001;
    EXPECT_NEAR(mse, 0.005, tolerance);
    EXPECT_NEAR(0.985, transform(0, 0), tolerance);
    EXPECT_NEAR(0.174, transform(0, 1), tolerance);
    EXPECT_NEAR(-0.174, transform(1, 0), tolerance);
    EXPECT_NEAR(0.985, transform(1, 1), tolerance);
    EXPECT_NEAR(0.0, transform(0, 3), tolerance);
    EXPECT_NEAR(0.0, transform(1, 3), tolerance);
    EXPECT_NEAR(0.0, transform(2, 3), tolerance);
}

TEST(TeaserFilterTest, RecoverRandomRotation)
{
    StageFactory f;

    Options rOpts;
    rOpts.add("filename", Support::datapath("las/sample_c_thin.las"));

    Stage* fixed(f.createStage("readers.las"));
    fixed->setOptions(rOpts);

    Stage* moving(f.createStage("readers.las"));
    moving->setOptions(rOpts);

    Options translate1Opts;
    translate1Opts.add(
        "matrix",
        "1 0 0 -674568.4487 0 1 0 -1206773.638 0 0 1 -650.9486969 0 0 0 1");

    Stage* translate1(f.createStage("filters.transformation"));
    translate1->setInput(*moving);
    translate1->setOptions(translate1Opts);

    // Maybe random is not really what we want. It is nice that it always seems
    // to work, but could there be a situation in which TEASER++ does fail? Or
    // fails to fall below the threshold MSE?
    Quaterniond q = Quaterniond::UnitRandom();
    Affine3d T;
    T.matrix().block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    IOFormat MetadataFmt(FullPrecision, DontAlignCols, " ", " ", "", "", "",
                         "");
    std::stringstream ss;
    ss << T.matrix().format(MetadataFmt);

    Options rotateOpts;
    rotateOpts.add("matrix", ss.str());
    Stage* rotate(f.createStage("filters.transformation"));
    rotate->setInput(*translate1);
    rotate->setOptions(rotateOpts);

    Options translate2Opts;
    translate2Opts.add(
        "matrix",
        "1 0 0 674568.4487 0 1 0 1206773.638 0 0 1 650.9486969 0 0 0 1");

    Stage* translate2(f.createStage("filters.transformation"));
    translate2->setInput(*rotate);
    translate2->setOptions(translate2Opts);

    Stage* filter(f.createStage("filters.teaser"));
    filter->setInput(*fixed);
    filter->setInput(*translate2);

    PointTable t;
    filter->prepare(t);
    PointViewSet s = filter->execute(t);

    MetadataNode root = filter->getMetadata();
    MatrixXd transform = root.findChild("transform").value<MatrixXd>();
    double mse = root.findChild("fitness").value<double>();
    EXPECT_LT(mse, 0.01);

    double tolerance = 0.001;
    for (size_t r = 0; r < 4; ++r)
        for (size_t c = 0; c < 4; ++c)
            EXPECT_NEAR(T.inverse()(r, c), transform(r, c), tolerance);
}

TEST(TeaserFilterTest, TooFewInputs)
{
    auto reader = newReader();
    auto filter = newFilter();
    filter->setInput(*reader);

    PointTable table;
    filter->prepare(table);
    ASSERT_THROW(filter->execute(table), pdal_error);
}

TEST(TeaserFilterTest, ThreeInputs)
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
} // namespace pdal
