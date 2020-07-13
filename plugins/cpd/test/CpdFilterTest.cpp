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
#include <filters/CpdFilter.hpp>
#include <filters/TransformationFilter.hpp>
#include <io/LasReader.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_test_main.hpp>
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

std::unique_ptr<CpdFilter> newFilter()
{
    return std::unique_ptr<CpdFilter>(new CpdFilter());
}

void checkNoThrow(const std::string& method)
{
    auto reader1 = newReader();
    auto reader2 = newReader();
    auto filter = newFilter();
    filter->setInput(*reader1);
    filter->setInput(*reader2);

    Options options;
    options.add("method", method);
    filter->setOptions(options);

    PointTable table;
    filter->prepare(table);
    ASSERT_NO_THROW(filter->execute(table));
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

TEST(CpdFilterTest, DefaultMethod)
{
    EXPECT_EQ("rigid", CpdFilter::defaultMethod());
}

TEST(CpdFilterTest, DefaultIdentity)
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
    EXPECT_EQ(CpdFilter::defaultMethod(), root.findChild("method").value());
    MetadataNode transform = root.findChild("transform");
    EXPECT_EQ("matrix", transform.type());
    Eigen::MatrixXd transformMatrix = transform.value<Eigen::MatrixXd>();
    EXPECT_TRUE(
        transformMatrix.isApprox(Eigen::MatrixXd::Identity(4, 4), 1e-4));
}

TEST(CpdFilterTest, RecoverTranslation)
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
    double tolerance = 1e-4;
    EXPECT_NEAR(-1.0, transform(0, 3), tolerance);
    EXPECT_NEAR(-2.0, transform(1, 3), tolerance);
    EXPECT_NEAR(-3.0, transform(2, 3), tolerance);
    checkPointsEqualReader(pointViewSet, tolerance);
}

TEST(CpdFilterTest, TooFewInputs)
{
    auto reader = newReader();
    auto filter = newFilter();
    filter->setInput(*reader);

    PointTable table;
    filter->prepare(table);
    ASSERT_THROW(filter->execute(table), pdal_error);
}

TEST(CpdFilterTest, TooManyInputs)
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
    ASSERT_THROW(filter->execute(table), pdal_error);
}

TEST(CpdFilterTest, RigidNoThrow)
{
    checkNoThrow("rigid");
}

TEST(CpdFilterTest, AffineNoThrow)
{
    checkNoThrow("affine");
}

TEST(CpdFilterTest, NonrigidNoThrow)
{
    checkNoThrow("nonrigid");
}
} // namespace pdal
