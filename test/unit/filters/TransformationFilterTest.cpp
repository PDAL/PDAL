/******************************************************************************
* Copyright (c) 2014, Pete Gadomski <pete.gadomski@gmail.com>
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
#include <TransformationFilter.hpp>

#include <pdal/StageFactory.hpp>


namespace pdal
{


class TransformationFilterTest : public ::testing::Test
{
public:

    TransformationFilterTest()
        : factory()
        , reader(factory.createStage("readers.faux"))
        , filter(factory.createStage("filters.transformation"))
    {}

    virtual void SetUp()
    {
        StageFactory f;

        BOX3D bounds(1, 2, 3, 4, 5, 6);
        Options readerOpts;
        readerOpts.add("mode", "constant");
        readerOpts.add("num_points", 3);
        readerOpts.add("bounds", bounds);
        reader->setOptions(readerOpts);

        filter->setInput(reader.get());
    }

    StageFactory factory;
    std::unique_ptr<Stage> reader;
    std::unique_ptr<Stage> filter;

};


TEST(TransformationMatrix, FromString)
{
    std::string s = "1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1";
    TransformationMatrix m = transformationMatrixFromString(s);
    EXPECT_DOUBLE_EQ(1, m[0]);
    EXPECT_DOUBLE_EQ(0, m[1]);
    EXPECT_DOUBLE_EQ(0, m[2]);
    EXPECT_DOUBLE_EQ(0, m[3]);
    EXPECT_DOUBLE_EQ(0, m[4]);
    EXPECT_DOUBLE_EQ(1, m[5]);
    EXPECT_DOUBLE_EQ(0, m[6]);
    EXPECT_DOUBLE_EQ(0, m[7]);
    EXPECT_DOUBLE_EQ(0, m[8]);
    EXPECT_DOUBLE_EQ(0, m[9]);
    EXPECT_DOUBLE_EQ(1, m[10]);
    EXPECT_DOUBLE_EQ(0, m[11]);
    EXPECT_DOUBLE_EQ(0, m[12]);
    EXPECT_DOUBLE_EQ(0, m[13]);
    EXPECT_DOUBLE_EQ(0, m[14]);
    EXPECT_DOUBLE_EQ(1, m[15]);
}


TEST(TransformationMatrix, TooShort)
{
    std::string s = "1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0";
    EXPECT_THROW(transformationMatrixFromString(s), invalid_format);
}


TEST(TransformationMatrix, TooLong)
{
    std::string s = "1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1 0";
    EXPECT_THROW(transformationMatrixFromString(s), invalid_format);
}


TEST_F(TransformationFilterTest, NoChange)
{
    Options filterOpts;
    filterOpts.add("matrix", "1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1");
    filter->setOptions(filterOpts);

    PointContext ctx;
    filter->prepare(ctx);
    PointBufferSet pbSet = filter->execute(ctx);
    EXPECT_EQ(1u, pbSet.size());
    PointBufferPtr buf = *pbSet.begin();
    EXPECT_EQ(3u, buf->size());

    for (point_count_t i = 0; i < buf->size(); ++i)
    {
        EXPECT_DOUBLE_EQ(1, buf->getFieldAs<double>(Dimension::Id::X, i));
        EXPECT_DOUBLE_EQ(2, buf->getFieldAs<double>(Dimension::Id::Y, i));
        EXPECT_DOUBLE_EQ(3, buf->getFieldAs<double>(Dimension::Id::Z, i));
    }
}


TEST_F(TransformationFilterTest, Translation)
{
    Options filterOpts;
    filterOpts.add("matrix", "1 0 0 1\n0 1 0 2\n0 0 1 3\n0 0 0 1");
    filter->setOptions(filterOpts);

    PointContext ctx;
    filter->prepare(ctx);
    PointBufferSet pbSet = filter->execute(ctx);
    PointBufferPtr buf = *pbSet.begin();

    for (point_count_t i = 0; i < buf->size(); ++i)
    {
        EXPECT_DOUBLE_EQ(2, buf->getFieldAs<double>(Dimension::Id::X, i));
        EXPECT_DOUBLE_EQ(4, buf->getFieldAs<double>(Dimension::Id::Y, i));
        EXPECT_DOUBLE_EQ(6, buf->getFieldAs<double>(Dimension::Id::Z, i));
    }
}


TEST_F(TransformationFilterTest, Rotation)
{
    Options filterOpts;
    filterOpts.add("matrix", "0 1 0 0\n-1 0 0 0\n0 0 1 0\n0 0 0 1");
    filter->setOptions(filterOpts);

    PointContext ctx;
    filter->prepare(ctx);
    PointBufferSet pbSet = filter->execute(ctx);
    PointBufferPtr buf = *pbSet.begin();

    for (point_count_t i = 0; i < buf->size(); ++i)
    {
        EXPECT_DOUBLE_EQ(2, buf->getFieldAs<double>(Dimension::Id::X, i));
        EXPECT_DOUBLE_EQ(-1, buf->getFieldAs<double>(Dimension::Id::Y, i));
        EXPECT_DOUBLE_EQ(3, buf->getFieldAs<double>(Dimension::Id::Z, i));
    }
}


}
