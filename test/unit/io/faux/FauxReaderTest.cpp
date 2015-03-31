/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <FauxReader.hpp>

using namespace pdal;

TEST(FauxReaderTest, test_constant_mode_sequential_iter)
{
    Options ops;

    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    ops.add("bounds", bounds);
    ops.add("count", 750);
    ops.add("mode", "constant");
    std::shared_ptr<FauxReader> reader(new FauxReader);
    reader->setOptions(ops);

    PointTable table;
    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 750u);
    for (point_count_t i = 0; i < view->size(); i++)
    {
        double x = view->getFieldAs<double>(Dimension::Id::X, i);
        double y = view->getFieldAs<double>(Dimension::Id::Y, i);
        double z = view->getFieldAs<double>(Dimension::Id::Z, i);
        uint64_t t = view->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, i);

        EXPECT_FLOAT_EQ(x, 1.0);
        EXPECT_FLOAT_EQ(y, 2.0);
        EXPECT_FLOAT_EQ(z, 3.0);
        EXPECT_EQ(t, i);
    }
}


TEST(FauxReaderTest, test_random_mode)
{
    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 750);
    ops.add("mode", "constant");
    std::shared_ptr<FauxReader> reader(new FauxReader);
    reader->setOptions(ops);

    PointTable table;
    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 750u);

    for (point_count_t i = 0; i < view->size(); ++i)
    {
        double x = view->getFieldAs<double>(Dimension::Id::X, i);
        double y = view->getFieldAs<double>(Dimension::Id::Y, i);
        double z = view->getFieldAs<double>(Dimension::Id::Z, i);
        uint64_t t = view->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, i);

        EXPECT_GE(x, 1.0);
        EXPECT_LE(x, 101.0);

        EXPECT_GE(y, 2.0);
        EXPECT_LE(y, 102.0);

        EXPECT_GE(z, 3.0);
        EXPECT_LE(z, 103.0);

        EXPECT_EQ(t, i);
    }
}


TEST(FauxReaderTest, test_ramp_mode_1)
{
    BOX3D bounds(0, 0, 0, 4, 4, 4);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 2);
    ops.add("mode", "ramp");

    std::shared_ptr<FauxReader> reader(new FauxReader);
    reader->setOptions(ops);

    PointTable table;
    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 2u);

    double x0 = view->getFieldAs<double>(Dimension::Id::X, 0);
    double y0 = view->getFieldAs<double>(Dimension::Id::Y, 0);
    double z0 = view->getFieldAs<double>(Dimension::Id::Z, 0);
    uint64_t t0 = view->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, 0);

    double x1 = view->getFieldAs<double>(Dimension::Id::X, 1);
    double y1 = view->getFieldAs<double>(Dimension::Id::Y, 1);
    double z1 = view->getFieldAs<double>(Dimension::Id::Z, 1);
    uint64_t t1 = view->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, 1);

    EXPECT_FLOAT_EQ(x0, 0.0);
    EXPECT_FLOAT_EQ(y0, 0.0);
    EXPECT_FLOAT_EQ(z0, 0.0);
    EXPECT_EQ(t0, 0u);

    EXPECT_FLOAT_EQ(x1, 4.0);
    EXPECT_FLOAT_EQ(y1, 4.0);
    EXPECT_FLOAT_EQ(z1, 4.0);
    EXPECT_EQ(t1, 1u);
}


TEST(FauxReaderTest, test_ramp_mode_2)
{
    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 152.0, 203.0);
    Options ops;
    ops.add("bounds", bounds);
    ops.add("count", 750);
    ops.add("mode", "ramp");
    std::shared_ptr<FauxReader> reader(new FauxReader);
    reader->setOptions(ops);

    PointTable table;
    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 750u);

    double delX = (101.0 - 1.0) / (750.0 - 1.0);
    double delY = (152.0 - 2.0) / (750.0 - 1.0);
    double delZ = (203.0 - 3.0) / (750.0 - 1.0);

    for (point_count_t i = 0; i < view->size(); ++i)
    {
        double x = view->getFieldAs<double>(Dimension::Id::X, i);
        double y = view->getFieldAs<double>(Dimension::Id::Y, i);
        double z = view->getFieldAs<double>(Dimension::Id::Z, i);
        uint64_t t = view->getFieldAs<uint64_t>(Dimension::Id::OffsetTime, i);

        EXPECT_FLOAT_EQ(x, 1.0 + delX * i);
        EXPECT_FLOAT_EQ(y, 2.0 + delY * i);
        EXPECT_FLOAT_EQ(z, 3.0 + delZ * i);
        EXPECT_EQ(t, i);
    }
}


TEST(FauxReaderTest, test_return_number)
{
    Options ops;

    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    ops.add("bounds", bounds);
    ops.add("count", 100);
    ops.add("mode", "constant");
    ops.add("number_of_returns", 9);
    std::shared_ptr<FauxReader> reader(new FauxReader);
    reader->setOptions(ops);

    PointTable table;
    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 100u);

    for (point_count_t i = 0; i < view->size(); i++)
    {
        uint8_t returnNumber = view->getFieldAs<uint8_t>(
            Dimension::Id::ReturnNumber, i);
        uint8_t numberOfReturns =
            view->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, i);

        EXPECT_EQ(returnNumber, (i % 9) + 1);
        EXPECT_EQ(numberOfReturns, 9);
    }
}
