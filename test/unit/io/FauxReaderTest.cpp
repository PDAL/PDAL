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

#include <io/FauxReader.hpp>

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

        EXPECT_DOUBLE_EQ(x, 1.0);
        EXPECT_DOUBLE_EQ(y, 2.0);
        EXPECT_DOUBLE_EQ(z, 3.0);
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

    EXPECT_DOUBLE_EQ(x0, 0.0);
    EXPECT_DOUBLE_EQ(y0, 0.0);
    EXPECT_DOUBLE_EQ(z0, 0.0);
    EXPECT_EQ(t0, 0u);

    EXPECT_DOUBLE_EQ(x1, 4.0);
    EXPECT_DOUBLE_EQ(y1, 4.0);
    EXPECT_DOUBLE_EQ(z1, 4.0);
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

        EXPECT_DOUBLE_EQ(x, 1.0 + delX * i);
        EXPECT_DOUBLE_EQ(y, 2.0 + delY * i);
        EXPECT_DOUBLE_EQ(z, 3.0 + delZ * i);
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

TEST(FauxReaderTest, one_point)
{
    Options ops;

    ops.add("bounds", BOX3D(1, 2, 3, 1, 2, 3));
    ops.add("count", 1);
    ops.add("mode", "ramp");
    FauxReader reader;
    reader.setOptions(ops);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 1u);

    EXPECT_EQ(1, view->getFieldAs<int>(Dimension::Id::X, 0));
    EXPECT_EQ(2, view->getFieldAs<int>(Dimension::Id::Y, 0));
    EXPECT_EQ(3, view->getFieldAs<int>(Dimension::Id::Z, 0));
}

TEST(FauxReaderTest, uniform)
{
    Options ops;

    ops.add("bounds", BOX3D(0, 0, 0, 100, 100, 100));
    ops.add("count", 1000);
    ops.add("seed", 2121212);
    ops.add("mode", "uniform");
    FauxReader reader;
    reader.setOptions(ops);

    PointTable t;
    reader.prepare(t);
    PointViewSet vs = reader.execute(t);
    EXPECT_EQ(vs.size(), 1u);
    PointViewPtr v = *vs.begin();
    EXPECT_EQ(v->size(), 1000u);
    double hx[10] {};
    double hy[10] {};
    double hz[10] {};
    for (size_t i = 0; i < 1000; ++i)
    {
        int x = (int)(v->getFieldAs<double>(Dimension::Id::X, i) / 10);
        int y = (int)(v->getFieldAs<double>(Dimension::Id::Y, i) / 10);
        int z = (int)(v->getFieldAs<double>(Dimension::Id::Z, i) / 10);
        hx[x]++;
        hy[y]++;
        hz[z]++;
    }

    int xtot[] = { 117, 95, 94, 93, 90, 118, 102, 97, 102, 92 };
    int ytot[] = { 92, 110, 105, 100, 98, 114, 83, 93, 108, 97 };
    int ztot[] = { 92, 99, 106, 100, 105, 106, 109, 88, 84, 111 };

    for (size_t i = 0; i < 10; ++i)
    {
        EXPECT_EQ(hx[i], xtot[i]);
        EXPECT_EQ(hy[i], ytot[i]);
        EXPECT_EQ(hz[i], ztot[i]);
    }
}

TEST(FauxReaderTest, normal)
{
    Options ops;

    ops.add("count", 1000);
    ops.add("seed", 2121212);
    ops.add("mode", "normal");
    ops.add("mean_x", 50);
    ops.add("mean_y", 100);
    ops.add("mean_z", 150);
    ops.add("stdev_x", 10);
    ops.add("stdev_y", 10);
    ops.add("stdev_z", 10);
    FauxReader reader;
    reader.setOptions(ops);

    PointTable t;
    reader.prepare(t);
    PointViewSet vs = reader.execute(t);
    EXPECT_EQ(vs.size(), 1u);
    PointViewPtr v = *vs.begin();
    EXPECT_EQ(v->size(), 1000u);
    double hx[10] {};
    double hy[10] {};
    double hz[10] {};
    for (size_t i = 0; i < 1000; ++i)
    {
        int x = v->getFieldAs<int>(Dimension::Id::X, i) / 10;
        int y = (v->getFieldAs<int>(Dimension::Id::Y, i) - 50) / 10;
        int z = (v->getFieldAs<int>(Dimension::Id::Z, i) - 100) / 10;
        x = Utils::clamp(x, 0, 9);
        y = Utils::clamp(y, 0, 9);
        z = Utils::clamp(z, 0, 9);
        hx[x]++;
        hy[y]++;
        hz[z]++;
    }

    int xtot[] = { 0, 3, 19, 145, 313, 340, 156, 22, 2, 0 };
    int ytot[] = { 0, 1, 23, 129, 355, 333, 134, 23, 2, 0 };
    int ztot[] = { 0, 0, 20, 131, 339, 357, 118, 31, 4, 0 };

    for (size_t i = 0; i < 10; ++i)
    {
        EXPECT_EQ(hx[i], xtot[i]);
        EXPECT_EQ(hy[i], ytot[i]);
        EXPECT_EQ(hz[i], ztot[i]);
    }
}

TEST(FauxReaderTest, badseed)
{
    Options ops;

    ops.add("mode", "ramp");
    ops.add("count", 100);
    ops.add("seed", 100);
    ops.add("bounds", BOX3D(1, 100, 1, 100, 1, 100));
    FauxReader r;
    r.setOptions(ops);

    PointTable t;
    EXPECT_THROW(r.prepare(t), pdal_error);
}

void testGrid(point_count_t xlimit, point_count_t ylimit, point_count_t zlimit)
{
    point_count_t size = 1;
    if (xlimit)
        size *= xlimit;
    if (ylimit)
        size *= ylimit;
    if (zlimit)
        size *= zlimit;
    if (!xlimit && !ylimit && !zlimit)
        return;

    Options ops;

    ops.add("bounds", BOX3D(0, 0, 0,
        (double)xlimit, (double)ylimit, (double)zlimit));
    ops.add("mode", "grid");
    FauxReader reader;
    reader.setOptions(ops);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), size);

    PointId index = 0;
    int x = 0;
    int y = 0;
    int z = 0;
    for (PointId index =  0; index < size; index++)
    {
        EXPECT_EQ(x, view->getFieldAs<int>(Dimension::Id::X, index));
        EXPECT_EQ(y, view->getFieldAs<int>(Dimension::Id::Y, index));
        EXPECT_EQ(z, view->getFieldAs<int>(Dimension::Id::Z, index));
        bool incNext = true;
        if (xlimit)
        {
            x++;
            if (x >= (int)xlimit)
                x = 0;
            else
                incNext = false;
        }

        if (ylimit && incNext)
        {
            y++;
            if (y >= (int)ylimit)
                y = 0;
            else
                incNext = false;
        }

        if (zlimit && incNext)
            z++;
    }
}

TEST(FauxReaderTest, grid)
{
    testGrid(2, 3, 4);
    testGrid(0, 3, 4);
    testGrid(2, 0, 4);
    testGrid(2, 3, 0);
    testGrid(2, 0, 0);
    testGrid(0, 3, 0);
    testGrid(0, 3, 4);
}
