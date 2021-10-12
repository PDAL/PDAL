//
// Created by Chloe T on 10/7/21.
//

#define NOMINMAX

#include <stdio.h>

#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"

#include "../io/Bounds4D.hpp"

namespace pdal
{

class Bounds4DTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        bounds.reset(
            BOX4D(0., 0., 0., 0.,
                      10., 10., 10., 10.)
                     );
        b = bounds.to4d();
    }
    Bounds4D bounds;
    BOX4D b;
};

TEST(Bounds4D, test_ctor4D)
{
    BOX4D b1;
    EXPECT_TRUE(b1.empty());

    b1.clear();
    BOX4D b2;
    EXPECT_EQ(b1, b2);
}

TEST(Bounds4D, test_equals)
{
    BOX4D b1(1, 2, 3, 4,
             5, 6, 7, 8);
    BOX4D b2(1, 2, 3, 4,
             5, 6, 7, 8);
    BOX4D b3(1, 2, 3, 4,
             5, 6, 7, 7);

    EXPECT_TRUE(b1 == b1);
    EXPECT_TRUE(b1 == b2);
    EXPECT_TRUE(b2 == b1);
    EXPECT_TRUE(b1 != b3);
    EXPECT_TRUE(b2 != b3);
}

TEST_F(Bounds4DTest, test_copy)
{
    BOX4D b1(b);
    EXPECT_TRUE(b1 == b);
}

TEST_F(Bounds4DTest, test_accessor)
{
    EXPECT_DOUBLE_EQ(b.minx, 0.);
    EXPECT_DOUBLE_EQ(b.miny, 0);
    EXPECT_DOUBLE_EQ(b.minz, 0);
    EXPECT_DOUBLE_EQ(b.mintm, 0);
    EXPECT_DOUBLE_EQ(b.maxx, 10);
    EXPECT_DOUBLE_EQ(b.maxy, 10);
    EXPECT_DOUBLE_EQ(b.maxz, 10);
    EXPECT_DOUBLE_EQ(b.maxtm, 10);
}

TEST_F(Bounds4DTest, test_clip)
{
    BOX4D b1(b);

    BOX4D b2(1, 1, 1, 1,
             11, 11, 11, 11);
    BOX4D b3(1, 1, 1, 1,
             10, 10, 10, 10);
    b1.clip(b2);

    EXPECT_TRUE(b1 == b3);

    BOX4D b4(2, 3, 4, 5,
             5, 6, 7, 8);
    b1.clip(b4);

    EXPECT_TRUE(b1 == b4);

    BOX4D b5(20, 20, 20, 20,
             -10, -10, -10, -10);
    b1.clip(b5);

    EXPECT_TRUE(b1 == b4);

    BOX4D b6(6, 4, 8, 6,
             4, 2, 6, 4);
    BOX4D b7(2, 4, 4, 6,
             4, 6, 6, 8);
    b1.clip(b6);

    EXPECT_TRUE(b1 == b7);
}

TEST_F(Bounds4DTest, test_to4d)
{
    BOX4D b1(bounds.to4d());
    EXPECT_TRUE(b1 == b);
}

TEST_F(Bounds4DTest, test_to3d)
{
    BOX3D b1(bounds.to3d());
    BOX3D b2(b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz);
    EXPECT_TRUE(b1 == b2);
}

TEST_F(Bounds4DTest, test_to2d)
{
    BOX2D b1(bounds.to2d());
    BOX2D b2(b.minx, b.miny, b.maxx, b.maxy);
    EXPECT_TRUE(b1 == b2);
}

}