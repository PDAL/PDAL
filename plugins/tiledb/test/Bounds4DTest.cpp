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

TEST(Bounds4D, test_box_ctor_empty)
{
    BOX4D b1;
    EXPECT_TRUE(b1.empty());

    b1.clear();
    BOX4D b2;
    EXPECT_EQ(b1, b2);
}

TEST_F(Bounds4DTest, test_bounds_ctor_empty)
{
    Bounds4D bounds1;
    EXPECT_TRUE(bounds1.empty());
    Bounds4D * bds1_ptr(&bounds1);
    EXPECT_TRUE(dynamic_cast<const Bounds4D*>(bds1_ptr) != nullptr);
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

TEST_F(Bounds4DTest, test_ctor_box4d)
{
    Bounds4D bounds1(b);
    Bounds4D * bds1_ptr(&bounds1);
    EXPECT_TRUE(dynamic_cast<const Bounds4D*>(bds1_ptr) != nullptr);
}

TEST_F(Bounds4DTest, test_ctor_box3d)
{
    Bounds4D bounds1(b.to3d());
    Bounds4D * bds1_ptr(&bounds1);
    EXPECT_TRUE(dynamic_cast<const Bounds4D*>(bds1_ptr) != nullptr);
}

TEST_F(Bounds4DTest, test_ctor_box2d)
{
    Bounds4D bounds1(b.to2d());
    Bounds4D * bds1_ptr(&bounds1);
    EXPECT_TRUE(dynamic_cast<const Bounds4D*>(bds1_ptr) != nullptr);
}

//TEST_F(Bounds4DTest, test_bounds_to2d)
//{
//    BOX2D b1(bounds.to2d());
//    BOX2D b2(b.minx, b.miny, b.maxx, b.maxy);
//    EXPECT_TRUE(b1 == b2);
//}

TEST_F(Bounds4DTest, test_bounds_empty)
{
    Bounds4D bounds1;
    EXPECT_TRUE(bounds1.empty());
}

TEST_F(Bounds4DTest, test_bounds_valid)
{
    Bounds4D bounds1;
    EXPECT_FALSE(bounds1.valid());
    bounds1 = bounds;
    EXPECT_TRUE(bounds1.valid());
}

TEST_F(Bounds4DTest, test_is4d_3d_2d)
{
    Bounds4D bounds4d(bounds);
    Bounds4D bounds3d(bounds.to3d());
    Bounds4D bounds2d(bounds.to2d());

    EXPECT_TRUE(bounds4d.is4d());
    EXPECT_TRUE(bounds3d.is3d());
    EXPECT_TRUE(bounds2d.is2d());
}

TEST_F(Bounds4DTest, test_reset)
{
    Bounds4D bounds1(b);
    BOX4D b1(1, 2, 3, 4,
             5, 6, 7, 8);
    bounds1.reset(b1);
    EXPECT_TRUE(bounds1.is4d());
    EXPECT_FALSE(bounds1.is3d());
    EXPECT_FALSE(bounds1.is2d());
    EXPECT_TRUE(bounds1.to4d() == b1);

    BOX3D b2(2, 3, 4, 5, 6, 7);
    bounds1.reset(b2);
    EXPECT_TRUE(bounds1.is3d());
    EXPECT_FALSE(bounds1.is4d());
    EXPECT_FALSE(bounds1.is2d());
    EXPECT_TRUE(bounds1.to3d() == b2);

    BOX2D b3(3, 4, 5, 6);
    bounds1.reset(b3);
    EXPECT_TRUE(bounds1.is2d());
    EXPECT_FALSE(bounds1.is3d());
    EXPECT_FALSE(bounds1.is4d());
    EXPECT_TRUE(bounds1.to2d() == b3);
}



}