/******************************************************************************
 * Copyright (c) 2021 TileDB, Inc.
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
        bounds.reset(BOX4D(0., 0., 0., 0., 10., 10., 10., 10.));
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
    Bounds4D* bds1_ptr(&bounds1);
    EXPECT_TRUE(dynamic_cast<const Bounds4D*>(bds1_ptr) != nullptr);
}

TEST(Bounds4D, test_equals)
{
    BOX4D b1(1, 2, 3, 4, 5, 6, 7, 8);
    BOX4D b2(1, 2, 3, 4, 5, 6, 7, 8);
    BOX4D b3(1, 2, 3, 4, 5, 6, 7, 7);

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

    BOX4D b2(1, 1, 1, 1, 11, 11, 11, 11);
    BOX4D b3(1, 1, 1, 1, 10, 10, 10, 10);
    b1.clip(b2);

    EXPECT_TRUE(b1 == b3);

    BOX4D b4(2, 3, 4, 5, 5, 6, 7, 8);
    b1.clip(b4);

    EXPECT_TRUE(b1 == b4);

    BOX4D b5(20, 20, 20, 20, -10, -10, -10, -10);
    b1.clip(b5);

    EXPECT_TRUE(b1 == b4);

    BOX4D b6(6, 4, 8, 6, 4, 2, 6, 4);
    BOX4D b7(2, 4, 4, 6, 4, 6, 6, 8);
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
    Bounds4D* bds1_ptr(&bounds1);
    EXPECT_TRUE(dynamic_cast<const Bounds4D*>(bds1_ptr) != nullptr);
}

TEST_F(Bounds4DTest, test_ctor_box3d)
{
    Bounds4D bounds1(b.to3d());
    Bounds4D* bds1_ptr(&bounds1);
    EXPECT_TRUE(dynamic_cast<const Bounds4D*>(bds1_ptr) != nullptr);
}

TEST_F(Bounds4DTest, test_ctor_box2d)
{
    Bounds4D bounds1(b.to2d());
    Bounds4D* bds1_ptr(&bounds1);
    EXPECT_TRUE(dynamic_cast<const Bounds4D*>(bds1_ptr) != nullptr);
}

// TEST_F(Bounds4DTest, test_bounds_to2d)
//{
//     BOX2D b1(bounds.to2d());
//     BOX2D b2(b.minx, b.miny, b.maxx, b.maxy);
//     EXPECT_TRUE(b1 == b2);
// }

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
    BOX4D b1(1, 2, 3, 4, 5, 6, 7, 8);
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

TEST_F(Bounds4DTest, test_grow_2args)
{
    // no grow if is4d or is3d
    Bounds4D bounds4d(bounds.to4d());
    Bounds4D bounds3d(bounds.to3d());
    BOX3D b1(0, 0, 0, 10, 10, 10);
    bounds4d.grow(-10, 20);
    bounds3d.grow(-10, 20);
    EXPECT_TRUE(bounds4d.to4d() == b);
    EXPECT_TRUE(bounds3d.to3d() == b1);

    Bounds4D bounds2d(bounds.to2d());
    BOX2D b2(-10, 0, 10, 20);
    bounds2d.grow(-10, 20);
    EXPECT_TRUE(bounds2d.to2d() == b2);
}

TEST_F(Bounds4DTest, test_grow_3args)
{
    Bounds4D bounds4d(bounds.to4d());
    Bounds4D bounds2d(bounds.to2d());
    BOX2D b1(b.minx, b.miny, b.maxx, b.maxy);
    bounds4d.grow(-10, 20, -10);
    bounds2d.grow(-10, 20, -10);
    EXPECT_TRUE(bounds4d.to4d() == b);

    EXPECT_TRUE(bounds2d.is2d());
    EXPECT_TRUE(bounds2d.to2d() == b1);

    Bounds4D bounds3d(bounds.to3d());
    bounds3d.grow(-10, 20, -10);

    BOX3D b2(-10, 0, -10, 10, 20, 10);
    EXPECT_TRUE(bounds3d.to3d() == b2);
}

TEST_F(Bounds4DTest, test_grow_4args)
{
    Bounds4D bounds3d(bounds.to3d());
    Bounds4D bounds2d(bounds.to2d());

    bounds3d.grow(-10, 20, -10, 20);
    bounds2d.grow(-10, 20, -10, 20);

    EXPECT_TRUE(bounds3d.is3d());
    EXPECT_TRUE(bounds3d.to3d() == b.to3d());

    EXPECT_TRUE(bounds2d.is2d());
    EXPECT_TRUE(bounds2d.to2d() == b.to2d());

    Bounds4D bounds4d(bounds);

    bounds4d.grow(-10, 20, -10, 20);

    BOX4D b1(-10, 0, -10, 0, 10, 20, 10, 20);
    EXPECT_TRUE(bounds4d.to4d() == b1);
}

TEST_F(Bounds4DTest, test_parse)
{
    std::stringstream ss1("([0, 10], [0, 10], [0, 10], [0, 10])",
                          std::stringstream::in | std::stringstream::out);

    BOX4D r1;
    ss1 >> r1;
    EXPECT_TRUE(r1 == b);

    std::stringstream ss2("([0, 10], [0, 10], [0, 10])",
                          std::stringstream::in | std::stringstream::out);

    BOX3D r2;
    ss2 >> r2;
    EXPECT_TRUE(r2 == b.to3d());

    std::stringstream ss3("([0, 10], [0, 10])",
                          std::stringstream::in | std::stringstream::out);

    BOX2D r3;
    ss3 >> r3;
    EXPECT_TRUE(r3 == b.to2d());
}

} // namespace pdal