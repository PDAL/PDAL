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

class BOX4DTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {}
    BOX4D b{0., 0., 0., 0., 10., 10., 10., 10.};
};

TEST_F(BOX4DTest, test_bbox4d)
{
    EXPECT_EQ(b.minx, 0);
    EXPECT_EQ(b.miny, 0);
    EXPECT_EQ(b.minz, 0);
    EXPECT_EQ(b.mintm, 0);
    EXPECT_EQ(b.maxx, 10);
    EXPECT_EQ(b.maxy, 10);
    EXPECT_EQ(b.maxz, 10);
    EXPECT_EQ(b.maxtm, 10);
}

TEST_F(BOX4DTest, test_clip)
{
    BOX4D b1 = b;
    EXPECT_TRUE(b1 == b);

    BOX4D b2(2., 2., 2., 2., 8., 8., 8., 8.);
    b1.clip(b2);

    // fails because of bug in pdal BOX3D clip() currently being edited
    EXPECT_TRUE(b1 == b2);


}

}