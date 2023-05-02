/******************************************************************************
 * Copyright (c) 2023 TileDB, Inc.
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

#include "../io/TileDBUtils.hpp"
#include <pdal/pdal_test_main.hpp>

namespace pdal
{

TEST(DomainBounds, parse_empty)
{
    std::stringstream ss("()");
    DomainBounds bounds;
    ss >> bounds;

    EXPECT_TRUE(bounds.empty());
    EXPECT_EQ(bounds.ndim(), 0);
}

TEST(DomainBounds, parse_bbox1d)
{
    std::stringstream ss("([-1.0, 1.0])");
    DomainBounds bounds;
    ss >> bounds;

    EXPECT_FALSE(bounds.empty());
    EXPECT_EQ(bounds.ndim(), 1);
    EXPECT_FLOAT_EQ(bounds.minX(), -1.0);
    EXPECT_FLOAT_EQ(bounds.maxX(), 1.0);
}

TEST(DomainBounds, parse_bbox2d)
{
    std::stringstream ss("([-1.0, 1.0], [-2.0, 2.0])");
    DomainBounds bounds;
    ss >> bounds;

    EXPECT_FALSE(bounds.empty());
    EXPECT_EQ(bounds.ndim(), 2);
    EXPECT_FLOAT_EQ(bounds.minX(), -1.0);
    EXPECT_FLOAT_EQ(bounds.maxX(), 1.0);
    EXPECT_FLOAT_EQ(bounds.minY(), -2.0);
    EXPECT_FLOAT_EQ(bounds.maxY(), 2.0);
}

TEST(DomainBounds, parse_bbox3d)
{
    std::stringstream ss("([-1.0, 1.0], [-2.0, 2.0], [-3.0, 3.0])");
    DomainBounds bounds;
    ss >> bounds;

    EXPECT_FALSE(bounds.empty());
    EXPECT_EQ(bounds.ndim(), 3);
    EXPECT_FLOAT_EQ(bounds.minX(), -1.0);
    EXPECT_FLOAT_EQ(bounds.maxX(), 1.0);
    EXPECT_FLOAT_EQ(bounds.minY(), -2.0);
    EXPECT_FLOAT_EQ(bounds.maxY(), 2.0);
    EXPECT_FLOAT_EQ(bounds.minZ(), -3.0);
    EXPECT_FLOAT_EQ(bounds.maxZ(), 3.0);
}

TEST(DomainBounds, parse_bbox4d)
{
    std::stringstream ss("([-1.0, 1.0], [-2.0, 2.0], [-3.0, 3.0], [12, 16])");
    DomainBounds bounds;
    ss >> bounds;

    EXPECT_FALSE(bounds.empty());
    EXPECT_EQ(bounds.ndim(), 4);
    EXPECT_FLOAT_EQ(bounds.minX(), -1.0);
    EXPECT_FLOAT_EQ(bounds.maxX(), 1.0);
    EXPECT_FLOAT_EQ(bounds.minY(), -2.0);
    EXPECT_FLOAT_EQ(bounds.maxY(), 2.0);
    EXPECT_FLOAT_EQ(bounds.minZ(), -3.0);
    EXPECT_FLOAT_EQ(bounds.maxZ(), 3.0);
    EXPECT_FLOAT_EQ(bounds.minGpsTime(), 12.0);
    EXPECT_FLOAT_EQ(bounds.maxGpsTime(), 16.0);
}

TEST(DomainBounds, parse_bbox5d_with_error)
{
    std::stringstream ss(
        "([-1.0, 1.0], [-2.0, 2.0], [-3.0, 3.0], [12, 16], [1.0, 2.0])");
    DomainBounds bounds;
    EXPECT_THROW(ss >> bounds, std::runtime_error);
}

TEST(DomainBounds, parse_missing_comma_xy_with_error)
{
    std::stringstream ss("([-1.0, 1.0] [-2.0, 2.0], [-3.0, 3.0], [12, 16])");
    DomainBounds bounds;
    EXPECT_THROW(ss >> bounds, std::runtime_error);
}

TEST(DomainBounds, parse_missing_comma_yz_with_error)
{
    std::stringstream ss("([-1.0, 1.0], [-2.0, 2.0] [-3.0, 3.0], [12, 16])");
    DomainBounds bounds;
    EXPECT_THROW(ss >> bounds, std::runtime_error);
}

TEST(DomainBounds, parse_missing_comma_zt_with_error)
{
    std::stringstream ss("([-1.0, 1.0], [-2.0, 2.0], [-3.0, 3.0] [12, 16])");
    DomainBounds bounds;
    EXPECT_THROW(ss >> bounds, std::runtime_error);
}

TEST(DomainBounds, parse_pair_no_comma_with_error)
{
    std::istringstream ss("[-1.0 1.0]");
    DomainBounds bounds;
    EXPECT_THROW(bounds.parsePair(ss, "Dim"), std::runtime_error);
}

TEST(DomainBounds, parse_pair_one_value_with_error)
{
    std::istringstream ss("[-1.0]");
    DomainBounds bounds;
    EXPECT_THROW(bounds.parsePair(ss, "Dim"), std::runtime_error);
}

TEST(DomainBounds, parse_pair_missing_front_bracket_with_error)
{
    std::istringstream ss("-1.0, 1.0]");
    DomainBounds bounds;
    EXPECT_THROW(bounds.parsePair(ss, "Dim"), std::runtime_error);
}

TEST(DomainBounds, parse_pair_missing_back_bracket_with_error)
{
    std::istringstream ss("[-1.0, 1.0)");
    DomainBounds bounds;
    EXPECT_THROW(bounds.parsePair(ss, "Dim"), std::runtime_error);
}

} // namespace pdal
