/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski <pete.gadomski@gmail.com>
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

#define _USE_MATH_DEFINES
#include <pdal/pdal_test_main.hpp>
#include <pdal/util/Georeference.hpp>

#include <cmath>


namespace pdal
{
namespace georeference
{


TEST(RotationMatrix, Constructor)
{
    RotationMatrix matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
    EXPECT_DOUBLE_EQ(1, matrix.m00);
    EXPECT_DOUBLE_EQ(0, matrix.m01);
    EXPECT_DOUBLE_EQ(0, matrix.m01);
    EXPECT_DOUBLE_EQ(0, matrix.m10);
    EXPECT_DOUBLE_EQ(1, matrix.m11);
    EXPECT_DOUBLE_EQ(0, matrix.m12);
    EXPECT_DOUBLE_EQ(0, matrix.m20);
    EXPECT_DOUBLE_EQ(0, matrix.m21);
    EXPECT_DOUBLE_EQ(1, matrix.m22);
}


TEST(RotationMatrix, IdentityMatrix)
{
    RotationMatrix matrix = createIdentityMatrix();
    EXPECT_DOUBLE_EQ(1, matrix.m00);
    EXPECT_DOUBLE_EQ(0, matrix.m01);
    EXPECT_DOUBLE_EQ(0, matrix.m01);
    EXPECT_DOUBLE_EQ(0, matrix.m10);
    EXPECT_DOUBLE_EQ(1, matrix.m11);
    EXPECT_DOUBLE_EQ(0, matrix.m12);
    EXPECT_DOUBLE_EQ(0, matrix.m20);
    EXPECT_DOUBLE_EQ(0, matrix.m21);
    EXPECT_DOUBLE_EQ(1, matrix.m22);
}


TEST(Georeference, Zeros)
{
    Xyz point = georeferenceWgs84(0, 0, createIdentityMatrix(),
                                  createIdentityMatrix(), Xyz(0, 0, 0));
    EXPECT_DOUBLE_EQ(0, point.X);
    EXPECT_DOUBLE_EQ(0, point.Y);
    EXPECT_DOUBLE_EQ(0, point.Z);
}


TEST(Georeference, LatLonElev)
{
    Xyz point = georeferenceWgs84(0, 0, createIdentityMatrix(),
                                  createIdentityMatrix(), Xyz(1, 2, 3));
    EXPECT_DOUBLE_EQ(1, point.X);
    EXPECT_DOUBLE_EQ(2, point.Y);
    EXPECT_DOUBLE_EQ(3, point.Z);
}


TEST(Georeference, Range)
{
    Xyz point = georeferenceWgs84(3, 0, createIdentityMatrix(),
                                  createIdentityMatrix(), Xyz(1, 2, 3));
    EXPECT_DOUBLE_EQ(1, point.X);
    EXPECT_DOUBLE_EQ(2, point.Y);
    EXPECT_DOUBLE_EQ(0, point.Z);
}


TEST(Georeference, RangeAndAngle)
{
    Xyz point = georeferenceWgs84(3, M_PI / 2, createIdentityMatrix(),
                                  createIdentityMatrix(), Xyz(1, 2, 3));
    EXPECT_DOUBLE_EQ(0.9999988728659957, point.X);
    EXPECT_DOUBLE_EQ(2, point.Y);
    EXPECT_DOUBLE_EQ(3, point.Z);
}


TEST(Georeference, WithImu)
{
    RotationMatrix imuMatrix(0, 1, 0, 0, 0, -1, -1, 0, 0);
    Xyz point =
        georeferenceWgs84(3, 0, imuMatrix, createIdentityMatrix(), Xyz(1, 2, 3));
    EXPECT_DOUBLE_EQ(1, point.X);
    EXPECT_DOUBLE_EQ(2.0000004696006983, point.Y);
    EXPECT_DOUBLE_EQ(3, point.Z);
}
}
}
