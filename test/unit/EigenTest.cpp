/******************************************************************************
* Copyright (c) 2016, Peter J. Gadomski <pete.gadomski@gmail.com>
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

#include <pdal/EigenUtils.hpp>
#include <pdal/PointView.hpp>

#include <Eigen/Dense>

#include <limits>

using namespace pdal;

TEST(EigenTest, PointViewToEigen)
{
    PointTable table;
    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);
    PointView pointView(table);
    pointView.setField(Dimension::Id::X, 0, 1.0);
    pointView.setField(Dimension::Id::Y, 0, 2.0);
    pointView.setField(Dimension::Id::Z, 0, 3.0);
    Eigen::MatrixXd expected(1, 3);
    expected << 1.0, 2.0, 3.0;
    Eigen::MatrixXd actual = eigen::pointViewToEigen(pointView);
    ASSERT_EQ(1, actual.rows());
    ASSERT_EQ(3, actual.cols());
    EXPECT_EQ(expected, actual);
}

TEST(EigenTest, ComputeValues)
{
    using namespace Eigen;

    Matrix3d A;
    A << 1.8339,  0.3188, 0.3426,
        -2.2588, -1.3077, 3.5784,
         0.8622, -0.4336, 2.7694;

    double spacing(1.4);

    double zXX = eigen::centralDiffX2(A, spacing);
    EXPECT_NEAR(2.0076530612, zXX, 0.0001);

    double zYY = eigen::centralDiffY2(A, spacing);
    EXPECT_NEAR(1.2758163265, zYY, 0.0001);

    double zXY = eigen::centralDiffXY(A, spacing);
    EXPECT_NEAR(-0.4334821429, zXY, 0.0001);

    double zX = eigen::centralDiffX(A, spacing);
    EXPECT_NEAR(2.0847142857, zX, 0.0001);

    double zY = eigen::centralDiffY(A, spacing);
    EXPECT_NEAR(0.2687142857, zY, 0.0001);

    double p = (zX * zX) + (zY * zY);
    EXPECT_NEAR(4.4182410203, p, 0.0001);

    double q = p + 1;
    EXPECT_NEAR(5.4182410203, q, 0.0001);

    double contour = eigen::computeContour(A, spacing);
    EXPECT_NEAR(0.1669520079, contour, 0.0001);

    double profile = eigen::computeProfile(A, spacing);
    EXPECT_NEAR(0.149520634, profile, 0.0001);

    double tangential = eigen::computeTangential(A, spacing);
    EXPECT_NEAR(0.6004609473, tangential, 0.0001);

    double total = eigen::computeTotal(A, spacing);
    EXPECT_NEAR(6.0341916495, total, 0.0001);

    double dZdX = eigen::computeDZDX(A, spacing);
    EXPECT_NEAR(1.0794910714, dZdX, 0.0001);

    double dZdY = eigen::computeDZDY(A, spacing);
    EXPECT_NEAR(-0.0044375, dZdY, 0.0001);

    double slope = eigen::computeSlopeRad(dZdX, dZdY);
    EXPECT_NEAR(0.8236099869, slope, 0.0001);

    double sd8 = eigen::computeSlopeD8(A, spacing);
    EXPECT_NEAR(67.9357, sd8, 0.0001);

    double sfd = eigen::computeSlopeFD(A, spacing);
    EXPECT_NEAR(210.1961, sfd, 0.0001);

    double ad8 = eigen::computeAspectD8(A, spacing);
    EXPECT_NEAR(64.0, ad8, 0.0001);

    double afd = eigen::computeAspectFD(A, spacing);
    EXPECT_NEAR(269.8718, afd, 0.0001);

    double hs = eigen::computeHillshade(A, spacing, 45.0, 315.0);
    
    MatrixXd out = eigen::gradX(A);
    
    Matrix3d gx;
    gx << -1.5151, -0.7457, 0.0238,
           0.9511,  2.9186, 4.8861,
          -1.2958,  0.9536, 3.2030;
    
    for (size_t i = 0; i < 9; ++i)
        EXPECT_NEAR(gx(i), out(i), 0.0001);
    
    MatrixXd out2 = eigen::gradY(A);
    
    Matrix3d gy;
    gy << -4.0927, -1.6265,  3.2358,
          -0.4859, -0.3762,  1.2134,
           3.1210,  0.8741, -0.8090;
    
    for (size_t i = 0; i < 9; ++i)
        EXPECT_NEAR(gy(i), out2(i), 0.0001);

    A(0, 0) = std::numeric_limits<double>::quiet_NaN();
    Matrix3d B = eigen::replaceNaNs(A);
    EXPECT_NEAR(0.4839, B(0, 0), 0.0001);
}

TEST(EigenTest, CheckThrow)
{
    using namespace Eigen;

    MatrixXd A = MatrixXd::Zero(3, 4);

    EXPECT_THROW(eigen::computeSlopeD8(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::computeSlopeFD(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::computeAspectD8(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::computeAspectFD(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::computeContour(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::computeProfile(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::computeTangential(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::computeHillshade(A, 1.0, 45.0, 315.0), pdal_error);
    EXPECT_THROW(eigen::computeTotal(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::centralDiffX2(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::centralDiffY2(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::centralDiffXY(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::centralDiffX(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::centralDiffY(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::computeDZDX(A, 1.0), pdal_error);
    EXPECT_THROW(eigen::computeDZDY(A, 1.0), pdal_error);

    MatrixXd B = MatrixXd::Zero(3, 3);

    EXPECT_THROW(eigen::computeSlopeD8(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::computeSlopeFD(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::computeAspectD8(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::computeAspectFD(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::computeContour(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::computeProfile(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::computeTangential(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::computeHillshade(B, -1.0, 45.0, 315.0), pdal_error);
    EXPECT_THROW(eigen::computeTotal(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::centralDiffX2(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::centralDiffY2(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::centralDiffXY(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::centralDiffX(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::centralDiffY(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::computeDZDX(B, -1.0), pdal_error);
    EXPECT_THROW(eigen::computeDZDY(B, -1.0), pdal_error);
}

TEST(EigenTest, Padding)
{
    using namespace Eigen;

    MatrixXd A(3, 3);
    A << 1.8339,  0.3188, 0.3426,
        -2.2588, -1.3077, 3.5784,
         0.8622, -0.4336, 2.7694;

    MatrixXd B = eigen::padMatrix(A, 1);

    EXPECT_EQ(5, B.rows());
    EXPECT_EQ(5, B.cols());

    EXPECT_EQ(-2.2588, B(2, 0));
    EXPECT_EQ(0.3188, B(0, 2));
    EXPECT_EQ(3.5784, B(2, 4));
    EXPECT_EQ(-0.4336, B(4, 2));
}

TEST(EigenTest, Dilate)
{
    using namespace Eigen;
    
    MatrixXd C(5, 5);
    C << 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0,
         0, 1, 1, 0, 0,
         0, 0, 1, 1, 0,
         0, 0, 0, 0, 0;
    
    MatrixXd D = eigen::dilate(C, 1);
    
    EXPECT_EQ(0, D(0, 0));
    EXPECT_EQ(1, D(1, 0));
    EXPECT_EQ(1, D(0, 1));
    
    MatrixXd E(5, 5);
    E << 0, 0, 0, 0, 0,
         0, 1, 1, 1, 0,
         0, 1, 1, 1, 0,
         0, 1, 1, 1, 0,
         0, 0, 0, 0, 0;
    
    MatrixXd F = eigen::erode(E, 1);
    
    EXPECT_EQ(0, F(1, 3));
    EXPECT_EQ(1, F(2, 2));
}
