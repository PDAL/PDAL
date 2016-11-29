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

using namespace pdal;

TEST(EigenTest, PointViewToEigen) {
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

TEST(EigenTest, DiffTests) {
    using namespace Eigen;

    Matrix3d A;
    A << 1.8339, 0.3188, 0.3426,
    -2.2588, -1.3077, 3.5784,
    0.8622, -0.4336, 2.7694;

    MatrixXd out = eigen::gradX(A);
    
    Matrix3d gx;
    gx << -1.5151, -0.7457, 0.0238,
    0.9511, 2.9186, 4.8861,
    -1.2958, 0.9536, 3.2030;
    
    for (size_t i = 0; i < 9; ++i)
        EXPECT_NEAR(gx(i), out(i), 0.0001);
    
    MatrixXd out2 = eigen::gradY(A);
    
    Matrix3d gy;
    gy << -4.0927, -1.6265, 3.2358,
    -0.4859, -0.3762, 1.2134,
    3.1210, 0.8741, -0.8090;
    
    for (size_t i = 0; i < 9; ++i)
        EXPECT_NEAR(gy(i), out2(i), 0.0001);
}
