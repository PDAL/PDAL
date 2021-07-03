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

#include <pdal/PointView.hpp>
#include <pdal/private/MathUtils.hpp>

#include <Eigen/Dense>

#include <limits>
#include <numeric>

using namespace pdal;

PointViewPtr makeTestView(PointTableRef table, point_count_t cnt = 17)
{
    PointLayoutPtr layout(table.layout());

    layout->registerDim(Dimension::Id::Classification);
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);

    PointViewPtr view(new PointView(table));

    // write the data into the view
    for (PointId i = 0; i < cnt; i++)
    {
        const uint8_t x = static_cast<uint8_t>(i + 1);
        const int32_t y = static_cast<int32_t>(i * 10);
        const double z = static_cast<double>(i * 100);

        view->setField(Dimension::Id::Classification, i, x);
        view->setField(Dimension::Id::X, i, y);
        view->setField(Dimension::Id::Y, i, z);
    }
    EXPECT_EQ(view->size(), cnt);
    return view;
}


static void check_bounds(const BOX3D& box,
                         double minx, double maxx,
                         double miny, double maxy,
                         double minz, double maxz)
{
    EXPECT_DOUBLE_EQ(box.minx, minx);
    EXPECT_DOUBLE_EQ(box.maxx, maxx);
    EXPECT_DOUBLE_EQ(box.miny, miny);
    EXPECT_DOUBLE_EQ(box.maxy, maxy);
    EXPECT_DOUBLE_EQ(box.minz, minz);
    EXPECT_DOUBLE_EQ(box.maxz, maxz);
}

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
    Eigen::MatrixXd actual = math::pointViewToEigen(pointView);
    ASSERT_EQ(1, actual.rows());
    ASSERT_EQ(3, actual.cols());
    EXPECT_EQ(expected, actual);
}

TEST(EigenTest, ComputeValues)
{
    using namespace Eigen;

    Matrix3d A;
    A << 1.8339, 0.3188, 0.3426, -2.2588, -1.3077, 3.5784, 0.8622, -0.4336,
        2.7694;

    double spacing(1.4);

    MatrixXd out = math::gradX(A);

    Matrix3d gx;
    gx << -1.5151, -0.7457, 0.0238, 0.9511, 2.9186, 4.8861, -1.2958, 0.9536,
        3.2030;

    for (size_t i = 0; i < 9; ++i)
        EXPECT_NEAR(gx(i), out(i), 0.0001);

    MatrixXd out2 = math::gradY(A);

    Matrix3d gy;
    gy << -4.0927, -1.6265, 3.2358, -0.4859, -0.3762, 1.2134, 3.1210, 0.8741,
        -0.8090;

    for (size_t i = 0; i < 9; ++i)
        EXPECT_NEAR(gy(i), out2(i), 0.0001);
}

TEST(EigenTest, Morphological)
{
    using namespace Eigen;

    MatrixXd C(5, 5);
    C << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
        0;
    std::vector<double> Cv(C.data(), C.data() + C.size());

    std::vector<double> Dv = Cv;
    math::dilateDiamond(Dv, 5, 5, 1);
    std::vector<double> Dv2 = Cv;
    math::dilateDiamond(Dv2, 5, 5, 2);

    EXPECT_EQ(0, Dv[0]);
    EXPECT_EQ(1, Dv[1]);
    EXPECT_EQ(1, Dv[5]);
    EXPECT_EQ(1, Dv2[0]);
    EXPECT_EQ(1, Dv2[10]);
    EXPECT_EQ(0, Dv2[15]);

    MatrixXd E(5, 5);
    E << 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
        0;
    std::vector<double> Ev(E.data(), E.data() + E.size());

    std::vector<double> Fv = Ev;
    math::erodeDiamond(Fv, 5, 5, 1);
    std::vector<double> Fv2 = Ev;
    math::erodeDiamond(Fv2, 5, 5, 2);

    EXPECT_EQ(0, Fv[16]);
    EXPECT_EQ(1, Fv[12]);
    EXPECT_EQ(0, Fv2[12]);
}

TEST(EigenTest, RoundtripString)
{
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd target;
    Utils::fromString(Utils::toString(identity), target);
    ASSERT_EQ(identity.size(), target.size());
    EXPECT_EQ(identity, target);
}

TEST(EigenTest, calcBounds)
{
    auto set_points = [](PointViewPtr view, PointId i, double x, double y,
        double z)
    {
        view->setField(Dimension::Id::X, i, x);
        view->setField(Dimension::Id::Y, i, y);
        view->setField(Dimension::Id::Z, i, z);
    };

    PointTable table;
    PointLayoutPtr layout(table.layout());

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);

    const double lim_min = (std::numeric_limits<double>::lowest)();
    const double lim_max = (std::numeric_limits<double>::max)();
    PointViewPtr b0(new PointView(table));
    BOX3D box_b0;
    b0->calculateBounds(box_b0);
    check_bounds(box_b0, lim_max, lim_min, lim_max, lim_min, lim_max, lim_min);

    PointViewPtr b1(new PointView(table));
    set_points(b1, 0, 0.0, 0.0, 0.0);
    set_points(b1, 1, 2.0, 2.0, 2.0);

    PointViewPtr b2(new PointView(table));
    set_points(b2, 0, 3.0, 3.0, 3.0);
    set_points(b2, 1, 1.0, 1.0, 1.0);

    PointViewSet bs;
    bs.insert(b1);
    bs.insert(b2);

    BOX3D box_b1;
    b1->calculateBounds(box_b1);
    check_bounds(box_b1, 0.0, 2.0, 0.0, 2.0, 0.0, 2.0);

    BOX3D box_b2;
    b2->calculateBounds(box_b2);
    check_bounds(box_b2, 1.0, 3.0, 1.0, 3.0, 1.0, 3.0);
}

TEST(EigenTest, demeanTest)
{
    PointTable table;
    PointViewPtr view = makeTestView(table);
    PointViewPtr demeanView = math::demeanPointView(*view);
    EXPECT_EQ(-80, demeanView->getFieldAs<double>(Dimension::Id::X, 0));
    EXPECT_EQ(-800, demeanView->getFieldAs<double>(Dimension::Id::Y, 0));
}

TEST(EigenTest, computeCentroid)
{
    PointTable table;
    PointViewPtr view = makeTestView(table);
    PointIdList ids(view->size());
    std::iota(ids.begin(), ids.end(), 0);
    auto centroid = math::computeCentroid(*view, ids);
    EXPECT_EQ(80, centroid.x());
    EXPECT_EQ(800, centroid.y());
}
