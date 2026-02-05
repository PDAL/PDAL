/******************************************************************************
* Copyright (c) 2015, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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

#include <chrono>

#include <pdal/pdal_test_main.hpp>
#include "Support.hpp"

#include <io/LasReader.hpp>
#include <io/CopcReader.hpp>
#include <filters/CropFilter.hpp>
#include <pdal/private/PointGrid.hpp>
#include <pdal/KDIndex.hpp>

using namespace pdal;
/*
void testPointsKnn2d(PointViewPtr view, PointId pos, int knn)
{
    PointViewPtr kdView = view->makeNew();
    kdView->append(*view);

    const auto t1 = std::chrono::high_resolution_clock::now();
    const KD2Index& index = kdView->build2dIndex();
    const auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "kdtree build: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;
    PointIdList ids(knn);
    std::vector<double> sqr_dists(knn);
    index.knnSearch(pos, knn, &ids, &sqr_dists);
    const auto t3 = std::chrono::high_resolution_clock::now();
    std::cout << "Time kdtree: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t1).count() << std::endl;


    const auto t4 = std::chrono::high_resolution_clock::now();
    PointGrid grid(*view);
    grid.build();
    const auto t5 = std::chrono::high_resolution_clock::now();
    std::cout << "pointgrid build: " << std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count() << std::endl;
    DistanceResults neighbors = 
        grid.knnSearch({view->getFieldAs<double>(Dimension::Id::X, pos), 
            view->getFieldAs<double>(Dimension::Id::Y, pos)}, knn);
    const auto t6 = std::chrono::high_resolution_clock::now();
    std::cout << "Time pointgrid: " << std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t4).count() << std::endl;

    EXPECT_EQ(neighbors.size(), knn);
    EXPECT_EQ(ids.size(), knn);

    for (size_t i = 0; i < neighbors.size(); ++i)
    {
        //EXPECT_EQ(neighbors[i].first, ids[i]);
        EXPECT_NEAR(neighbors[i].second, sqr_dists[i], 0.0001);
    }
}

void testPointsKnn3d(PointViewPtr view, PointId pos, int knn)
{
    PointViewPtr kdView = view->makeNew();
    kdView->append(*view);

    BOX2D bounds;
    view->calculateBounds(bounds);
    const auto t1 = std::chrono::high_resolution_clock::now();
    const KD3Index& index = kdView->build3dIndex();
    const auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "kdtree build: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;
    PointIdList ids(knn);
    std::vector<double> sqr_dists(knn);
    index.knnSearch(pos, knn, &ids, &sqr_dists);
    const auto t3 = std::chrono::high_resolution_clock::now();
    std::cout << "Time kdtree: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t1).count() << std::endl;


    const Eigen::Vector3d point(view->getFieldAs<double>(Dimension::Id::X, pos), 
            view->getFieldAs<double>(Dimension::Id::Y, pos), 
            view->getFieldAs<double>(Dimension::Id::Z, pos));
    const auto t4 = std::chrono::high_resolution_clock::now();
    PointGrid grid(*view);
    grid.build();
    const auto t5 = std::chrono::high_resolution_clock::now();
    std::cout << "pointgrid build: " << std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count() << std::endl;
    DistanceResults neighbors = 
        grid.knnSearch(point, knn, std::pow(bounds.maxx - bounds.minx, 2) + std::pow(bounds.maxy - bounds.miny, 2));
    const auto t6 = std::chrono::high_resolution_clock::now();
    std::cout << "Time pointgrid: " << std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t4).count() << std::endl;

    EXPECT_EQ(neighbors.size(), knn);
    EXPECT_EQ(ids.size(), knn);

    for (size_t i = 0; i < neighbors.size(); ++i)
    {
        //EXPECT_EQ(neighbors[i].first, ids[i]);
        EXPECT_NEAR(neighbors[i].second, sqr_dists[i], 0.0001);
    }
}

TEST(PointGridTest, outsidePolygon)
{
    LasReader r;
    Options ro;
    ro.add("filename", Support::datapath("las/4_6_crop.las"));
    r.setOptions(ro);

    CropFilter f;
    Options fo;
    fo.add("polygon", "POLYGON ((1639605.52151933 1454498.88572908,1639605.20941558 1454507.23450445,1639597.48484772 1454506.45424507,1639595.2089086 1454685.11546591,1639776.14606856 1454683.97749635,1639770.45622077 1454489.38470167,1639605.52151933 1454498.88572908))");
    fo.add("outside", true);
    f.setOptions(fo);
    f.setInput(r);

    PointTable table;
    f.prepare(table);
    PointViewSet viewSet = f.execute(table);
    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();

    testPointsKnn2d(view, 0, 100);
    testPointsKnn2d(view, 878, 100);
    testPointsKnn3d(view, 0, 100);
    testPointsKnn3d(view, 878, 100);
}

TEST(PointGridTest, largeFile)
{
    CopcReader r;
    Options ro;
    ro.add("filename", "https://s3.amazonaws.com/grid-public-ept/camas/copc/CamasBE-15_0.copc.laz");
    ro.add("threads", 10);
    r.setOptions(ro);

    PointTable table;
    r.prepare(table);
    PointViewSet viewSet = r.execute(table);
    EXPECT_EQ(1u, viewSet.size());
    PointViewPtr view = *viewSet.begin();

    testPointsKnn2d(view, 0, 100);
    testPointsKnn2d(view, 878, 140);
    testPointsKnn2d(view, 20029, 200);
    testPointsKnn3d(view, 0, 100);
    testPointsKnn3d(view, 878, 140);
    testPointsKnn3d(view, 20029, 200);
}
*/