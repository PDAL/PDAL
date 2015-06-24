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

#include <pdal/pdal_test_main.hpp>

#include <pdal/KDIndex.hpp>

using namespace pdal;

TEST(KDIndex, neighbors2D)
{
    PointTable table;
    PointLayoutPtr layout = table.layout();
    PointView view(table);

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);

    view.setField(Dimension::Id::X, 0, 0);
    view.setField(Dimension::Id::Y, 0, 0);
    
    view.setField(Dimension::Id::X, 1, 1);
    view.setField(Dimension::Id::Y, 1, 1);
    
    view.setField(Dimension::Id::X, 2, 3);
    view.setField(Dimension::Id::Y, 2, 3);
    
    view.setField(Dimension::Id::X, 3, 6);
    view.setField(Dimension::Id::Y, 3, 6);
    
    view.setField(Dimension::Id::X, 4, 10);
    view.setField(Dimension::Id::Y, 4, 10);
    
    KD2Index index(view);
    index.build();

    EXPECT_EQ(index.neighbor(0, 0), 0u);
    EXPECT_EQ(index.neighbor(1.1, 1.1), 1u);
    EXPECT_EQ(index.neighbor(3.3, 3.3), 2u);
    EXPECT_EQ(index.neighbor(6.1, 6.1), 3u);
    EXPECT_EQ(index.neighbor(15, 15), 4u);

    std::vector<PointId> ids;
    ids = index.neighbors(0, 0, 5);
    EXPECT_EQ(ids.size(), 5u);
    EXPECT_EQ(ids[0], 0u);
    EXPECT_EQ(ids[1], 1u);
    EXPECT_EQ(ids[2], 2u);
    EXPECT_EQ(ids[3], 3u);
    EXPECT_EQ(ids[4], 4u);

    ids = index.neighbors(0, 0, 25);
    EXPECT_EQ(ids.size(), 5u);
    EXPECT_EQ(ids[0], 0u);
    EXPECT_EQ(ids[1], 1u);
    EXPECT_EQ(ids[2], 2u);
    EXPECT_EQ(ids[3], 3u);
    EXPECT_EQ(ids[4], 4u);

    ids = index.neighbors(3.1, 3.1, 5);
    EXPECT_EQ(ids.size(), 5u);
    EXPECT_EQ(ids[0], 2u);
    EXPECT_EQ(ids[1], 1u);
    EXPECT_EQ(ids[2], 3u);
    EXPECT_EQ(ids[3], 0u);
    EXPECT_EQ(ids[4], 4u);
}

TEST(KDIndex, neighbors3D)
{
    PointTable table;
    PointLayoutPtr layout = table.layout();
    PointView view(table);

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);

    view.setField(Dimension::Id::X, 0, 0);
    view.setField(Dimension::Id::Y, 0, 0);
    view.setField(Dimension::Id::Z, 0, 0);
    
    view.setField(Dimension::Id::X, 1, 1);
    view.setField(Dimension::Id::Y, 1, 1);
    view.setField(Dimension::Id::Z, 1, 1);
    
    view.setField(Dimension::Id::X, 2, 3);
    view.setField(Dimension::Id::Y, 2, 3);
    view.setField(Dimension::Id::Z, 2, 3);
    
    view.setField(Dimension::Id::X, 3, 6);
    view.setField(Dimension::Id::Y, 3, 6);
    view.setField(Dimension::Id::Z, 3, 6);
    
    view.setField(Dimension::Id::X, 4, 10);
    view.setField(Dimension::Id::Y, 4, 10);
    view.setField(Dimension::Id::Z, 4, 10);
    
    KD3Index index(view);
    index.build();

    EXPECT_EQ(index.neighbor(0, 0, 0), 0u);
    EXPECT_EQ(index.neighbor(1.1, 1.1, 1.1), 1u);
    EXPECT_EQ(index.neighbor(3.3, 3.3, 3.3), 2u);
    EXPECT_EQ(index.neighbor(6.1, 6.1, 6.1), 3u);
    EXPECT_EQ(index.neighbor(15, 15, 15), 4u);

    std::vector<PointId> ids;
    ids = index.neighbors(0, 0, 0, 5);
    EXPECT_EQ(ids.size(), 5u);
    EXPECT_EQ(ids[0], 0u);
    EXPECT_EQ(ids[1], 1u);
    EXPECT_EQ(ids[2], 2u);
    EXPECT_EQ(ids[3], 3u);
    EXPECT_EQ(ids[4], 4u);

    ids = index.neighbors(0, 0, 0, 25);
    EXPECT_EQ(ids.size(), 5u);
    EXPECT_EQ(ids[0], 0u);
    EXPECT_EQ(ids[1], 1u);
    EXPECT_EQ(ids[2], 2u);
    EXPECT_EQ(ids[3], 3u);
    EXPECT_EQ(ids[4], 4u);

    ids = index.neighbors(3.1, 3.1, 3.1, 5);
    EXPECT_EQ(ids.size(), 5u);
    EXPECT_EQ(ids[0], 2u);
    EXPECT_EQ(ids[1], 1u);
    EXPECT_EQ(ids[2], 3u);
    EXPECT_EQ(ids[3], 0u);
    EXPECT_EQ(ids[4], 4u);
}

TEST(KDIndex, neighbordims)
{
    PointTable table;
    PointLayoutPtr layout = table.layout();
    PointView view(table);

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Z);

    view.setField(Dimension::Id::X, 0, 0);
    view.setField(Dimension::Id::Z, 0, 0);
    
    view.setField(Dimension::Id::X, 1, 1);
    view.setField(Dimension::Id::Z, 1, 1);
    
    EXPECT_THROW(KD2Index index(view), pdal_error);

    PointTable table2;
    PointLayoutPtr layout2 = table.layout();
    PointView view2(table2);

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);

    view.setField(Dimension::Id::X, 0, 0);
    view.setField(Dimension::Id::Y, 0, 0);
    
    view.setField(Dimension::Id::X, 1, 1);
    view.setField(Dimension::Id::Y, 1, 1);
    
    EXPECT_THROW(KD3Index index(view2), pdal_error);
}

TEST(KDIndex, radius2D)
{
    PointTable table;
    PointLayoutPtr layout = table.layout();
    PointView view(table);

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);

    view.setField(Dimension::Id::X, 0, 0);
    view.setField(Dimension::Id::Y, 0, 0);
    
    view.setField(Dimension::Id::X, 1, 1);
    view.setField(Dimension::Id::Y, 1, 1);
    
    view.setField(Dimension::Id::X, 2, 3);
    view.setField(Dimension::Id::Y, 2, 3);
    
    view.setField(Dimension::Id::X, 3, 6);
    view.setField(Dimension::Id::Y, 3, 6);
    
    view.setField(Dimension::Id::X, 4, 10);
    view.setField(Dimension::Id::Y, 4, 10);
    
    KD2Index index(view);
    index.build();

    std::vector<PointId> ids;
    ids = index.radius(0, 0, 4.25);
    
    EXPECT_EQ(ids.size(), 3u);
    EXPECT_EQ(ids[0], 0u);
    EXPECT_EQ(ids[1], 1u);
    EXPECT_EQ(ids[2], 2u);

    ids = index.radius(3.1, 3.1, 10);
    EXPECT_EQ(ids.size(), 5u);
    EXPECT_EQ(ids[0], 2u);
    EXPECT_EQ(ids[1], 1u);
    EXPECT_EQ(ids[2], 3u);
    EXPECT_EQ(ids[3], 0u);
    EXPECT_EQ(ids[4], 4u);
}

TEST(KDIndex, radius3D)
{
    PointTable table;
    PointLayoutPtr layout = table.layout();
    PointView view(table);

    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);

    view.setField(Dimension::Id::X, 0, 0);
    view.setField(Dimension::Id::Y, 0, 0);
    view.setField(Dimension::Id::Z, 0, 0);
    
    view.setField(Dimension::Id::X, 1, 1);
    view.setField(Dimension::Id::Y, 1, 1);
    view.setField(Dimension::Id::Z, 1, 1);
    
    view.setField(Dimension::Id::X, 2, 3);
    view.setField(Dimension::Id::Y, 2, 3);
    view.setField(Dimension::Id::Z, 2, 3);
    
    view.setField(Dimension::Id::X, 3, 6);
    view.setField(Dimension::Id::Y, 3, 6);
    view.setField(Dimension::Id::Z, 3, 6);
    
    view.setField(Dimension::Id::X, 4, 10);
    view.setField(Dimension::Id::Y, 4, 10);
    view.setField(Dimension::Id::Z, 4, 10);
    
    KD3Index index(view);
    index.build();

    std::vector<PointId> ids;
    ids = index.radius(0, 0, 0, 5.2);
    
    EXPECT_EQ(ids.size(), 3u);
    EXPECT_EQ(ids[0], 0u);
    EXPECT_EQ(ids[1], 1u);
    EXPECT_EQ(ids[2], 2u);

    ids = index.radius(3.1, 3.1, 3.1, 12.2);
    EXPECT_EQ(ids.size(), 5u);
    EXPECT_EQ(ids[0], 2u);
    EXPECT_EQ(ids[1], 1u);
    EXPECT_EQ(ids[2], 3u);
    EXPECT_EQ(ids[3], 0u);
    EXPECT_EQ(ids[4], 4u);
}

