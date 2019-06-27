/******************************************************************************
* Copyright (c) 2019, Helix Re Inc.
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
*     * Neither the name of Helix Re Inc. nor the
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
#include "Support.hpp"

#include "plugins/e57/io/E57Reader.hpp"
#include "plugins/e57/io/Utils.hpp"

using namespace pdal;

TEST(E57Reader, testCtr)
{
    Options ops;
    ops.add("filename",Support::datapath("e57/A4.e57"));
    E57Reader reader2;
    reader2.setOptions(ops);
    PointTable table;
    reader2.prepare(table);
    ASSERT_TRUE(table.layout()->hasDim(Dimension::Id::X));
}

TEST(E57Reader, testGetDimension) 
{
    Options ops;
    ops.add("filename", Support::datapath("e57/A_B.e57"));
    E57Reader reader;
    reader.setOptions(ops);
    PointTable table;
    reader.prepare(table);

    auto dimensions = reader.getDimensions();
    ASSERT_EQ(dimensions.size(),7u);
    std::vector<std::string> expectedDimensions = {"cartesianX","cartesianY","cartesianZ",
        "colorRed","colorGreen","colorBlue","intensity"};
    for (auto dim: expectedDimensions)
    {
        ASSERT_TRUE(dimensions.find(dim) != dimensions.end());
    }
}

TEST(E57Reader, testPreview)
{
    Options ops;
    ops.add("filename", Support::datapath("e57/A_B.e57"));
    E57Reader reader;
    reader.setOptions(ops);
    auto qi = reader.preview();

    auto dimensions = qi.m_dimNames;
    ASSERT_EQ(dimensions.size(),7u);
    ASSERT_EQ(qi.m_pointCount,6u);
    ASSERT_TRUE(qi.m_valid);
    ASSERT_TRUE(qi.m_bounds.valid());
}

TEST(E57Reader, testHeader) 
{
    Options ops;
    ops.add("filename", Support::datapath("e57/A_B.e57"));
    E57Reader reader;
    reader.setOptions(ops);
    PointTable table;
    reader.prepare(table);

    auto expectedE57Dimensions = reader.getDimensions();
    for (auto& e57Dim: expectedE57Dimensions)
    {
        ASSERT_TRUE(table.layout()->hasDim(pdal::e57plugin::e57ToPdal(e57Dim)));
    }
}

TEST(E57Reader, pointCount)
{
    E57Reader reader(Support::datapath("e57/A4.e57"));
    auto count = reader.getNumberPoints();
    ASSERT_EQ(count,4u);
}

TEST(E57Reader, getScans)
{
    E57Reader reader(Support::datapath("e57/A_B.e57"));
    auto scans = reader.getScans();
    ASSERT_EQ(scans.size(),2u);
    E57Reader reader2(Support::datapath("e57/A4.e57"));
    scans = reader2.getScans();
    ASSERT_EQ(scans.size(),1u);
}

TEST(E57Reader, testRead) 
{
    Options ops;
    ops.add("filename",Support::datapath("e57/A4.e57"));
    E57Reader reader;
    reader.setOptions(ops);
    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    ASSERT_EQ(viewSet.size(),1u);

    auto cloud = *viewSet.begin();
    ASSERT_EQ(cloud->size(),4u);

    auto pt = cloud->point(0);
    ASSERT_FLOAT_EQ(pt.getFieldAs<double>(pdal::Dimension::Id::X),-44.300098);
    ASSERT_FLOAT_EQ(pt.getFieldAs<double>(pdal::Dimension::Id::Y),-1.132100);
    ASSERT_FLOAT_EQ(pt.getFieldAs<double>(pdal::Dimension::Id::Z),0.335800);
    ASSERT_FLOAT_EQ(pt.getFieldAs<double>(pdal::Dimension::Id::Red),0);
    ASSERT_FLOAT_EQ(pt.getFieldAs<double>(pdal::Dimension::Id::Green),65535);
    ASSERT_FLOAT_EQ(pt.getFieldAs<double>(pdal::Dimension::Id::Blue),0);
    ASSERT_FLOAT_EQ(pt.getFieldAs<double>(pdal::Dimension::Id::Intensity),0);

    auto pt2 = cloud->point(1);
    ASSERT_FLOAT_EQ(pt2.getFieldAs<double>(pdal::Dimension::Id::X),-44.506901);
    ASSERT_FLOAT_EQ(pt2.getFieldAs<double>(pdal::Dimension::Id::Y),-0.886000);
    ASSERT_FLOAT_EQ(pt2.getFieldAs<double>(pdal::Dimension::Id::Z),0.328600);
}

PointViewSet readertest_readE57(std::string filename,PointTableRef table)
{
    Options ops;
    ops.add("filename",filename);
    E57Reader reader;
    reader.setOptions(ops);
    reader.prepare(table);
    return reader.execute(table);
}

TEST(E57Reader, testMultipleClouds) 
{
    PointTable table;
    PointViewSet viewSet = readertest_readE57(Support::datapath("e57/A_B.e57"),table);
    ASSERT_EQ(viewSet.size(),1u);
    auto cloud = *viewSet.begin();
    ASSERT_EQ(cloud->size(),6u);

    PointTable tableA;
    PointViewSet viewSetA = readertest_readE57(Support::datapath("e57/A4.e57"),tableA);
    auto cloudA = *viewSetA.begin();

    PointTable tableB;
    PointViewSet viewSetB = readertest_readE57(Support::datapath("e57/B2.e57"),tableB);
    auto cloudB = *viewSetB.begin();

    auto expectedDimensions = {pdal::Dimension::Id::X,pdal::Dimension::Id::Y,pdal::Dimension::Id::Z,
        pdal::Dimension::Id::Red,pdal::Dimension::Id::Green,pdal::Dimension::Id::Blue};
    for (int i =0; i < 2;i++)
    {
        auto ptB = cloudB->point(i);
        auto pt = cloud->point(i);
        for (auto& dim: expectedDimensions)
        {
            ASSERT_FLOAT_EQ(pt.getFieldAs<double>(dim),ptB.getFieldAs<double>(dim));
        }
    }

    for (int i =2; i < 6;i++)
    {
        auto ptA = cloudA->point(i-2);
        auto pt = cloud->point(i);
        for (auto& dim: expectedDimensions)
        {
            ASSERT_FLOAT_EQ(pt.getFieldAs<double>(dim),ptA.getFieldAs<double>(dim));
        }
    }
}

TEST(E57Reader, testTransformMerge) 
{
    PointTable table;
    PointViewSet viewSet = readertest_readE57(Support::datapath("e57/A_moved_B.e57"),table);
    ASSERT_EQ(viewSet.size(),1u);
    auto cloud = *viewSet.begin();
    ASSERT_EQ(cloud->size(),6u);

    PointTable tableA;
    PointViewSet viewSetA = readertest_readE57(Support::datapath("e57/A4_moved.e57"),tableA);
    auto cloudA = *viewSetA.begin();

    PointTable tableB;
    PointViewSet viewSetB = readertest_readE57(Support::datapath("e57/B2.e57"),tableB);
    auto cloudB = *viewSetB.begin();

    auto expectedDimensions = {pdal::Dimension::Id::X,pdal::Dimension::Id::Y,pdal::Dimension::Id::Z,
        pdal::Dimension::Id::Red,pdal::Dimension::Id::Green,pdal::Dimension::Id::Blue};
    for (int i =0; i < 2;i++)
    {
        auto ptB = cloudB->point(i);
        auto pt = cloud->point(i);
        for (auto& dim: expectedDimensions)
        {
            ASSERT_FLOAT_EQ(pt.getFieldAs<double>(dim),ptB.getFieldAs<double>(dim));
        }
    }

    for (int i =2; i < 6;i++)
    {
        auto ptA = cloudA->point(i-2);
        auto pt = cloud->point(i);
        for (auto& dim: expectedDimensions)
        {
            ASSERT_FLOAT_EQ(pt.getFieldAs<double>(dim),ptA.getFieldAs<double>(dim));
        }
    }
}

