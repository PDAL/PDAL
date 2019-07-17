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

#include <gtest/gtest.h>
#include "Support.hpp"

#include "plugins/e57/io/Scan.hpp"
#include "plugins/e57/io/E57Reader.hpp"

using namespace pdal;

TEST(ScanTest,testScanCtr) 
{
    E57Reader reader(Support::datapath("e57/A4.e57"));
    auto scans = reader.getScans();
    auto firstScan = scans[0];
    ASSERT_EQ(firstScan->getNumPoints(),(pdal::point_count_t)4);
}

TEST(ScanTest,getDimension) 
{
    E57Reader reader(Support::datapath("e57/A4.e57"));
    auto scans = reader.getScans();
    auto firstscan = scans[0];

    auto dimensions = firstscan->getDimensions();
    ASSERT_EQ(dimensions.size(),(unsigned long)7);
    std::vector<std::string> expectedDimensions = {"cartesianX","cartesianY","cartesianZ",
        "colorRed","colorGreen","colorBlue","intensity"};
    for (auto dim: expectedDimensions)
    {
        ASSERT_TRUE(dimensions.find(dim) != dimensions.end());
    }
}

TEST(ScanTest,testGetPoints) 
{
    E57Reader reader(Support::datapath("e57/A_B.e57"));
    auto scans = reader.getScans();
    auto firstScan = scans[0];
    auto pts = firstScan->getPoints();
    ASSERT_EQ((pdal::point_count_t)pts.childCount(),firstScan->getNumPoints());
    ASSERT_EQ(pts.childCount(),2);
    auto secondScan = scans[1];
    pts = secondScan->getPoints();
    ASSERT_EQ((pdal::point_count_t)pts.childCount(),secondScan->getNumPoints());
    ASSERT_EQ(pts.childCount(),(int64_t)4);
}

TEST(ScanTest,testBbox)
{
    E57Reader reader(Support::datapath("e57/A4.e57"));
    auto scans = reader.getScans();
    auto firstScan = scans[0];
    auto box = firstScan->getBoundingBox();
    ASSERT_FLOAT_EQ(box.minx,-4.49522018432617188e+01);
    ASSERT_FLOAT_EQ(box.maxx,-4.43000984191894531e+01);
    ASSERT_FLOAT_EQ(box.miny,-1.34930002689361572);
    ASSERT_FLOAT_EQ(box.maxy,-8.85999977588653564e-01);
    ASSERT_FLOAT_EQ(box.minz,-6.07699990272521973e-01);
    ASSERT_FLOAT_EQ(box.maxz,3.69399994611740112e-01);
}