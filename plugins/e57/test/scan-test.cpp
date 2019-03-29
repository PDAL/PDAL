#include <gtest/gtest.h>
#include "Support.hpp"

#include "../io/scan.hpp"
#include "../io/e57reader.hpp"

using namespace pdal;

TEST(ScanTest,testScanCtr) 
{
    E57Reader reader(Support::datapath("e57/A4.e57"));
    auto scans = reader.getScans();
    auto firstScan = scans[0];
    ASSERT_EQ(firstScan->getNumPoints(),4);
}

TEST(ScanTest,getDimension) 
{
    E57Reader reader(Support::datapath("e57/A4.e57"));
    auto scans = reader.getScans();
    auto firstscan = scans[0];

    auto dimensions = firstscan->getDimensions();
    ASSERT_EQ(dimensions.size(),7);
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
    ASSERT_EQ(pts.childCount(),firstScan->getNumPoints());
    ASSERT_EQ(pts.childCount(),2);
    auto secondScan = scans[1];
    pts = secondScan->getPoints();
    ASSERT_EQ(pts.childCount(),secondScan->getNumPoints());
    ASSERT_EQ(pts.childCount(),4);
}