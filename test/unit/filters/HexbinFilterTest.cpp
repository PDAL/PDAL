/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
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

#include <io/LasReader.hpp>
#include <filters/private/hexer/HexGrid.hpp>
#include <filters/private/hexer/H3grid.hpp>

#include <pdal/SpatialReference.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/FileUtils.hpp>

#include <h3api.h>

#include "filters/HexBinFilter.hpp"
#include "Support.hpp"

using namespace pdal;

void printChildren(std::ostream& out, MetadataNode m, int depth = 0)
{
    std::vector<MetadataNode> children = m.children();
    for (auto mi = children.begin(); mi != children.end(); ++mi)
    {
        MetadataNode& c = *mi;
        for (int i = 0; i < depth; i++)
            out << "\t";
        out << c.name() << " : " << c.value() << "\n";
        printChildren(out, c, depth + 1);
    }
}

TEST(HexbinFilterTest, HexbinFilterTest_test_1)
{
    StageFactory f;

    Options options;
    options.add("filename", Support::datapath("las/hextest.las"));

    Stage* reader(f.createStage("readers.las"));
    EXPECT_TRUE(reader);
    reader->setOptions(options);

    Stage* hexbin(f.createStage("filters.hexbin"));

    Options hexOptions;
    hexOptions.add("output_tesselation", true);
    hexOptions.add("sample_size", 5000);
    hexOptions.add("threshold", 1);
    hexOptions.add("edge_length", 0.666666666);
    EXPECT_TRUE(hexbin);
    hexbin->setOptions(hexOptions);
    hexbin->setInput(*reader);

    PointTable table;

    hexbin->prepare(table);
    hexbin->execute(table);

    MetadataNode m = table.metadata();
    m = m.findChild(hexbin->getName());

    std::string filename = Support::temppath("hexbin.txt");
    std::ofstream out(filename);
    printChildren(out, m);
    out.close();
    FileUtils::deleteFile(filename);
}

// Test that we create proper WKT for geometry with islands.
TEST(HexbinFilterTest, HexGrid_issue_2507)
{
    hexer::HexGrid grid(1);

    // This is an arrangement with two holes.  One of the holes has two
    // islands and one of those islands has a hole.
    // Here's a link to the picture.  The green numbers indicate the hexes
    // listed here:
    // https://photos.app.goo.gl/P3B3mU4Zre6zADEQ6
    grid.setHexes( {
        {0, 3}, {0, 4}, {0,5}, {0, 6},
        {1, 2}, {1, 6},
        {2, 2}, {2, 4}, {2, 5}, {2, 7},
        {3, 1}, {3, 3}, {3, 5}, {3, 7},
        {4, 1}, {4, 2}, {4, 4}, {4, 5}, {4, 8},
        {5, 0}, {5, 2}, {5, 6}, {5, 8},
        {6, 1}, {6, 3}, {6, 4}, {6, 8},
        {7, 1}, {7, 3}, {7, 4}, {7, 5}, {7, 7},
        {8, 2}, {8, 3}, {8, 4}, {8, 5}, {8, 6}, {8, 7}
    } );

    grid.findShapes();
    grid.findParentPaths();

    pdal::Utils::OStringStreamClassicLocale oss;
    grid.toWKT(oss);
    std::string s(oss.str());

    std::string test =
    R"delim(MULTIPOLYGON (((-6 -1, -5 -0.5, -5 -0.5, -5 -0.5, -4 -1, -4 -1, -3 -1.5, -3 -1.5, -2 -2, -2 -2, -1 -2.5, -1 -2.5, 0 -3, 0 -3, 0 -3, 0 -4, 0 -4, 0 -5, 0 -5, 0 -6, 0 -6, 0 -6, -1 -6.5, -1 -6.5, -2 -7, -2 -7, -3 -7.5, -3 -7.5, -4 -8, -4 -8, -5 -8.5, -5 -8.5, -5 -8.5, -6 -8, -6 -8, -7 -7.5, -7 -7.5, -8 -7, -8 -7, -8 -7, -8 -6, -8 -6, -8 -5, -8 -5, -8 -4, -8 -4, -8 -3, -8 -3, -8 -2, -8 -2, -8 -2, -7 -1.5, -7 -1.5, -6 -1, -6 -1), (-6 -8, -7 -7.5, -7 -7.5, -8 -7, -8 -6, -7 -5.5, -7 -5.5, -7 -5.5, -7 -4.5, -6 -4, -6 -4, -6 -4, -6 -3, -5 -2.5, -5 -2.5, -4 -2, -4 -2, -3 -1.5, -2 -2, -2 -2, -1 -2.5, -1 -2.5, 0 -3, 0 -4, 0 -4, 0 -5, 0 -5, 0 -6, -1 -6.5, -1 -6.5, -2 -7, -2 -7, -3 -7.5, -3 -7.5, -4 -8, -4 -8, -5 -8.5, -6 -8, -6 -8), (-6 -3, -6 -3, -7 -3.5, -8 -3, -8 -2, -7 -1.5, -7 -1.5, -6 -1, -6 -1, -5 -0.5, -4 -1, -4 -2, -5 -2.5, -5 -2.5, -6 -3)), ((-5 -6.5, -5 -6.5, -5 -6.5, -5 -6.5, -5 -6.5, -5 -6.5, -5 -6.5)), ((-4 -4, -3 -3.5, -3 -3.5, -3 -3.5, -2 -4, -2 -4, -2 -4, -2 -5, -2 -5, -2 -5, -3 -5.5, -3 -5.5, -3 -5.5, -4 -5, -4 -5, -4 -5, -4 -4, -4 -4, -4 -4), (-3 -5.5, -4 -5, -4 -4, -3 -3.5, -2 -4, -2 -5, -3 -5.5))))delim";
    EXPECT_EQ(s, test);
}

TEST(HexbinFilterTest, H3Grid_issue_2507)
{
    hexer::H3Grid grid(10, 1);

    // This is the same grid as the previous test, but adapted for
    // the H3 IJ indexing scheme.
    grid.setHexes( {
        {5, 2}, {5, 3},
        {6, 2}, {6, 4},
        {7, 3}, {7, 4},
        {3, 0}, {3, 1}, {3, 2}, {3, 3}, {3, 4}, {3, 5},
        {4, 0}, {4, 4}, {4, 6},
        {5, 0}, {5, 2}, {5, 3}, {5, 5}, {5, 7},
        {6, 0}, {6, 2}, {6, 4}, {6, 6}, {6, 8},
        {7, 1}, {7, 3}, {7, 4}, {7, 7}, {7, 8},
        {8, 2}, {8, 7}, {8, 8},
        {9, 3}, {9, 5}, {9, 7}, {9, 8},
        {10, 4}, {10, 8},
        {11, 5}, {11, 6}, {11, 7}, {11, 8},
    } );

    LatLng location{PDALH3degsToRads(40.689167), PDALH3degsToRads(-74.044444)};
    int resolution = grid.getRes();
    H3Index index;
    EXPECT_EQ(PDALH3latLngToCell(&location, resolution, &index), E_SUCCESS); 
    grid.setOrigin(index);

    grid.findShapes();
    grid.findParentPaths();

    pdal::Utils::OStringStreamClassicLocale oss;
    grid.toWKT(oss);
    std::string s(oss.str());

    std::string test =
    R"delim(MULTIPOLYGON (((-70.147 40.0044, -70.1464 40.0039, -70.1455 40.004, -70.1449 40.0035, -70.144 40.0036, -70.1435 40.0031, -70.1426 40.0032, -70.142 40.0027, -70.1423 40.0021, -70.1417 40.0016, -70.1421 40.001, -70.1415 40.0005, -70.1418 39.9999, -70.1412 39.9994, -70.1415 39.9988, -70.141 39.9983, -70.1413 39.9976, -70.1407 39.9971, -70.141 39.9965, -70.1419 39.9964, -70.1423 39.9958, -70.1432 39.9957, -70.1435 39.995, -70.1444 39.9949, -70.1447 39.9943, -70.1456 39.9942, -70.1462 39.9947, -70.1471 39.9946, -70.1477 39.9951, -70.1486 39.9949, -70.1492 39.9954, -70.1501 39.9953, -70.1507 39.9958, -70.1516 39.9957, -70.1521 39.9962, -70.153 39.9961, -70.1536 39.9966, -70.1533 39.9972, -70.1539 39.9977, -70.1535 39.9983, -70.1541 39.9988, -70.1538 39.9994, -70.1544 39.9999, -70.154 40.0006, -70.1531 40.0007, -70.1528 40.0013, -70.1519 40.0014, -70.1516 40.002, -70.1507 40.0022, -70.1504 40.0028, -70.1495 40.0029, -70.1491 40.0035, -70.1482 40.0036, -70.1479 40.0043, -70.147 40.0044)),((-70.1502 39.9999, -70.1496 39.9994, -70.1499 39.9988, -70.1494 39.9983, -70.1497 39.9977, -70.1491 39.9972, -70.1494 39.9966, -70.1488 39.9961, -70.1479 39.9962, -70.1474 39.9957, -70.1465 39.9958, -70.1459 39.9953, -70.145 39.9954, -70.1447 39.996, -70.1438 39.9962, -70.1434 39.9968, -70.1425 39.9969, -70.1422 39.9975, -70.1428 39.998, -70.1425 39.9986, -70.143 39.9991, -70.1427 39.9998, -70.1433 40.0003, -70.143 40.0009, -70.1435 40.0014, -70.1432 40.002, -70.1438 40.0025, -70.1447 40.0024, -70.1453 40.0029, -70.1462 40.0028, -70.1467 40.0033, -70.1477 40.0031, -70.148 40.0025, -70.1474 40.002, -70.1477 40.0014, -70.1486 40.0013, -70.149 40.0007, -70.1499 40.0005, -70.1502 39.9999)),((-70.1529 39.9996, -70.1523 39.9991, -70.1526 39.9984, -70.1521 39.9979, -70.1524 39.9973, -70.1518 39.9968, -70.1509 39.9969, -70.1506 39.9976, -70.1512 39.9981, -70.1508 39.9987, -70.1514 39.9992, -70.1511 39.9998, -70.1517 40.0003, -70.1526 40.0002, -70.1529 39.9996)),((-70.146 39.9999, -70.1454 39.9994, -70.1445 39.9995, -70.1439 39.999, -70.1443 39.9984, -70.1437 39.9979, -70.144 39.9973, -70.1449 39.9972, -70.1452 39.9965, -70.1461 39.9964, -70.1467 39.9969, -70.1476 39.9968, -70.1482 39.9973, -70.1479 39.9979, -70.1484 39.9984, -70.1481 39.999, -70.1472 39.9992, -70.1469 39.9998, -70.146 39.9999)),((-70.145 40.0018, -70.1444 40.0013, -70.1448 40.0006, -70.1457 40.0005, -70.1462 40.001, -70.1459 40.0016, -70.145 40.0018)),((-70.147 39.998, -70.1464 39.9975, -70.1455 39.9977, -70.1452 39.9983, -70.1457 39.9988, -70.1466 39.9987, -70.147 39.998))))delim";
    EXPECT_EQ(s, test);
}
