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
    R"delim(MULTIPOLYGON (((-5 -0.5, -5 -0.5, -6 -1, -6 -1, -7 -1.5, -7 -1.5, -8 -2, -8 -2, -8 -2, -8 -3, -8 -3, -8 -4, -8 -4, -8 -5, -8 -5, -8 -6, -8 -6, -8 -7, -8 -7, -8 -7, -7 -7.5, -7 -7.5, -6 -8, -6 -8, -5 -8.5, -5 -8.5, -5 -8.5, -4 -8, -4 -8, -3 -7.5, -3 -7.5, -2 -7, -2 -7, -1 -6.5, -1 -6.5, 0 -6, 0 -6, 0 -6, 0 -5, 0 -5, 0 -4, 0 -4, 0 -3, 0 -3, 0 -3, -1 -2.5, -1 -2.5, -2 -2, -2 -2, -3 -1.5, -3 -1.5, -4 -1, -4 -1, -5 -0.5, -5 -0.5), (-4 -2, -4 -1, -5 -0.5, -6 -1, -6 -1, -7 -1.5, -7 -1.5, -8 -2, -8 -3, -7 -3.5, -6 -3, -6 -3, -5 -2.5, -5 -2.5, -4 -2), (0 -6, 0 -5, 0 -5, 0 -4, 0 -4, 0 -3, -1 -2.5, -1 -2.5, -2 -2, -2 -2, -3 -1.5, -4 -2, -4 -2, -5 -2.5, -5 -2.5, -6 -3, -6 -4, -6 -4, -6 -4, -7 -4.5, -7 -5.5, -7 -5.5, -7 -5.5, -8 -6, -8 -7, -7 -7.5, -7 -7.5, -6 -8, -6 -8, -5 -8.5, -4 -8, -4 -8, -3 -7.5, -3 -7.5, -2 -7, -2 -7, -1 -6.5, -1 -6.5, 0 -6)), ((-3 -3.5, -3 -3.5, -4 -4, -4 -4, -4 -4, -4 -5, -4 -5, -4 -5, -3 -5.5, -3 -5.5, -3 -5.5, -2 -5, -2 -5, -2 -5, -2 -4, -2 -4, -2 -4, -3 -3.5, -3 -3.5), (-2 -5, -2 -4, -3 -3.5, -4 -4, -4 -5, -3 -5.5, -2 -5)), ((-5 -6.5, -5 -6.5, -5 -6.5, -5 -6.5, -5 -6.5, -5 -6.5, -5 -6.5))))delim";
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
    std::cout << s;
    //std::string test =
    //R"delim(MULTIPOLYGON (((-5 -0.5, -5 -0.5, -6 -1, -6 -1, -7 -1.5, -7 -1.5, -8 -2, -8 -2, -8 -2, -8 -3, -8 -3, -8 -4, -8 -4, -8 -5, -8 -5, -8 -6, -8 -6, -8 -7, -8 -7, -8 -7, -7 -7.5, -7 -7.5, -6 -8, -6 -8, -5 -8.5, -5 -8.5, -5 -8.5, -4 -8, -4 -8, -3 -7.5, -3 -7.5, -2 -7, -2 -7, -1 -6.5, -1 -6.5, 0 -6, 0 -6, 0 -6, 0 -5, 0 -5, 0 -4, 0 -4, 0 -3, 0 -3, 0 -3, -1 -2.5, -1 -2.5, -2 -2, -2 -2, -3 -1.5, -3 -1.5, -4 -1, -4 -1, -5 -0.5, -5 -0.5), (-4 -2, -4 -1, -5 -0.5, -6 -1, -6 -1, -7 -1.5, -7 -1.5, -8 -2, -8 -3, -7 -3.5, -6 -3, -6 -3, -5 -2.5, -5 -2.5, -4 -2), (0 -6, 0 -5, 0 -5, 0 -4, 0 -4, 0 -3, -1 -2.5, -1 -2.5, -2 -2, -2 -2, -3 -1.5, -4 -2, -4 -2, -5 -2.5, -5 -2.5, -6 -3, -6 -4, -6 -4, -6 -4, -7 -4.5, -7 -5.5, -7 -5.5, -7 -5.5, -8 -6, -8 -7, -7 -7.5, -7 -7.5, -6 -8, -6 -8, -5 -8.5, -4 -8, -4 -8, -3 -7.5, -3 -7.5, -2 -7, -2 -7, -1 -6.5, -1 -6.5, 0 -6)), ((-3 -3.5, -3 -3.5, -4 -4, -4 -4, -4 -4, -4 -5, -4 -5, -4 -5, -3 -5.5, -3 -5.5, -3 -5.5, -2 -5, -2 -5, -2 -5, -2 -4, -2 -4, -2 -4, -3 -3.5, -3 -3.5), (-2 -5, -2 -4, -3 -3.5, -4 -4, -4 -5, -3 -5.5, -2 -5)), ((-5 -6.5, -5 -6.5, -5 -6.5, -5 -6.5, -5 -6.5, -5 -6.5, -5 -6.5))))delim";
    //EXPECT_EQ(s, test);
}
