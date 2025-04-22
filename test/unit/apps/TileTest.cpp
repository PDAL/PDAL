/******************************************************************************
* Copyright (c) 2018, Hobu Inc., (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the names of contributors
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

#include <fstream>
#include <string>

#include <pdal/pdal_test_main.hpp>

#include <pdal/util/FileUtils.hpp>
#include <io/LasReader.hpp>

#include "Support.hpp"

using namespace pdal;

void checkFile(int i, int j, int lines, double xoff = 0, double yoff = 0)
{
    std::string header;
    std::string t(Support::temppath("tile/out" +
        std::to_string(i) + "_" + std::to_string(j) + ".txt"));
    Utils::ClassicLocaleStream<std::ifstream> in(t);
    std::getline(in, header);

    int count = 0;
    while (in)
    {
        char c;
        double x, y, z;

        in >> x >> c >> y >> c >> z;
        if (!in.good()) break;
        EXPECT_GE(x, xoff + (i * 10));
        EXPECT_LT(x, xoff + ((i + 1) * 10));
        EXPECT_GE(y, yoff + (j * 10));
        EXPECT_LT(y, yoff + ((j + 1) * 10));
        count++;
    }
    EXPECT_EQ(count, lines);
}

TEST(Tile, test1)
{
    std::string inSpec(Support::datapath("text/file*.txt"));
    std::string outSpec(Support::temppath("tile/out#.txt"));

    std::string baseCmd = Support::binpath("pdal") + " tile \"" +
        inSpec + "\" \"" + outSpec + "\" ";

    FileUtils::deleteDirectory(Support::temppath("tile"));
    FileUtils::createDirectory(Support::temppath("tile"));

    std::string output;
    std::string cmd = baseCmd + " --origin_x=0 --origin_y=0 --length=10";
    Utils::run_shell_command(cmd, output);

    EXPECT_EQ(FileUtils::directoryList(Support::temppath("tile")).size(), 11U);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            checkFile(i, j, 3);
}


TEST(Tile, test2)
{
    std::string output;
    std::string infile(Support::datapath("tile/tile.txt"));
    std::string file1(Support::temppath("tile/file1.las"));
    std::string file2(Support::temppath("tile/file2.las"));
    std::string file3(Support::temppath("tile/file3.las"));

    FileUtils::deleteDirectory(Support::temppath("tile"));
    FileUtils::createDirectory(Support::temppath("tile"));

    std::string tx1_cmd = Support::binpath("pdal") + " translate \"" +
        infile + "\" \"" + file1 +
        "\" --readers.text.override_srs=\"EPSG:2029\"";
    Utils::run_shell_command(tx1_cmd, output);

    std::string tx2_cmd = Support::binpath("pdal") + " translate \"" +
        infile + "\" \"" + file2 + "\" -f reprojection "
        "--filters.reprojection.in_srs=\"EPSG:2029\" "
        "--filters.reprojection.out_srs=\"EPSG:2031\"";
    Utils::run_shell_command(tx2_cmd, output);

    std::string tx3_cmd = Support::binpath("pdal") + " translate \"" +
        infile + "\" \"" + file3 + "\" -f reprojection "
        "--filters.reprojection.in_srs=\"EPSG:2029\" "
        "--filters.reprojection.out_srs=\"EPSG:2958\"";
    Utils::run_shell_command(tx3_cmd, output);

    std::string inSpec(Support::temppath("tile/file*.las"));
    std::string outSpec(Support::temppath("tile/out#.txt"));

    std::string baseCmd = Support::binpath("pdal") + " tile \"" +
        inSpec + "\" \"" + outSpec + "\" ";

    std::string cmd = baseCmd + " --origin_x=500000 --origin_y=5000000 "
        "--length=10 --out_srs=EPSG:2029 --writers.text.order=X,Y,Z "
        "--writers.text.keep_unspecified=false";
    Utils::run_shell_command(cmd, output);

    auto files = FileUtils::directoryList(Support::temppath("tile/out*.txt"));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            checkFile(i, j, 3, 500000, 5000000);
}

TEST(Tile, test3)
{
    std::string tmp(Support::temppath("tile/tile1.txt"));

    FileUtils::deleteDirectory(Support::temppath("tile"));
    FileUtils::createDirectory(Support::temppath("tile"));

    // Write the test output.
    std::ofstream out(tmp);
    out << "X Y Z\n";

    // Fill up a set with a strings representing the points. Then remove them when read
    // to make sure that we have exactly the same points read as written.
    std::set<std::string> points;
    // This gets us 25600 points.
    for (int x = 0; x < 160; ++x)
        for (int y = 0; y < 160; ++y)
        {
            // Stick points in our test set just to account for each of them.
            points.insert(std::to_string(x) + " " + std::to_string(y));

            // Write points to our temp text file.
            out << x << " " << y << " " << "0\n";
        }
    out.close();

    // Create a file with no points.
    tmp = Support::temppath("tile/tile2.txt");
    std::ofstream out2(tmp);
    out2 << "X Y Z\n";
    out2.close();

    // Create a file with one point.
    tmp = Support::temppath("tile/tile3.txt");
    std::ofstream out3(tmp);
    out3 << "X Y Z\n";
    out3 << "-1 -1 0\n";
    out3.close();
    points.insert("-1 -1");

    // Run tile on the test output.
    std::string outSpec(Support::temppath("tile/out#.las"));
    std::string baseCmd = Support::binpath("pdal") + " tile \"" +
        Support::temppath("tile/tile*.txt") + "\" \"" + outSpec + "\" ";

    std::string cmd = baseCmd + "--length=30";
    std::string output;
    Utils::run_shell_command(cmd, output);

    StringList files = FileUtils::directoryList(Support::temppath("tile"));

    EXPECT_EQ(points.size(), 160u * 160u + 1);
    for (std::string& f : files)
    {
        if (Utils::endsWith(f, ".las"))
        {
            Options o;
            o.add("filename", f);
            LasReader r;
            r.setOptions(o);
            PointTable t;
            r.prepare(t);
            PointViewSet s = r.execute(t);
            EXPECT_EQ(s.size(), 1u);
            PointViewPtr v = *s.begin();
            for (PointId i = 0; i < v->size(); ++i)
            {
                int x = v->getFieldAs<int>(Dimension::Id::X, i);
                int y = v->getFieldAs<int>(Dimension::Id::Y, i);

                // Make sure that the point is valid and was one we wrote.
                // Then remove the found point from our test set..
                std::string s = std::to_string(x) + " " + std::to_string(y);
                auto it = points.find(s);
                EXPECT_TRUE(it != points.end());
                points.erase(it);
            }
        }
    }
    // Make sure we removed all the points.
    EXPECT_EQ(points.size(), 0u);

    FileUtils::deleteDirectory(Support::temppath("tile"));
}
