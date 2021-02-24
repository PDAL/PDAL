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

#include "Support.hpp"

using namespace pdal;

void checkFile(int i, int j, int lines, double xoff = 0, double yoff = 0)
{
    std::string header;
    std::string t(Support::temppath("tile/out" +
        std::to_string(i) + "_" + std::to_string(j) + ".txt"));
    std::ifstream in(t);
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

    EXPECT_EQ(FileUtils::directoryList(Support::temppath("tile")).size(), 9U);
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
