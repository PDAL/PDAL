/******************************************************************************
* Copyright (c) 2015, Hobu Inc., (info@hobu.co)
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

#include <iostream>
#include <string>

#include <pdal/pdal_test_main.hpp>

#include <pdal/util/FileUtils.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(TIndex, test1)
{
    std::string inSpec(Support::datapath("tindex/*.txt"));
    std::string outSpec(Support::temppath("tindex.out"));
    std::string outPoints(Support::temppath("points.txt"));

    std::string cmd = Support::binpath("pdal") + " tindex create " +
        outSpec + " \"" + inSpec + "\"";

    FileUtils::deleteDirectory(outSpec);

    std::string output;
    Utils::run_shell_command(cmd, output);

    cmd = Support::binpath("pdal") + " --verbose=info tindex merge " +
        outSpec + " " + outPoints + " --log=stdout "
        "--bounds=\"([1.25, 3],[1.25, 3])\"";

    FileUtils::deleteFile(outPoints);
    Utils::run_shell_command(cmd, output);
    std::string::size_type pos = output.find("Merge filecount: 3");
    EXPECT_NE(pos, std::string::npos);

    cmd = Support::binpath("pdal") + " --verbose=info tindex merge " +
        outSpec + " " + outPoints + " --log=stdout "
        "--bounds=\"([1.25, 2],[1.25, 2])\"";
    FileUtils::deleteFile(outPoints);
    Utils::run_shell_command(cmd, output);
    pos = output.find("Merge filecount: 2");
    EXPECT_NE(pos, std::string::npos);

    cmd = Support::binpath("pdal") + " --verbose=info tindex merge " +
        outSpec + " " + outPoints + " --log=stdout "
        "--bounds=\"([1.25, 1.75],[1.25, 1.75])\"";
    FileUtils::deleteFile(outPoints);
    Utils::run_shell_command(cmd, output);
    pos = output.find("Merge filecount: 1");
    EXPECT_NE(pos, std::string::npos);
}

TEST(TIndex, test2)
{
    std::string inSpec(Support::datapath("tindex/*.txt"));
    std::string outSpec(Support::temppath("tindex.out"));

    FileUtils::deleteDirectory(outSpec);
    std::string cmd = Support::binpath("pdal") + " tindex create --stdin " +
        outSpec + " \"" + inSpec + "\" 2>&1";

    std::string output;
    Utils::run_shell_command(cmd, output);
    std::string::size_type pos = output.find("Can't specify both");
    EXPECT_NE(pos, std::string::npos);

    cmd = Support::binpath("pdal") + " tindex create --stdin " +
        outSpec + " --filespec=\"" + inSpec + "\" 2>&1";
    Utils::run_shell_command(cmd, output);
    pos = output.find("Can't specify both");
    EXPECT_NE(pos, std::string::npos);
}

// Indentical to test1, but filespec input comes from find command.
TEST(TIndex, test3)
{
// No find on Windows.
#ifndef _WIN32
    std::string outSpec(Support::temppath("tindex.out"));
    std::string outPoints(Support::temppath("points.txt"));

    std::string cmd = "find " + Support::datapath("tindex") +
        " -name \"*.txt\" | " + Support::binpath("pdal") +
        " tindex create --stdin " + outSpec;

    FileUtils::deleteDirectory(outSpec);

    std::string output;
    Utils::run_shell_command(cmd, output);

    cmd = Support::binpath("pdal") + " --verbose=info tindex merge " +
        outSpec + " " + outPoints + " --log=stdout "
        "--bounds=\"([1.25, 3],[1.25, 3])\"";

    FileUtils::deleteFile(outPoints);
    Utils::run_shell_command(cmd, output);
    std::string::size_type pos = output.find("Merge filecount: 3");
    EXPECT_NE(pos, std::string::npos);

    cmd = Support::binpath("pdal") + " --verbose=info tindex merge " +
        outSpec + " " + outPoints + " --log=stdout "
        "--bounds=\"([1.25, 2],[1.25, 2])\"";
    FileUtils::deleteFile(outPoints);
    Utils::run_shell_command(cmd, output);
    pos = output.find("Merge filecount: 2");
    EXPECT_NE(pos, std::string::npos);

    cmd = Support::binpath("pdal") + " --verbose=info tindex merge " +
        outSpec + " " + outPoints + " --log=stdout "
        "--bounds=\"([1.25, 1.75],[1.25, 1.75])\"";
    FileUtils::deleteFile(outPoints);
    Utils::run_shell_command(cmd, output);
    pos = output.find("Merge filecount: 1");
    EXPECT_NE(pos, std::string::npos);
#endif
}

