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
#include <pdal/util/Utils.hpp>

#include "Support.hpp"

using namespace pdal;

namespace
{
std::string appName()
{
    return Support::binpath("pdal merge");
}
}

TEST(Merge, pdalinfoTest_no_input)
{
    const std::string cmd = appName() + " 2>&1";

    std::string output;
    EXPECT_EQ(Utils::run_shell_command(cmd, output), 1);

    const std::string expected = "PDAL: Must specify an input and output file.";
    EXPECT_EQ(output.substr(0, expected.length()), expected);
}


TEST(Merge, Simple)
{
    std::string file1(Support::datapath("las/utm15.las"));
    std::string file2(Support::datapath("las/utm17.las"));
    std::string outfile(Support::temppath("out.las"));
    std::string cmd = appName() + " " + file1 + " " + file2 + " " + outfile +
        " 2>/dev/null";

    std::string output;
    EXPECT_EQ(Utils::run_shell_command(cmd, output), 0);

    std::string dump(Support::binpath("lasdump"));

    cmd = dump + " " + outfile;

    Utils::run_shell_command(cmd, output);
    EXPECT_TRUE(output.find("Point count: 11") != std::string::npos);

    FileUtils::deleteFile(outfile);
}


TEST(Merge, Args)
{
    std::string file1(Support::datapath("las/utm15.las"));
    std::string file2(Support::datapath("las/utm17.las"));
    std::string outfile(Support::temppath("out.las"));
    std::string cmd = appName() +
        " --writers.las.scale_x=.001 --writers.las.offset_x=289814 " + file1 +
        " " + file2 + " " + outfile;

    std::string output;
    EXPECT_EQ(Utils::run_shell_command(cmd, output), 0);

    std::string dump(Support::binpath("lasdump"));
    cmd = dump + " " + outfile;
    Utils::run_shell_command(cmd, output);

    EXPECT_TRUE(
        output.find("Scales X/Y/Z: 0.001/0.01/0.01") != std::string::npos);
    EXPECT_TRUE(
        output.find("Offsets X/Y/Z: 289814/0/0") != std::string::npos);

    FileUtils::deleteFile(outfile);
}

