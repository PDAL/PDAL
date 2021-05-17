/******************************************************************************
* Copyright (c) 2016, Hobu Inc., (info@hobu.co)
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

#include <string>

#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"

namespace pdal
{

namespace
{
std::string appName()
{
    return Support::binpath("pdal");
}
} // unnamed namespace

TEST(PdalApp, load)
{
    std::string output;

    Utils::run_shell_command(appName() + " 2>&1", output);
    EXPECT_TRUE(output.find("Usage") != std::string::npos);

    Utils::run_shell_command(appName() + " sort 2>&1", output);
    EXPECT_TRUE(output.find("kernels.sort") != std::string::npos);

    Utils::run_shell_command(appName() + " foobar 2>&1", output);
    EXPECT_TRUE(output.find("not recognized") != std::string::npos);
}

TEST(PdalApp, log)
{
    std::string output;

    Utils::run_shell_command(appName() + " -v Debug 2>&1", output);
    EXPECT_TRUE(output.find("PDAL Debug)") != std::string::npos);

    output.clear();
    Utils::run_shell_command(appName() + " --verbose=3 2>&1", output);
    EXPECT_TRUE(output.find("PDAL Debug)") != std::string::npos);

    output.clear();
    Utils::run_shell_command(appName() + " 2>&1", output);
    EXPECT_TRUE(output.find("PDAL Debug)") == std::string::npos);

    output.clear();
    // With logtiming there should be a time after "Debug" and before the
    // closing paren.
    Utils::run_shell_command(appName() + " --logtiming 2>&1", output);
    EXPECT_TRUE(output.find("PDAL Debug)") == std::string::npos);
}

TEST(PdalApp, option_file)
{
    std::string output;

    std::string baseCommand = appName() + " translate " +
        Support::datapath("las/simple.las") + " " +
        Support::temppath("out.las") + " -f filters.range ";
    std::string command;

    Utils::run_shell_command(baseCommand + " 2>&1", output);
    EXPECT_TRUE(output.find("Missing value") != std::string::npos);

    command = baseCommand +
        "--filters.range.option_file=" + Support::datapath("apps/nofile") +
        " 2>&1";
    Utils::run_shell_command(command, output);
    EXPECT_TRUE(output.find("Can't read") != std::string::npos);

    command = baseCommand +
        "--filters.range.option_file=" +
        Support::datapath("apps/good_cmd_opt") +
        " 2>&1";
    Utils::run_shell_command(command, output);
    EXPECT_TRUE(output.empty());

    command = baseCommand +
        "--filters.range.option_file=" +
        Support::datapath("apps/good_json_opt") +
        " 2>&1";
    Utils::run_shell_command(command, output);
    EXPECT_TRUE(output.empty());

    command = baseCommand +
        "--filters.range.option_file=" +
        Support::datapath("apps/bad_cmd_opt") +
        " 2>&1";
    Utils::run_shell_command(command, output);
    EXPECT_TRUE(output.find("Unexpected argument") != std::string::npos);

    command = baseCommand +
        "--filters.range.option_file=" +
        Support::datapath("apps/bad_json_opt") +
        " 2>&1";
    Utils::run_shell_command(command, output);
    EXPECT_TRUE(output.find("Unexpected argument") != std::string::npos);
}

} // unnamed namespace
