/******************************************************************************
* Copyright (c) 2016, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor thenames of its contributors
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

#include <pdal/pdal_test_main.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>

#include "Support.hpp"

namespace pdal
{

TEST(NitfWrap, las)
{
    std::string exeName(Support::binpath(Support::exename("nitfwrap")));

    std::string output;

    FileUtils::deleteFile("simple.ntf");
    FileUtils::deleteFile("simple.las");
    FileUtils::deleteFile("simple.las.save");
    Utils::run_shell_command("cp " + Support::datapath("las/simple.las") +
        " .",  output);
    Utils::run_shell_command("cp simple.las simple.las.save", output);
    Utils::run_shell_command(exeName + " simple.las", output);
    FileUtils::deleteFile("simple.las");
    Utils::run_shell_command(exeName + " -u simple.ntf", output);
    uint32_t ret = Support::diff_files("simple.las", "simple.las.save");
    EXPECT_EQ(ret, 0u);
    FileUtils::deleteFile("simple.ntf");
    FileUtils::deleteFile("simple.las");
    FileUtils::deleteFile("simple.las.save");
}

TEST(NitfWrap, altPath)
{
    std::string exeName(Support::binpath(Support::exename("nitfwrap")));

    std::string output;

    FileUtils::deleteFile("foo");
    FileUtils::deleteFile("bar");
    FileUtils::deleteFile("baz");
    Utils::run_shell_command("cp " + Support::datapath("las/simple.las") +
        " foo",  output);
    Utils::run_shell_command(exeName + " foo bar", output);
    Utils::run_shell_command(exeName + " -u bar baz", output);
    uint32_t ret = Support::diff_files("foo", "baz");
    EXPECT_EQ(ret, 0u);
    FileUtils::deleteFile("foo");
    FileUtils::deleteFile("bar");
    FileUtils::deleteFile("baz");
}

TEST(NitfWrap, bpf)
{
    std::string exeName(Support::binpath(Support::exename("nitfwrap")));

    std::string output;

    FileUtils::deleteFile("autzen-dd.bpf");
    FileUtils::deleteFile("autzen-dd.ntf");
    FileUtils::deleteFile("autzen-dd.bpf.save");
    Utils::run_shell_command("cp " + Support::datapath("bpf/autzen-dd.bpf") +
        " .",  output);
    Utils::run_shell_command("cp autzen-dd.bpf autzen-dd.bpf.save", output);
    Utils::run_shell_command(exeName + " autzen-dd.bpf", output);
    FileUtils::deleteFile("autzen-dd.bpf");
    Utils::run_shell_command(exeName + " -u autzen-dd.ntf", output);
    uint32_t ret = Support::diff_files("autzen-dd.bpf", "autzen-dd.bpf.save");
    EXPECT_EQ(ret, 0u);
    FileUtils::deleteFile("autzen-dd.bpf");
    FileUtils::deleteFile("autzen-dd.ntf");
    FileUtils::deleteFile("autzen-dd.bpf.save");
}

} // namespace pdal
