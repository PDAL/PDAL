/******************************************************************************
* Copyright (c) 2012, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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
#include <pdal/Log.hpp>
#include <pdal/util/FileUtils.hpp>
#include "Support.hpp"

namespace pdal
{

// Make sure that we properly throw away log stuff that isn't of the right
// level and that we generally log correctly.
TEST(Log, t1)
{
    std::string filename(Support::temppath("t1"));
    FileUtils::deleteFile(filename);

    // Scope makes sure file gets closed.
    {
        LogPtr l(Log::makeLog("", filename));

        l->setLevel(LogLevel::Debug);

        l->get(LogLevel::Debug) << "debug\n";
        l->get(LogLevel::Debug5) << "debug5\n";
        l->get(LogLevel::Info) << "info\n";
    }

    EXPECT_TRUE(Support::compare_text_files(filename,
        Support::datapath("logs/t1")));
}

// Make sure that devnull thing works.
TEST(Log, t2)
{
    std::string in(Support::datapath("las/utm15.las"));
    std::string out(Support::temppath("out.las"));

    FileUtils::deleteFile(out);
    std::string cmd = Support::binpath(Support::exename("pdal")) +
        " translate --log devnull -v Debug " + in + " " + out;

    std::string output;
    int stat = Utils::run_shell_command(cmd, output);
    EXPECT_EQ(stat, 0);
    EXPECT_EQ(output.size(), 0u);

    cmd = Support::binpath(Support::exename("pdal")) +
        " translate -v Debug " + in + " " + out + " 2>&1";
    stat = Utils::run_shell_command(cmd, output);
    EXPECT_EQ(stat, 0);
    EXPECT_NE(output.size(), 0u);

    FileUtils::deleteFile(out);
}

}
