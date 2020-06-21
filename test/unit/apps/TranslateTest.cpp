/******************************************************************************
* Copyright (c) 2016, Hobu Inc.
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
*     * Neither the name of Hobu, Inc. nor the
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

#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>
#include "Support.hpp"

#include <iostream>
#include <sstream>
#include <string>

using namespace pdal;

static int runTranslate(std::string const& cmdline, std::string& output)
{
    const std::string cmd = Support::binpath(Support::exename("pdal")) +
        " translate";

    return Utils::run_shell_command(cmd + " " + cmdline, output);
}

TEST(TranslateTest, t1)
{
    std::string output;

    std::string in = Support::datapath("las/autzen_trim.las");
    std::string out = Support::temppath("out.las");

    EXPECT_EQ(runTranslate(in + " " + out, output), 0);
    EXPECT_EQ(runTranslate(in + " -f filters.stats " + out, output), 0);
    EXPECT_EQ(runTranslate(in + " " + out + " stats", output), 0);
    EXPECT_EQ(runTranslate(in + " " + out + " filters.stats", output), 0);
    EXPECT_NE(runTranslate(in + " " + out + " foobar", output), 0);
    EXPECT_NE(runTranslate(in + " " + out +
        " filters.stats --json filters.stats", output), 0);
    EXPECT_NE(runTranslate(in + " " + out +
        " --json filters.stats", output), 0);
}

// Tests for processing JSON input.
TEST(TranslateTest, t2)
{
    std::string output;

    std::string in = Support::datapath("las/autzen_trim.las");
    std::string out = Support::temppath("out.las");

    std::string json = " \
        { \
        \\\"pipeline\\\" : [ \
        { \\\"type\\\":\\\"filters.stats\\\" }, \
        { \\\"type\\\":\\\"filters.range\\\", \
          \\\"limits\\\":\\\"Z[0:100]\\\" } \
        ] \
        }";

    // Check that we work with just a bunch of filters.
    EXPECT_EQ(runTranslate(in + " " + out + " --json=\"" + json + "\"",
        output), 0);

    // Check that we fail with no input file.
    FileUtils::deleteFile("foo.las");
    EXPECT_NE(runTranslate("foo.las " + out + " --json=\"" + json + "\"",
        output), 0);

    // Check that we file with bad output file.
    EXPECT_NE(runTranslate(in + " foo.blam " +  " --json=\"" + json + "\"",
        output), 0);

    // Check that we work with no stages.
    json = " \
        { \
        \\\"pipeline\\\" : [ \
        ] \
        }";
    EXPECT_EQ(runTranslate(in + " " + out + " --json=\"" + json + "\"",
        output), 0);

    // Check that we work with only an input (not specified as such).
    json = " \
        { \
        \\\"pipeline\\\" : [ \
          \\\"badinput.las\\\" \
        ] \
        }";
    EXPECT_EQ(runTranslate(in + " " + out + " --json=\"" + json + "\"",
        output), 0);

    // Check that we work with an input and an output.
    json = " \
        { \
        \\\"pipeline\\\" : [ \
          \\\"badinput.las\\\", \
          \\\"badoutput.las\\\" \
        ] \
        }";
    EXPECT_EQ(runTranslate(in + " " + out + " --json=\"" + json + "\"",
        output), 0);

    // Check that we work with only an output.
    json = " \
        { \
        \\\"pipeline\\\" : [ \
          { \
          \\\"type\\\":\\\"writers.las\\\", \
          \\\"filename\\\":\\\"badoutput.las\\\" \
          } \
        ] \
        }";
    EXPECT_EQ(runTranslate(in + " " + out + " --json=\"" + json + "\"",
        output), 0);

    // Check that we work with only an input.
    json = " \
        { \
        \\\"pipeline\\\" : [ \
          { \
          \\\"type\\\":\\\"readers.las\\\", \
          \\\"filename\\\":\\\"badinput.las\\\" \
          } \
        ] \
        }";
    EXPECT_EQ(runTranslate(in + " " + out + " --json=\"" + json + "\"",
        output), 0);

    // Check that we fail with unchanined multiple writers.
    json = " \
        { \
        \\\"pipeline\\\" : [ \
          { \
          \\\"type\\\":\\\"writers.las\\\", \
          \\\"filename\\\":\\\"badoutput.las\\\" \
          }, \
          \\\"badoutput2.las\\\" \
        ] \
        }";
    EXPECT_EQ(runTranslate(in + " " + out + " --json=\"" + json + "\"",
        output), 0);

    // Check that we can handle chained writers.
    json = " \
        { \
        \\\"pipeline\\\" : [ \
          { \
          \\\"type\\\":\\\"writers.las\\\", \
          \\\"filename\\\":\\\"badoutput.las\\\", \
          \\\"tag\\\":\\\"mytag\\\" \
          }, \
          { \
          \\\"filename\\\":\\\"badoutput2.las\\\", \
          \\\"inputs\\\": \\\"mytag\\\" \
          } \
        ] \
        }";
    EXPECT_EQ(runTranslate(in + " " + out + " --json=\"" + json + "\"",
        output), 0);
}

TEST(TranslateTest, t3)
{
    std::string output;

    std::string in = Support::datapath("las/autzen_trim.las");
    std::string out = Support::temppath("out.las");
    std::string meta = Support::temppath("meta.json");

    EXPECT_EQ(runTranslate(in + " " + out + " --metadata " + meta, output), 0);
#ifndef _WIN32
    Utils::run_shell_command("grep -c readers.las " + meta, output);
    EXPECT_EQ(std::stoi(output), 1);
    Utils::run_shell_command("grep -c writers.las " + meta, output);
    EXPECT_EQ(std::stoi(output), 1);
#endif
}

// Make sure that repeated inputs ("groups" in this case), are propagated
// to stage.
TEST(TranslateTest, issue_2114)
{
    FileUtils::deleteFile(Support::temppath("out1.las"));
    FileUtils::deleteFile(Support::temppath("out2.las"));
    FileUtils::deleteFile(Support::temppath("out3.las"));
    FileUtils::deleteFile(Support::temppath("out4.las"));

    std::string output;
    std::string in = Support::datapath("las/autzen_trim.las");
    std::string out = Support::temppath("out#.las");

    EXPECT_FALSE(FileUtils::fileExists(Support::temppath("out1.las")));
    EXPECT_FALSE(FileUtils::fileExists(Support::temppath("out2.las")));
    EXPECT_FALSE(FileUtils::fileExists(Support::temppath("out3.las")));
    EXPECT_FALSE(FileUtils::fileExists(Support::temppath("out4.las")));

    EXPECT_EQ(runTranslate(in + " " + out + " -f returns "
        "--filters.returns.groups=last --filters.returns.groups=first",
        output), 0);

    EXPECT_TRUE(FileUtils::fileExists(Support::temppath("out1.las")));
    EXPECT_TRUE(FileUtils::fileExists(Support::temppath("out2.las")));
    EXPECT_FALSE(FileUtils::fileExists(Support::temppath("out3.las")));
    EXPECT_FALSE(FileUtils::fileExists(Support::temppath("out4.las")));
}

TEST(TranslateTest, issue2835)
{
    FileUtils::deleteFile("devnull");

    std::string output;
    std::string in = Support::datapath("las/autzen_trim.las");
    EXPECT_EQ(runTranslate(in + " " + "devnull", output), 0);
    EXPECT_FALSE(FileUtils::fileExists("devnull"));
}
