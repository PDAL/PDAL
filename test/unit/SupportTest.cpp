/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include "Support.hpp"

#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>

using namespace pdal;
using namespace std;

TEST(SupportTest, test_paths)
{
    // does the data path work?
    string data_file = Support::datapath("las/simple.las");
    EXPECT_TRUE(FileUtils::fileExists(data_file));

    // make sure we have read access
    istream* istr = FileUtils::openFile(data_file);
    string yow;
    *istr >> yow;
    FileUtils::closeFile(istr);

    // does the temp path work?
    string temp_file_ok = Support::temppath("README.txt");
    EXPECT_TRUE(FileUtils::fileExists(temp_file_ok));
    string temp_file = Support::temppath("my_temp_file.dat");
    EXPECT_TRUE(!FileUtils::fileExists(temp_file));

    // make sure we have write access to the temp dir
    ostream* ostr = FileUtils::createFile(temp_file);
    *ostr << "yow";
    FileUtils::closeFile(ostr);
    EXPECT_TRUE(FileUtils::fileExists(temp_file));
    EXPECT_TRUE(FileUtils::deleteFile(temp_file));
    EXPECT_TRUE(!FileUtils::fileExists(temp_file));

    // does binpath (and exename) work?
    string this_bin = Support::exename("pdal");
#ifdef _WIN32
    EXPECT_EQ(this_bin, "pdal.exe");
#else
    EXPECT_EQ(this_bin, "pdal");
#endif
    this_bin = Support::binpath(this_bin);
    EXPECT_TRUE(FileUtils::fileExists(this_bin));
}


TEST(SupportTest, test_diff_file)
{
    uint32_t diffs = 0;
    bool same = false;

    diffs = Support::diff_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data0.dat"));
    same = Support::compare_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data0.dat"));
    EXPECT_TRUE(diffs > 0);
    EXPECT_TRUE(same == false);

    diffs = Support::diff_files(Support::datapath("misc/data0.dat"), Support::datapath("misc/data1.dat"));
    same = Support::compare_files(Support::datapath("misc/data0.dat"), Support::datapath("misc/data1.dat"));
    EXPECT_TRUE(diffs > 0);
    EXPECT_TRUE(same == false);

    diffs = Support::diff_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data1.dat"));
    same = Support::compare_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data1.dat"));
    EXPECT_EQ(diffs, 0U);
    EXPECT_TRUE(same == true);

    diffs = Support::diff_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data2.dat"));
    same = Support::compare_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data2.dat"));
    EXPECT_EQ(diffs, 1U);
    EXPECT_TRUE(same == false);

    diffs = Support::diff_files(Support::datapath("misc/data2.dat"), Support::datapath("misc/data1.dat"));
    same = Support::compare_files(Support::datapath("misc/data2.dat"), Support::datapath("misc/data1.dat"));
    EXPECT_EQ(diffs, 1U);
    EXPECT_TRUE(same == false);

    diffs = Support::diff_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data3.dat"));
    same = Support::compare_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data3.dat"));
    EXPECT_EQ(diffs, 1U);
    EXPECT_TRUE(same == false);

    diffs = Support::diff_files(Support::datapath("misc/data3.dat"), Support::datapath("misc/data1.dat"));
    same = Support::compare_files(Support::datapath("misc/data3.dat"), Support::datapath("misc/data1.dat"));
    EXPECT_EQ(diffs, 1U);
    EXPECT_TRUE(same == false);
}


TEST(SupportTest, test_diff_file_ignorable)
{
    uint32_t diffs = 0;

    // no ignorable region
    {
        diffs = Support::diff_files(Support::datapath("misc/data4a.dat"), Support::datapath("misc/data4b.dat"));
        EXPECT_TRUE(diffs == 6U);
        EXPECT_TRUE(Support::compare_files(Support::datapath("misc/data4a.dat"), Support::datapath("misc/data4b.dat")) == false);
    }

    // treat whole file as ignorable
    {
        uint32_t start[1] = {0};
        uint32_t len[1] = {100};
        diffs = Support::diff_files(Support::datapath("misc/data4a.dat"), Support::datapath("misc/data4b.dat"), start, len, 1);
        EXPECT_EQ(diffs, 0U);
    }

    // just ignore the first region
    {
        uint32_t start[1] = {3};
        uint32_t len[1] = {4};
        diffs = Support::diff_files(Support::datapath("misc/data4a.dat"), Support::datapath("misc/data4b.dat"), start, len, 1);
        EXPECT_EQ(diffs, 2U);
    }

    // ignore the first and second regions
    {
        uint32_t start[2] = {3, 23};
        uint32_t len[2] = {4, 2};
        diffs = Support::diff_files(Support::datapath("misc/data4a.dat"), Support::datapath("misc/data4b.dat"), start, len, 2);
        EXPECT_EQ(diffs, 0U);
    }

    // ignore first and part of second region
    {
        uint32_t start[2] = {3, 22};
        uint32_t len[2] = {4, 2};
        diffs = Support::diff_files(Support::datapath("misc/data4a.dat"),
            Support::datapath("misc/data4b.dat"), start, len, 2);
        EXPECT_EQ(diffs, 1U);
    }
}


TEST(SupportTest, test_diff_text_file)
{
    uint32_t diffs = 0;
    bool same = false;
    diffs = Support::diff_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data0.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data0.txt"));
    EXPECT_TRUE(diffs > 0);
    EXPECT_TRUE(same == false);

    diffs = Support::diff_text_files(Support::datapath("misc/data0.txt"), Support::datapath("misc/data1.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data0.txt"), Support::datapath("misc/data1.txt"));
    EXPECT_TRUE(diffs > 0);
    EXPECT_TRUE(same == false);

    diffs = Support::diff_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data1.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data1.txt"));
    EXPECT_TRUE(diffs == 0);
    EXPECT_TRUE(same == true);

    diffs = Support::diff_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data2.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data2.txt"));
    EXPECT_TRUE(diffs == 1);
    EXPECT_TRUE(same == false);

    diffs = Support::diff_text_files(Support::datapath("misc/data2.txt"), Support::datapath("misc/data1.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data2.txt"), Support::datapath("misc/data1.txt"));
    EXPECT_TRUE(diffs == 1);
    EXPECT_TRUE(same == false);

    diffs = Support::diff_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data3.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data3.txt"));
    EXPECT_TRUE(diffs == 2);
    EXPECT_TRUE(same == false);

    diffs = Support::diff_text_files(Support::datapath("misc/data3.txt"), Support::datapath("misc/data1.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data3.txt"), Support::datapath("misc/data1.txt"));
    EXPECT_TRUE(diffs == 2);
    EXPECT_TRUE(same == false);
}


TEST(SupportTest, test_run_command)
{
    // amazingly, this command works under both dos *and* unix shells
    string cmd = "echo foo";

    string output;
    const int stat = Utils::run_shell_command(cmd, output);

    EXPECT_EQ(output.substr(0, 3), "foo");
    EXPECT_EQ(stat, 0);
}
