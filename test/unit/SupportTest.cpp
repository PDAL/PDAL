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

#include <boost/test/unit_test.hpp>

#include "Support.hpp"
#include <pdal/FileUtils.hpp>


using namespace pdal;

BOOST_AUTO_TEST_SUITE(SupportTest)


BOOST_AUTO_TEST_CASE(test_paths)
{
    // does the data path work?
    const std::string data_file = Support::datapath("simple.las");
    BOOST_CHECK(FileUtils::fileExists(data_file));

    // make sure we have read access
    std::istream* istr = FileUtils::openFile(data_file);
    std::string yow;
    *istr >> yow;
    FileUtils::closeFile(istr);
    
    // does the temp path work?
    const std::string temp_file_ok = Support::temppath("README.txt");
    BOOST_CHECK(FileUtils::fileExists(temp_file_ok));
    const std::string temp_file = Support::temppath("my_temp_file.dat");
    BOOST_CHECK(!FileUtils::fileExists(temp_file));

    // make sure we have write access to the temp dir
    std::ostream* ostr = FileUtils::createFile(temp_file);
    *ostr << "yow";
    FileUtils::closeFile(ostr);
    BOOST_CHECK(FileUtils::fileExists(temp_file));
    BOOST_CHECK(FileUtils::deleteFile(temp_file));
    BOOST_CHECK(!FileUtils::fileExists(temp_file));

    // does binpath (and exename) work?
    std::string this_bin = Support::exename("pdal_test");
#ifdef PDAL_PLATFORM_WIN32
    BOOST_CHECK_EQUAL(this_bin, "pdal_test.exe");
#else
    BOOST_CHECK_EQUAL(this_bin, "pdal_test");
#endif
    this_bin = Support::binpath(this_bin);
    BOOST_CHECK(FileUtils::fileExists(this_bin));

    return;
}


BOOST_AUTO_TEST_CASE(test_diff_file)
{
    boost::uint32_t diffs = 0;
    bool same = false;

    diffs = Support::diff_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data0.dat"));
    same = Support::compare_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data0.dat"));
    BOOST_CHECK(diffs > 0);
    BOOST_CHECK(same == false);

    diffs = Support::diff_files(Support::datapath("misc/data0.dat"), Support::datapath("misc/data1.dat"));
    same = Support::compare_files(Support::datapath("misc/data0.dat"), Support::datapath("misc/data1.dat"));
    BOOST_CHECK(diffs > 0);
    BOOST_CHECK(same == false);

    diffs = Support::diff_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data1.dat"));
    same = Support::compare_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data1.dat"));
    BOOST_CHECK(diffs == 0);
    BOOST_CHECK(same == true);

    diffs = Support::diff_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data2.dat"));
    same = Support::compare_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data2.dat"));
    BOOST_CHECK(diffs == 1);
    BOOST_CHECK(same == false);

    diffs = Support::diff_files(Support::datapath("misc/data2.dat"), Support::datapath("misc/data1.dat"));
    same = Support::compare_files(Support::datapath("misc/data2.dat"), Support::datapath("misc/data1.dat"));
    BOOST_CHECK(diffs == 1);
    BOOST_CHECK(same == false);

    diffs = Support::diff_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data3.dat"));
    same = Support::compare_files(Support::datapath("misc/data1.dat"), Support::datapath("misc/data3.dat"));
    BOOST_CHECK(diffs == 2);
    BOOST_CHECK(same == false);

    diffs = Support::diff_files(Support::datapath("misc/data3.dat"), Support::datapath("misc/data1.dat"));
    same = Support::compare_files(Support::datapath("misc/data3.dat"), Support::datapath("misc/data1.dat"));
    BOOST_CHECK(diffs == 2);
    BOOST_CHECK(same == false);

    return;
}


BOOST_AUTO_TEST_CASE(test_diff_file_ignorable)
{
    boost::uint32_t diffs = 0;

    // no ignorable region
    {
        diffs = Support::diff_files(Support::datapath("misc/data4a.dat"), Support::datapath("misc/data4b.dat"));
        BOOST_CHECK(diffs == 6);
        BOOST_CHECK(Support::compare_files(Support::datapath("misc/data4a.dat"), Support::datapath("misc/data4b.dat")) == false);
    }

    // treat whole file as ignorable
    {
        boost::uint32_t start[1] = {0};
        boost::uint32_t len[1] = {100};
        diffs = Support::diff_files(Support::datapath("misc/data4a.dat"), Support::datapath("misc/data4b.dat"), start, len, 1);
        BOOST_CHECK(diffs == 0);
    }

    // just ignore the first region
    {
        boost::uint32_t start[1] = {3};
        boost::uint32_t len[1] = {4};
        diffs = Support::diff_files(Support::datapath("misc/data4a.dat"), Support::datapath("misc/data4b.dat"), start, len, 1);
        BOOST_CHECK(diffs == 2);
    }

    // ignore the first and second regions
    {
        boost::uint32_t start[2] = {3, 23};
        boost::uint32_t len[2] = {4, 2};
        diffs = Support::diff_files(Support::datapath("misc/data4a.dat"), Support::datapath("misc/data4b.dat"), start, len, 2);
        BOOST_CHECK(diffs == 0);
    }

    // ignore first and part of second region
    {
        boost::uint32_t start[2] = {3, 22};
        boost::uint32_t len[2] = {4, 2};
        diffs = Support::diff_files(Support::datapath("misc/data4a.dat"), Support::datapath("misc/data4b.dat"), start, len, 2);
        BOOST_CHECK(diffs == 1);
    }

    return;
}


BOOST_AUTO_TEST_CASE(test_diff_text_file)
{
    boost::uint32_t diffs = 0;
    bool same = false;

    diffs = Support::diff_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data0.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data0.txt"));
    BOOST_CHECK(diffs > 0);
    BOOST_CHECK(same == false);

    diffs = Support::diff_text_files(Support::datapath("misc/data0.txt"), Support::datapath("misc/data1.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data0.txt"), Support::datapath("misc/data1.txt"));
    BOOST_CHECK(diffs > 0);
    BOOST_CHECK(same == false);

    diffs = Support::diff_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data1.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data1.txt"));
    BOOST_CHECK(diffs == 0);
    BOOST_CHECK(same == true);

    diffs = Support::diff_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data2.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data2.txt"));
    BOOST_CHECK(diffs == 1);
    BOOST_CHECK(same == false);

    diffs = Support::diff_text_files(Support::datapath("misc/data2.txt"), Support::datapath("misc/data1.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data2.txt"), Support::datapath("misc/data1.txt"));
    BOOST_CHECK(diffs == 1);
    BOOST_CHECK(same == false);

    diffs = Support::diff_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data3.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data1.txt"), Support::datapath("misc/data3.txt"));
    BOOST_CHECK(diffs == 2);
    BOOST_CHECK(same == false);

    diffs = Support::diff_text_files(Support::datapath("misc/data3.txt"), Support::datapath("misc/data1.txt"));
    same = Support::compare_text_files(Support::datapath("misc/data3.txt"), Support::datapath("misc/data1.txt"));
    BOOST_CHECK(diffs == 2);
    BOOST_CHECK(same == false);

    return;
}


BOOST_AUTO_TEST_CASE(test_run_command)
{    
    // amazingly, this command works under both dos *and* unix shells
    const std::string cmd = "echo foo";

    std::string output;
    const int stat = Support::run_command(cmd, output);
    
    BOOST_CHECK_EQUAL(output.substr(0, 3), "foo");
    BOOST_CHECK_EQUAL(stat, 0);

    return;
}


BOOST_AUTO_TEST_SUITE_END()
