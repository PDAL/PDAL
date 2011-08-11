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

using namespace pdal;

BOOST_AUTO_TEST_SUITE(SupportTest)


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

BOOST_AUTO_TEST_SUITE_END()
