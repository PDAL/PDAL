/******************************************************************************
* Copyright (c) 2025, Norman Barker (norman.barker@gmail.com)
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

#include <pdal/util/OStream.hpp>
#include <pdal/util/VSIIO.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/util/FileUtils.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(VSITest, test_tells)
{
    int bufSize = 2;
    Support::Tempfile temp(true);
    std::string tmp = temp.filename();
    EXPECT_TRUE(FileUtils::fileExists(tmp)==false);

    // write test
    VSI::VSIOStream* ostr = new VSI::VSIOStream(
        tmp, std::ios::out | std::ios::binary, bufSize);

    *ostr << "TEST";
    EXPECT_EQ(ostr->tellp(), 4);
    *ostr << "12345";
    EXPECT_EQ(ostr->tellp(), 9);
    delete ostr;

    EXPECT_EQ(FileUtils::fileExists(tmp), true);
    EXPECT_EQ(FileUtils::fileSize(tmp), 9U);

    // read test
    VSI::VSIIStream* istr = new VSI::VSIIStream(
        tmp, std::ios::in | std::ios::binary, bufSize);
    auto _ = istr->get();
    EXPECT_EQ(istr->tellg(), 1);
    char str[4];
    istr->get(str, 4);
    EXPECT_EQ(istr->tellg(), 4);
    EXPECT_EQ(std::string(str), "EST");
    std::string s;
    *istr >> s;
    EXPECT_EQ(s, "12345");
    EXPECT_EQ(istr->tellg(), -1); // EoF
    delete istr;
}

TEST(VSITest, test_seeks_small_buffer)
{
    int bufSize = 2;
    Support::Tempfile temp(true);
    std::string tmp = temp.filename();
    EXPECT_TRUE(FileUtils::fileExists(tmp) == false);

    // write test
    VSI::VSIOStream* ostr = new VSI::VSIOStream(
        tmp, std::ios::out | std::ios::binary, bufSize);

    ostr->seekp(10);
    *ostr << "TEST";
    EXPECT_EQ(ostr->tellp(), 14);

    // set seek to earlier position not equal to a buffer size boundary
    ostr->seekp(1);
    *ostr << "12345";
    EXPECT_EQ(ostr->tellp(), 6);

    delete ostr;

    EXPECT_EQ(FileUtils::fileExists(tmp), true);
    EXPECT_EQ(FileUtils::fileSize(tmp), 14U);

    // read test
    VSI::VSIIStream* istr = new VSI::VSIIStream(
        tmp, std::ios::in | std::ios::binary, bufSize);
    istr->seekg(10);
    std::string s;
    *istr >> s;
    EXPECT_EQ(s, "TEST");
    EXPECT_EQ(istr->tellg(), -1); // EoF

    EXPECT_EQ(istr->good(), false);
    istr->clear();
    istr->seekg(1);
    char str[6];
    istr->get(str, 6);
    EXPECT_EQ(istr->tellg(), 6);
    EXPECT_EQ(std::string(str), "12345");

    delete istr;
}

TEST(VSITest, test_seeks_large_buffer)
{
    int bufSize = 1024;
    Support::Tempfile temp(true);
    std::string tmp = temp.filename();
    EXPECT_TRUE(FileUtils::fileExists(tmp) == false);

    // write test
    VSI::VSIOStream* ostr = new VSI::VSIOStream(
        tmp, std::ios::out | std::ios::binary, bufSize);

    ostr->seekp(10);
    *ostr << "TEST";
    EXPECT_EQ(ostr->tellp(), 14);

    ostr->seekp(111);
    *ostr << "12345";
    EXPECT_EQ(ostr->tellp(), 116);

    delete ostr;

    EXPECT_EQ(FileUtils::fileExists(tmp), true);
    EXPECT_EQ(FileUtils::fileSize(tmp), 116U);

    // read test
    VSI::VSIIStream* istr = new VSI::VSIIStream(
        tmp, std::ios::in | std::ios::binary, bufSize);

    istr->seekg(10);
    char str[5];
    istr->get(str, 5);
    EXPECT_EQ(std::string(str), "TEST");

    istr->seekg(111);
    std::string s;
    *istr >> s;
    EXPECT_EQ(s, "12345");
    EXPECT_EQ(istr->tellg(), -1); // EoF

    delete istr;
}
