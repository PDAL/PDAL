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

#include <sstream>
#include <vector>

#include <pdal/util/portable_endian.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>
#include "Support.hpp"

#ifdef _WIN32
#include <winioctl.h>
#endif

namespace pdal
{

TEST(UtilsTest, test_random)
{
    const double rangeMin = 0.0;
    const double rangeMax = 100.0;
    const double avg = (rangeMax - rangeMin) / 2.0;
    const int iters = 1000;

    Utils::random_seed(17);

    // make sure we treat the bounds as inclusive
    double sum=0;
    for (int i=0; i<iters; i++)
    {
        const double x = Utils::random(rangeMin, rangeMax);
        EXPECT_TRUE(x >= rangeMin);
        EXPECT_TRUE(x <= rangeMax);

        sum += x;
    }

    sum = sum / iters;

    EXPECT_TRUE(sum <= avg + 0.1*avg);
    EXPECT_TRUE(sum >= avg - 0.1*avg);
}


TEST(UtilsTest, test_comparators)
{
    bool ok;

    {
        ok = Utils::compare_approx(1.001f, 1.0f, 0.0001f);
        EXPECT_TRUE(!ok);

        ok = Utils::compare_approx(1.001f, 1.0f, 0.001f);
        EXPECT_TRUE(!ok);

        ok = Utils::compare_approx(1.001f, 1.0f, 0.01f);
        EXPECT_TRUE(ok);

        ok = Utils::compare_approx(1.001f, 1.0f, 0.1f);
        EXPECT_TRUE(ok);
    }

    {
        ok = Utils::compare_approx(10, 12, 2);
        EXPECT_TRUE(ok);

        ok = Utils::compare_approx(10, 12, 3);
        EXPECT_TRUE(ok);

        ok = Utils::compare_approx(10, 12, 1);
        EXPECT_TRUE(!ok);
    }
}


TEST(UtilsTest, test_base64)
{
    std::vector<uint8_t> data;
    for (int i=0; i<2; i++) data.push_back((uint8_t)i);

    uint32_t begin_size(0);
    for (std::vector<uint8_t>::size_type i = 0; i < data.size(); ++i)
    {
        begin_size = begin_size + data[i];
    }

    std::string encoded = Utils::base64_encode(data);
    std::vector<uint8_t> decoded = Utils::base64_decode(encoded);

    uint32_t size(0);
    for (std::vector<uint8_t>::size_type i = 0; i < decoded.size(); ++i)
    {
        size = size + decoded[i];
    }

    EXPECT_EQ(decoded.size(), data.size());
    EXPECT_EQ(size, begin_size);
}

TEST(UtilsTest, blanks)
{
    std::string base("This is a test");
    std::string trail("This is a test   ");
    std::string lead("  This is a test");
    std::string both("  This is a test    ");
    std::string empty;

    std::string s = "This is a test  \t  ";
    Utils::trimTrailing(s);
    EXPECT_EQ(s, base);
    s = "";
    Utils::trimTrailing(s);
    EXPECT_EQ(s, empty);
    s = "  \t\t  ";
    Utils::trimTrailing(s);
    EXPECT_EQ(s, empty);
    s = base;
    Utils::trimTrailing(s);
    EXPECT_EQ(s, base);
    s = "  \t This is a test";
    Utils::trimLeading(s);
    EXPECT_EQ(s, base);
    s = "  \t  \t  ";
    Utils::trimLeading(s);
    EXPECT_EQ(s, empty);
    s = "";
    Utils::trimLeading(s);
    EXPECT_EQ(s, empty);
    s = base;
    Utils::trimLeading(s);
    EXPECT_EQ(s, base);
}

TEST(UtilsTest, split)
{
    std::vector<std::string> result;

    std::string input("This is a test");
    auto pred = [](char c)
        { return c == ' '; };

    result = Utils::split(input, pred);
    EXPECT_EQ(result.size(), 4U);
    EXPECT_EQ(result[0], "This");
    EXPECT_EQ(result[1], "is");
    EXPECT_EQ(result[2], "a");
    EXPECT_EQ(result[3], "test");

    input = "  This  is a test  ";

    result = Utils::split(input, pred);
    EXPECT_EQ(result.size(), 9U);
    EXPECT_EQ(result[0], "");
    EXPECT_EQ(result[1], "");
    EXPECT_EQ(result[2], "This");
    EXPECT_EQ(result[3], "");
    EXPECT_EQ(result[4], "is");
    EXPECT_EQ(result[5], "a");
    EXPECT_EQ(result[6], "test");
    EXPECT_EQ(result[7], "");
    EXPECT_EQ(result[8], "");
}

TEST(UtilsTest, splitChar)
{
    std::vector<std::string> result;

    std::string input("This is a test");

    result = Utils::split(input, ' ');
    EXPECT_EQ(result.size(), 4U);
    EXPECT_EQ(result[0], "This");
    EXPECT_EQ(result[1], "is");
    EXPECT_EQ(result[2], "a");
    EXPECT_EQ(result[3], "test");

    input = "  This  is a test  ";

    result = Utils::split(input, ' ');
    EXPECT_EQ(result.size(), 9U);
    EXPECT_EQ(result[0], "");
    EXPECT_EQ(result[1], "");
    EXPECT_EQ(result[2], "This");
    EXPECT_EQ(result[3], "");
    EXPECT_EQ(result[4], "is");
    EXPECT_EQ(result[5], "a");
    EXPECT_EQ(result[6], "test");
    EXPECT_EQ(result[7], "");
    EXPECT_EQ(result[8], "");

    input = "";
    result = Utils::split(input, ' ');
    EXPECT_EQ(result.size(), 0U);
}

TEST(UtilsTest, split2)
{
    std::vector<std::string> result;

    std::string input("This is a test");
    auto pred = [](char c)
        { return c == ' '; };

    result = Utils::split2(input, pred);
    EXPECT_EQ(result.size(), 4U);
    EXPECT_EQ(result[0], "This");
    EXPECT_EQ(result[1], "is");
    EXPECT_EQ(result[2], "a");
    EXPECT_EQ(result[3], "test");

    input = "  This  is a test  ";

    result = Utils::split2(input, pred);
    EXPECT_EQ(result.size(), 4U);
    EXPECT_EQ(result[0], "This");
    EXPECT_EQ(result[1], "is");
    EXPECT_EQ(result[2], "a");
    EXPECT_EQ(result[3], "test");

    auto pred2 = [](char c)
        { return c == ' ' || c == ','; };

    input = " , This,is ,a test , ";

    result = Utils::split2(input, pred2);
    EXPECT_EQ(result.size(), 4U);
    EXPECT_EQ(result[0], "This");
    EXPECT_EQ(result[1], "is");
    EXPECT_EQ(result[2], "a");
    EXPECT_EQ(result[3], "test");
}

TEST(UtilsTest, split2Char)
{
    std::vector<std::string> result;

    std::string input(",,This,is,,a,test,,,");

    result = Utils::split2(input, ',');
    EXPECT_EQ(result.size(), 4U);
    EXPECT_EQ(result[0], "This");
    EXPECT_EQ(result[1], "is");
    EXPECT_EQ(result[2], "a");
    EXPECT_EQ(result[3], "test");

    input = "";
    result = Utils::split2(input, ' ');
    EXPECT_EQ(result.size(), 0U);
}

TEST(UtilsTest, case)
{
    std::string s("This is a test");

    EXPECT_EQ("THIS IS A TEST", Utils::toupper(s));
    EXPECT_EQ("this is a test", Utils::tolower(s));

    s = "FOOBARBAZ";

    EXPECT_EQ("FOOBARBAZ", Utils::toupper(s));
    EXPECT_EQ("foobarbaz", Utils::tolower(s));

    s = "foo!bar.baz";
    EXPECT_EQ("FOO!BAR.BAZ", Utils::toupper(s));
    EXPECT_EQ(s, Utils::tolower(s));
}

TEST(UtilsTest, starts)
{
    std::string s("reference 1");

    EXPECT_TRUE(Utils::startsWith(s, "ref"));
    EXPECT_TRUE(Utils::startsWith(s, s));
    EXPECT_FALSE(Utils::startsWith(s, "reference 123"));
    EXPECT_FALSE(Utils::startsWith(s, "rawference 123"));
    EXPECT_TRUE(Utils::startsWith(s, ""));
}

TEST(UtilsTest, iequals)
{
    EXPECT_TRUE(Utils::iequals("", ""));
    EXPECT_TRUE(Utils::iequals("Foobar", "foobar"));
    EXPECT_TRUE(Utils::iequals("FOO.BAR~", "foo.Bar~"));
    EXPECT_FALSE(Utils::iequals("Foobar", "foobarbaz"));
    EXPECT_FALSE(Utils::iequals("Foobar", "foobat"));
}

TEST(UtilsTest, replaceAll)
{
    std::string s(" This  is a   test ");
    EXPECT_EQ(Utils::replaceAll(s, " ", "  "), "  This    is  a      test  ");
    EXPECT_EQ(Utils::replaceAll(s, "  ", " "), " This is a  test ");
    EXPECT_EQ(Utils::replaceAll(s, " ", "\""), "\"This\"\"is\"a\"\"\"test\"");
}

TEST(UtilsTest, escapeNonprinting)
{
    std::string s("CTRL-N,A,B,R,V: \n\a\b\r\v\x12\xe\x01");
    std::string out = Utils::escapeNonprinting(s);
    EXPECT_EQ(out, "CTRL-N,A,B,R,V: \\n\\a\\b\\r\\v\\x12\\x0e\\x01");
}

TEST(UtilsTest, wordWrap)
{
    std::string s;
    std::vector<std::string> output;

    s = "This   is   a    test    1234567890abcdefghij1234 a   ";
    output = Utils::wordWrap(s, 10, 12);
    EXPECT_EQ(output.size(), 5u);
    EXPECT_EQ(output[0], "This is a");
    EXPECT_EQ(output[1], "test");
    EXPECT_EQ(output[2], "1234567890");
    EXPECT_EQ(output[3], "abcdefghij");
    EXPECT_EQ(output[4], "1234 a");

    s = "";
    output = Utils::wordWrap(s, 10, 12);
    EXPECT_EQ(output.size(), 0u);

    s = "012345678901abcdefghij01234567";
    output = Utils::wordWrap(s, 10, 12);
    EXPECT_EQ(output.size(), 3u);
    EXPECT_EQ(output[0], "012345678901");
    EXPECT_EQ(output[1], "abcdefghij");
    EXPECT_EQ(output[2], "01234567");
}

TEST(UtilsTest, wordWrap2)
{
    std::string s;
    std::vector<std::string> output;

    s = "This   is   a    test    1234567890abcdefghij1234 a   ";
    output = Utils::wordWrap2(s, 10, 12);
    EXPECT_EQ(output.size(), 6u);
    EXPECT_EQ(output[0], "This   is   ");
    EXPECT_EQ(output[1], "a    ");
    EXPECT_EQ(output[2], "test    ");
    EXPECT_EQ(output[3], "1234567890");
    EXPECT_EQ(output[4], "abcdefghij");
    EXPECT_EQ(output[5], "1234 a   ");

    s = "";
    output = Utils::wordWrap2(s, 10, 12);
    EXPECT_EQ(output.size(), 0u);

    s = "012345678901abcdefghij01234567";
    output = Utils::wordWrap2(s, 10, 12);
    EXPECT_EQ(output.size(), 3u);
    EXPECT_EQ(output[0], "012345678901");
    EXPECT_EQ(output[1], "abcdefghij");
    EXPECT_EQ(output[2], "01234567");

    s = std::string(30, ' ');
    output = Utils::wordWrap2(s, 10, 12);
    EXPECT_EQ(output.size(), 3u);
    EXPECT_EQ(output[0], std::string(12, ' '));
    EXPECT_EQ(output[1], std::string(10, ' '));
    EXPECT_EQ(output[2], std::string(8, ' '));
}

TEST(UtilsTest, simpleWordexpTest)
{
    std::string s;
    std::vector<std::string> output;

    s = "fo\"o\\n= \"b\\\"   ar\" \"b";
    output = Utils::simpleWordexp(s);
    EXPECT_EQ(output.size(), 2u);
    EXPECT_EQ(output[0], "foo\\n= b\"");
    EXPECT_EQ(output[1], "ar b");

    s = "\"\\ \\ \"";
    output = Utils::simpleWordexp(s);
    EXPECT_EQ(output.size(), 1u);
    EXPECT_EQ(output[0], "\\ \\ ");

    s = "\\g";
    output = Utils::simpleWordexp(s);
    EXPECT_EQ(output.size(), 1u);
    EXPECT_EQ(output[0], "g");

    s = "foo= \"b\\\" ar\"";
    output = Utils::simpleWordexp(s);
    EXPECT_EQ(output.size(), 2u);
    EXPECT_EQ(output[0], "foo=");
    EXPECT_EQ(output[1], "b\" ar");

    s = "fo\"o= \"b\\\"   ar\"\"";
    output = Utils::simpleWordexp(s);
    EXPECT_EQ(output.size(), 2u);
    EXPECT_EQ(output[0], "foo= b\"");
    EXPECT_EQ(output[1], "ar");

    s = "a b   c   def \"ghi jkl\"";
    output = Utils::simpleWordexp(s);
    EXPECT_EQ(output.size(), 5u);
    EXPECT_EQ(output[0], "a");
    EXPECT_EQ(output[1], "b");
    EXPECT_EQ(output[2], "c");
    EXPECT_EQ(output[3], "def");
    EXPECT_EQ(output[4], "ghi jkl");
}

TEST(UtilsTest, naninf)
{
    double d = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(Utils::toString(d), "NaN");
    d = std::numeric_limits<double>::infinity();
    EXPECT_EQ(Utils::toString(d), "Infinity");
    d = -d;
    EXPECT_EQ(Utils::toString(d), "-Infinity");
}


TEST(UtilsTest, numeric_cast)
{
    bool ok;
    double d;
    float f;

    f = std::numeric_limits<float>::quiet_NaN();
    ok = Utils::numericCast(f, d);
    EXPECT_TRUE(ok);
    EXPECT_TRUE(std::isnan(d));

    d = std::numeric_limits<double>::quiet_NaN();
    ok = Utils::numericCast(d, f);
    EXPECT_TRUE(ok);
    EXPECT_TRUE(std::isnan(f));

    d = (std::numeric_limits<float>::max)() * 2;
    EXPECT_FALSE(Utils::numericCast(d, f));

    d = (std::numeric_limits<float>::max)() / 2;
    EXPECT_TRUE(Utils::numericCast(d, f));
}

TEST(UtilsTest, escapeJSON)
{
    std::string escaped = Utils::escapeJSON("\u0001\t\f\n\\\"\u0016");
    EXPECT_EQ(escaped, "\\u0001\\t\\f\\n\\\\\\\"\\u0016");
}

TEST(UtilsTest, map)
{
    Support::Tempfile temp;

    std::string filename = temp.filename();

    std::ostream *out;
    // This turns on sparse file support. Otherwise, we're going to make a huge
    // file that won't fit on many filesystems and an error will occur. If we
    // can't set the file to sparse, we just return.  UNIX filesystems I'm
    // aware of support sparse files without this mess.
#ifdef _WIN32
    auto f = CreateFileA(filename.data(), GENERIC_READ | GENERIC_WRITE,
        0, NULL, CREATE_NEW, FILE_ATTRIBUTE_NORMAL, NULL);
    DWORD flags;
    GetVolumeInformationByHandleW(f, NULL, 0, NULL, NULL, &flags, NULL, 0);
    bool ok = false;
    if (flags & FILE_SUPPORTS_SPARSE_FILES)
    {
        DWORD tmp;
        ok = DeviceIoControl(f, FSCTL_SET_SPARSE, NULL, 0, NULL, 0, &tmp, NULL);
    }
    CloseHandle(f);
    if (!ok)
        return;
    out = FileUtils::openExisting(filename);
#else
    out = FileUtils::createFile(filename);
#endif

    out->seekp(50000);
    *out << 1234;
    out->write("Test", 4);
    out->seekp(0x10FFFFFFFF);
    *out << 5678;
    out->write("Another.", 9);
    FileUtils::closeFile(out);

    auto ctx = FileUtils::mapFile(filename);
    assert(ctx.addr());
    char *c = reinterpret_cast<char *>(ctx.addr()) + 50000;

    EXPECT_EQ(*c++, '1');
    EXPECT_EQ(*c++, '2');
    EXPECT_EQ(*c++, '3');
    EXPECT_EQ(*c++, '4');
    EXPECT_EQ(*c++, 'T');
    EXPECT_EQ(*c++, 'e');
    EXPECT_EQ(*c++, 's');
    EXPECT_EQ(*c++, 't');

    c = reinterpret_cast<char *>(ctx.addr()) + 0x10FFFFFFFF;
    EXPECT_EQ(*c++, '5');
    EXPECT_EQ(*c++, '6');
    EXPECT_EQ(*c++, '7');
    EXPECT_EQ(*c++, '8');
    EXPECT_EQ(std::string(c), "Another.");
    FileUtils::unmapFile(ctx);
}

}
