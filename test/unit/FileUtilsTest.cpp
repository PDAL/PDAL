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

#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(FileUtilsTest, test_file_ops)
{
    std::string tmp1(Support::temppath("unittest1.tmp"));
    std::string tmp2(Support::temppath("unittest2.tmp"));

    // first, clean up from any previous test run
    FileUtils::deleteFile(tmp1);
    FileUtils::deleteFile(tmp2);
    EXPECT_TRUE(FileUtils::fileExists(tmp1)==false);
    EXPECT_TRUE(FileUtils::fileExists(tmp2)==false);

    // write test
    std::ostream* ostr = FileUtils::createFile(tmp1);
    *ostr << "yow";
    FileUtils::closeFile(ostr);

    EXPECT_EQ(FileUtils::fileExists(tmp1), true);
    EXPECT_EQ(FileUtils::fileSize(tmp1), 3U);

    // rename test
    FileUtils::renameFile(tmp2,tmp1);
    EXPECT_TRUE(FileUtils::fileExists(tmp1)==false);
    EXPECT_TRUE(FileUtils::fileExists(tmp2)==true);

    // read test
    std::istream* istr = FileUtils::openFile(tmp2);
    std::string yow;
    *istr >> yow;
    FileUtils::closeFile(istr);
    EXPECT_TRUE(yow=="yow");

    // delete test
    FileUtils::deleteFile(tmp2);
    EXPECT_TRUE(FileUtils::fileExists(tmp2)==false);
}

TEST(FileUtilsTest, test_readFileIntoString)
{
    const std::string filename = Support::datapath("text/text.txt");
    EXPECT_TRUE(FileUtils::fileExists(filename));

    std::string source = FileUtils::readFileIntoString(filename);

    std::string ref = "This is a file that allows us to test that we "
        "can read a text file into a string through the file utils.\n";

    EXPECT_TRUE(source == ref);
}

TEST(FileUtilsTest, test_getcwd)
{
#if 0
    // this is hardcoded for mpg's environment
    const std::string cwd = FileUtils::getcwd();
    EXPECT_TRUE(cwd == "D:/dev/pdal/test/unit/");
#endif
}

#ifdef _WIN32
static const std::string drive = "A:";
#else
static const std::string drive = "";
#endif

static std::string normalize(const std::string p)
{
    return Utils::replaceAll(p, "\\", "/");
}

static void compare_paths(const std::string a, const std::string b)
{
    EXPECT_EQ(normalize(a), normalize(b));
}

TEST(FileUtilsTest, test_toAbsolutePath)
{
    using namespace std;

    const string root = FileUtils::getcwd();

    // check 1-arg version: make absolute when file is relative, via current working dir
    const string a = FileUtils::toAbsolutePath("foo.txt");
    compare_paths(a, root + "foo.txt");

    // check 1-arg version: make absolute when file is already absolute
    const string b = FileUtils::toAbsolutePath(drive + "/baz/foo.txt");
    compare_paths(b, drive + "/baz/foo.txt");

    // check 2-arg version: make absolute when file relative, via given base
    const string c = FileUtils::toAbsolutePath("foo.txt", drive + "/a/b/c/d");
    compare_paths(c, drive + "/a/b/c/d/foo.txt");

    // check 2-arg version: make absolute when file is relative, via given base (which isn't absolute)
    const string d = FileUtils::toAbsolutePath("foo.txt", "x/y/z");
    compare_paths(d, root + "x/y/z/" + "foo.txt");

    // check 1-arg version: make absolute when file is already absolute
    const string e = FileUtils::toAbsolutePath(drive+"/baz/foo.txt", drive+"/a/b/c/d");
    compare_paths(e, drive + "/baz/foo.txt");
}

TEST(FileUtilsTest, test_getDirectory)
{
    // test absolute case
    const std::string a = FileUtils::getDirectory(drive + "/a/b/foo.txt");
    compare_paths(a, drive + "/a/b/");

    // test relative case
    const std::string b = FileUtils::getDirectory("a/b/foo.txt");
    compare_paths(b, "a/b/");
}

TEST(FileUtilsTest, test_isAbsolute)
{
    // test absolute case
    const bool a = FileUtils::isAbsolutePath(drive + "/a/b/foo.txt");
    EXPECT_TRUE(a);

    // test relative case
    const bool b = FileUtils::isAbsolutePath("a/b/foo.txt");
    EXPECT_TRUE(!b);
}

TEST(FileUtilsTest, filename)
{
    std::string filename = "/foo//bar//baz.c";
    EXPECT_EQ(FileUtils::getFilename(filename), "baz.c");
}
