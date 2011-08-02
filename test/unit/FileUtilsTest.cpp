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

#include <sstream>

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include <pdal/FileUtils.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(FileUtilsTest)


BOOST_AUTO_TEST_CASE(test_file_ops)
{
    std::string tmp1 = "unittest1.tmp";
    std::string tmp2 = "unittest2.tmp";
    
    // first, clean up from any previous test run
    FileUtils::deleteFile(tmp1);
    FileUtils::deleteFile(tmp2);
    BOOST_CHECK(FileUtils::fileExists(tmp1)==false);
    BOOST_CHECK(FileUtils::fileExists(tmp2)==false);

    // write test
    std::ostream* ostr = FileUtils::createFile(tmp1);
    *ostr << "yow";
    FileUtils::closeFile(ostr);

    BOOST_CHECK(FileUtils::fileExists(tmp1)==true);
    BOOST_CHECK(FileUtils::fileSize(tmp1)==3);

    // rename test
    FileUtils::renameFile(tmp2,tmp1);
    BOOST_CHECK(FileUtils::fileExists(tmp1)==false);
    BOOST_CHECK(FileUtils::fileExists(tmp2)==true);

    // read test
    std::istream* istr = FileUtils::openFile(tmp2);
    std::string yow;
    *istr >> yow;
    FileUtils::closeFile(istr);
    BOOST_CHECK(yow=="yow");

    // delete test
    FileUtils::deleteFile(tmp2);
    BOOST_CHECK(FileUtils::fileExists(tmp2)==false);
}


BOOST_AUTO_TEST_SUITE_END()
