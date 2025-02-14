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

#include <pdal/util/VSIIO.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/util/FileUtils.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(VSITest, test_numeric)
{
    std::string tmp1(Support::temppath("unittest1.tmp"));

    // first, clean up from any previous test run
    FileUtils::deleteFile(tmp1);
    EXPECT_TRUE(FileUtils::fileExists(tmp1)==false);

    // write test
    VSI::VSIStream* pOstr = new VSI::VSIStream(
        tmp1, std::ios::out | std::ios::binary);

    *pOstr << (uint32_t) 10000;
    pOstr->flush();
    delete pOstr;

    EXPECT_EQ(FileUtils::fileExists(tmp1), true);
    EXPECT_EQ(FileUtils::fileSize(tmp1), 4U);

    // read test
    VSI::VSIStream* pIstr = new VSI::VSIStream(
        tmp1, std::ios::in | std::ios::binary);
    uint32_t r;
    *pIstr >> r;
    EXPECT_EQ(r, 10000);
}

TEST(VSIFileUtilsTest, test_file_vsi_numeric)
{
    // VSI write test
    std::string vsiFile("/vsimem/vsi.dat");
    std::ostream* oStrVsi = FileUtils::createFile(vsiFile);
    EXPECT_TRUE(FileUtils::fileExists(vsiFile));
    *oStrVsi << (uint32_t)10000;
    FileUtils::closeFile(oStrVsi);
    EXPECT_EQ(FileUtils::fileExists(vsiFile), true);
    EXPECT_EQ(FileUtils::fileSize(vsiFile), 4U);

    // VSI read test
    std::istream* iStrVsi = FileUtils::openFile(vsiFile);
    uint32_t rVsi;
    *iStrVsi >> rVsi;
    FileUtils::closeFile(iStrVsi);
    EXPECT_TRUE(rVsi==10000);
}
