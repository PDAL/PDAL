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

#include <pdal/pdal_features.hpp>
#include <pdal/pdal_config.hpp>

using namespace pdal;
using namespace pdal::Config;

TEST(ConfigTest, test_3rdparty_libs)
{
    bool laszip = hasFeature(Feature::LASZIP);

#ifdef PDAL_HAVE_LASZIP
    EXPECT_TRUE(laszip);
#else
    EXPECT_TRUE(!laszip);
#endif
}

TEST(ConfigTest, test_version)
{
    // just verify these functions can be called, don't worry about the values

    std::string version = versionString();
    EXPECT_TRUE(!version.empty());
    std::string fullVersion = fullVersionString();
    EXPECT_TRUE(!fullVersion.empty());

    int major = versionMajor();
    EXPECT_TRUE(major >= 0);
    int minor = versionMinor();
    EXPECT_TRUE(minor >= 0);
    int patch = versionPatch();
    EXPECT_TRUE(patch >= 0);

    int bignum = versionInteger();
    EXPECT_TRUE(bignum > 0);
}
