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
#include <boost/cstdint.hpp>

#include <pdal/pdal_internal.hpp>
#include <pdal/pdal_config.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(ConfigTest)

BOOST_AUTO_TEST_CASE(test_3rdparty_libs)
{
    bool geotiff = IsLibGeoTIFFEnabled();
    bool laszip = IsLasZipEnabled();

#ifdef PDAL_HAVE_LIBGEOTIFF
    BOOST_CHECK(geotiff);
#else
    BOOST_CHECK(!geotiff);
#endif

#ifdef PDAL_HAVE_LASZIP
    BOOST_CHECK(laszip);
#else
    BOOST_CHECK(!laszip);
#endif
}

BOOST_AUTO_TEST_CASE(test_version)
{
    // just verify these functions can be called, don't worry about the values

    std::string version = GetVersionString();
    BOOST_CHECK(!version.empty());
    std::string fullVersion = GetFullVersionString();
    BOOST_CHECK(!fullVersion.empty());

    int major = GetVersionMajor();
    BOOST_CHECK(major >= 0);
    int minor = GetVersionMinor();
    BOOST_CHECK(minor >= 0);
    int patch = GetVersionPatch();
    BOOST_CHECK(patch >= 0);

    int bignum = GetVersionInteger();
    BOOST_CHECK(bignum > 0);
}

BOOST_AUTO_TEST_SUITE_END()
