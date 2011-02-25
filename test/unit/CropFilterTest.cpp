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

#include "libpc/FauxReader.hpp"
#include "libpc/FauxWriter.hpp"
#include "libpc/CropFilter.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(CropFilterTest)

BOOST_AUTO_TEST_CASE(test_crop)
{
    Bounds<double> srcBounds(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);

    // crop tjhe window to 1/8th the size
    Bounds<double> dstBounds(0.0, 0.0, 0.0, 50.0, 50.0, 50.0);
    
    FauxReader reader(srcBounds, 1000, FauxReader::Random);

    CropFilter filter(reader, dstBounds);

    FauxWriter writer(filter);

    boost::uint64_t numWritten = writer.write(1000);

    // 1000 * 1/8 = 125, plus or minus 10%
    BOOST_CHECK(Utils::compare_approx<double>(static_cast<double>(numWritten), 125, 12.5));

    // test all the values to +/- 10%
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMinX(), 0.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMinY(), 0.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMinZ(), 0.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMaxX(), 50.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMaxY(), 50.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMaxZ(), 50.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getAvgX(), 25.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getAvgY(), 25.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getAvgZ(), 25.0, 5.0));

    return;
}

BOOST_AUTO_TEST_SUITE_END()
