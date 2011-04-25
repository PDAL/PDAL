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

#include <libpc/drivers/faux/Reader.hpp>
#include <libpc/drivers/faux/Writer.hpp>
#include <libpc/filters/CropFilter.hpp>

using namespace libpc;

BOOST_AUTO_TEST_SUITE(CropFilterTest)

BOOST_AUTO_TEST_CASE(test_crop)
{
    Bounds<double> srcBounds(0.0, 0.0, 0.0, 10.0, 100.0, 1000.0);

    // crop the window to 1/3rd the size in each dimension
    Bounds<double> dstBounds(3.33333, 33.33333, 333.33333, 6.66666, 66.66666, 666.66666);
    
    libpc::drivers::faux::Reader reader(srcBounds, 1000, libpc::drivers::faux::Reader::Ramp);

    libpc::filters::CropFilter filter(reader, dstBounds);
    BOOST_CHECK(filter.getDescription() == "Crop Filter");

    libpc::drivers::faux::Writer writer(filter);

    boost::uint64_t numWritten = writer.write(1000);

    // 1000 * 1/3 = 333, plus or minus a bit for rounding
    BOOST_CHECK(Utils::compare_approx<double>(static_cast<double>(numWritten), 333, 6));

    const double minX = writer.getMinX();
    const double minY = writer.getMinY();
    const double minZ = writer.getMinZ();
    const double maxX = writer.getMaxX();
    const double maxY = writer.getMaxY();
    const double maxZ = writer.getMaxZ();
    const double avgX = writer.getAvgX();
    const double avgY = writer.getAvgY();
    const double avgZ = writer.getAvgZ();

    const double delX = 10.0 / 999.0;
    const double delY = 100.0 / 999.0;
    const double delZ = 1000.0 / 999.0;

    BOOST_CHECK(Utils::compare_approx<double>(minX, 3.33333, delX));
    BOOST_CHECK(Utils::compare_approx<double>(minY, 33.33333, delY));
    BOOST_CHECK(Utils::compare_approx<double>(minZ, 333.33333, delZ));
    BOOST_CHECK(Utils::compare_approx<double>(maxX, 6.66666, delX));
    BOOST_CHECK(Utils::compare_approx<double>(maxY, 66.66666, delY));
    BOOST_CHECK(Utils::compare_approx<double>(maxZ, 666.66666, delZ));
    BOOST_CHECK(Utils::compare_approx<double>(avgX, 5.00000, delX));
    BOOST_CHECK(Utils::compare_approx<double>(avgY, 50.00000, delY));
    BOOST_CHECK(Utils::compare_approx<double>(avgZ, 500.00000, delZ));

    return;
}

BOOST_AUTO_TEST_SUITE_END()
