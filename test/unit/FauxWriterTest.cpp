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

#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/faux/Writer.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(FauxWriterTest)

BOOST_AUTO_TEST_CASE(test_1)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Constant);

    pdal::drivers::faux::Writer writer(reader, Options::none());
    BOOST_CHECK(writer.getDescription() == "Faux Writer");

    boost::uint64_t numWritten = writer.write(750);

    BOOST_CHECK(numWritten == 750);

    BOOST_CHECK(Utils::compare_approx(writer.getMinX(), 1.0, (std::numeric_limits<double>::min)()) == true);
    BOOST_CHECK(Utils::compare_approx(writer.getMinY(), 2.0, (std::numeric_limits<double>::min)()) == true);
    BOOST_CHECK(Utils::compare_approx(writer.getMinZ(), 3.0, (std::numeric_limits<double>::min)()) == true);

    BOOST_CHECK(Utils::compare_approx(writer.getMaxX(), 1.0, (std::numeric_limits<double>::min)()) == true);
    BOOST_CHECK(Utils::compare_approx(writer.getMaxY(), 2.0, (std::numeric_limits<double>::min)()) == true);
    BOOST_CHECK(Utils::compare_approx(writer.getMaxZ(), 3.0, (std::numeric_limits<double>::min)()) == true);

    BOOST_CHECK(Utils::compare_approx(writer.getAvgX(), 1.0, (std::numeric_limits<double>::min)()) == true);
    BOOST_CHECK(Utils::compare_approx(writer.getAvgY(), 2.0, (std::numeric_limits<double>::min)()) == true);
    BOOST_CHECK(Utils::compare_approx(writer.getAvgZ(), 3.0, (std::numeric_limits<double>::min)()) == true);


    return;
}

BOOST_AUTO_TEST_CASE(test_2)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Random);

    pdal::drivers::faux::Writer writer(reader, Options::none());

    boost::uint64_t numWritten = writer.write(750);

    BOOST_CHECK(numWritten == 750);

    // test all the values to +/- 10%
    BOOST_CHECK(Utils::compare_approx<double>(writer.getMinX(), 1.0, 10.0));
    BOOST_CHECK(Utils::compare_approx<double>(writer.getMinY(), 2.0, 10.0));
    BOOST_CHECK(Utils::compare_approx<double>(writer.getMinZ(), 3.0, 10.0));
    BOOST_CHECK(Utils::compare_approx<double>(writer.getMaxX(), 101.0, 10.0));
    BOOST_CHECK(Utils::compare_approx<double>(writer.getMaxY(), 102.0, 10.0));
    BOOST_CHECK(Utils::compare_approx<double>(writer.getMaxZ(), 103.0, 10.0));
    BOOST_CHECK(Utils::compare_approx<double>(writer.getAvgX(), 51.0, 10.0));
    BOOST_CHECK(Utils::compare_approx<double>(writer.getAvgY(), 52.0, 10.0));
    BOOST_CHECK(Utils::compare_approx<double>(writer.getAvgZ(), 53.0, 10.0));

    return;
}

BOOST_AUTO_TEST_SUITE_END()
