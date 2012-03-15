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

#include <pdal/Range.hpp>

#include <vector>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(UtilsTest)

BOOST_AUTO_TEST_CASE(test_random)
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
        BOOST_CHECK(x >= rangeMin);
        BOOST_CHECK(x <= rangeMax);
     
        sum += x;
    }

    sum = sum / iters;
    
    BOOST_CHECK(sum <= avg + 0.1*avg);
    BOOST_CHECK(sum >= avg - 0.1*avg);
}


BOOST_AUTO_TEST_CASE(test_comparators)
{
    bool ok;
    
    {
        ok = Utils::compare_distance<float>(1.000001f, 1.0f);
        BOOST_CHECK(!ok);

        ok = Utils::compare_distance<float>(1.0000001f, 1.0f);
        BOOST_CHECK(ok);

        ok = Utils::compare_distance<float>(1.00000001f, 1.0f);
        BOOST_CHECK(ok);
    }

    {
        ok = Utils::compare_approx<float>(1.001f, 1.0f, 0.0001f);
        BOOST_CHECK(!ok);

        ok = Utils::compare_approx<float>(1.001f, 1.0f, 0.001f);
        BOOST_CHECK(!ok);

        ok = Utils::compare_approx<float>(1.001f, 1.0f, 0.01f);
        BOOST_CHECK(ok);

        ok = Utils::compare_approx<float>(1.001f, 1.0f, 0.1f);
        BOOST_CHECK(ok);
    }

    {
        ok = Utils::compare_approx<unsigned int>(10, 12, 2);
        BOOST_CHECK(ok);

        ok = Utils::compare_approx<unsigned int>(10, 12, 3);
        BOOST_CHECK(ok);

        ok = Utils::compare_approx<unsigned int>(10, 12, 1);
        BOOST_CHECK(!ok);
    }
}


BOOST_AUTO_TEST_CASE(test_buffer_read_write)
{
    const char buf[] = "quick brown fox";
    std::string bufstr(buf);

    std::ostringstream ostr;
    Utils::write_n(ostr, buf, 15);

    BOOST_CHECK(memcmp(buf, ostr.str().c_str(), 15) == 0);

    std::istringstream istr;
    istr.str(bufstr);
    boost::uint8_t tmp[30];
    Utils::read_n(tmp, istr, 15);
    BOOST_CHECK(memcmp(buf, tmp, 15) == 0);

    return;
}


BOOST_AUTO_TEST_CASE(test_field_read_write)
{
    boost::uint8_t buffer[100];
    boost::uint8_t* p = buffer;

    boost::uint8_t one = 1;
    double two = 2.0;

    Utils::write_field<boost::uint8_t>(p, one);
    Utils::write_field<double>(p, two);

    p = buffer;

    boost::uint8_t x = Utils::read_field<boost::uint8_t>(p);
    double y = Utils::read_field<double>(p);

    BOOST_CHECK(x==one);
    BOOST_CHECK(Utils::compare_approx(y, two, std::numeric_limits<double>::min()) == true);
    
    return;
}


BOOST_AUTO_TEST_CASE(test_base64)
{
    std::vector<boost::uint8_t> data;
    for(int i=0;i<100;i++) data.push_back(i);
    
    std::string encoded = Utils::base64_encode(data);
    std::vector<boost::uint8_t> decoded = Utils::base64_decode(encoded);
    
    boost::uint32_t size(0);
    for (std::vector<boost::uint8_t>::size_type i = 0; i < decoded.size(); ++i)
    {
        size = size + decoded[i];
    }
    
    BOOST_CHECK_EQUAL(size, 4950u);
    
    
    return;
}



BOOST_AUTO_TEST_SUITE_END()
