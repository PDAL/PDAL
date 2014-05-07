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

#include <pdal/UserCallback.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/faux/Writer.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(FauxWriterTest)

BOOST_AUTO_TEST_CASE(FauxWriterTest_test_1)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Constant);

    pdal::drivers::faux::Writer writer;
    writer.setInput(&reader);
    BOOST_CHECK(writer.getDescription() == "Faux Writer");
    writer.prepare();

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
}

BOOST_AUTO_TEST_CASE(test_2)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Random);

    pdal::drivers::faux::Writer writer;
    writer.setInput(&reader);
    writer.prepare();

    boost::uint64_t numWritten = writer.write(750);

    BOOST_CHECK_EQUAL(numWritten, 750);

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
}


class MyUserCallback_0 : public UserCallback
{
public:
    virtual void callback()
    {
        // just run to completion
        return;
    }
};

class MyUserCallback_1 : public UserCallback
{
public:
    virtual void callback()
    {
        if (getPercentComplete() > 50.0)
        {
            setInterruptFlag(true);
        }
    }
};

class MyUserCallback_2 : public UserCallback
{
public:
    virtual void callback()
    {
        // whichever comes first
        if (getHeartbeats() > 6)
        {
            setInterruptFlag(true);
        }
    }
};


BOOST_AUTO_TEST_CASE(test_callbacks)
{
    // write(0) and run to end without interrupts
    {
        Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
        pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Random);
        pdal::drivers::faux::Writer writer;
        writer.setInput(&reader);
        writer.prepare();

        MyUserCallback_0 cb;
        writer.setUserCallback(&cb);

        writer.write(0, 0, 100);

        BOOST_CHECK_EQUAL(cb.getHeartbeats(), 12u);
        BOOST_CHECK_EQUAL(cb.getPercentComplete(), 100);
    }

    // write(1000) and run to end without interrupts
    {
        Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
        pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Random);
        pdal::drivers::faux::Writer writer;
        writer.setInput(&reader);
        writer.prepare();

        MyUserCallback_0 cb;
        writer.setUserCallback(&cb);

        writer.write(1000, 0, 100);

        BOOST_CHECK_EQUAL(cb.getHeartbeats(), 12u);
        BOOST_CHECK_EQUAL(cb.getPercentComplete(), 100);
    }

    // write(1000) and get interrupted by perc completed
    {
        Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
        pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Random);
        pdal::drivers::faux::Writer writer;
        writer.setInput(&reader);
        writer.prepare();

        MyUserCallback_1 cb;
        writer.setUserCallback(&cb);

        BOOST_CHECK_THROW(writer.write(1000, 0, 100), pipeline_interrupt);

        BOOST_CHECK_EQUAL(cb.getHeartbeats(), 7u);
        BOOST_CHECK_EQUAL(cb.getPercentComplete(), 60);
    }

    // write(0) and get interrupted by heartbeats
    {
        Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
        pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Random);
        pdal::drivers::faux::Writer writer;
        writer.setInput(&reader);
        writer.prepare();

        MyUserCallback_2 cb;
        writer.setUserCallback(&cb);

        BOOST_CHECK_THROW(writer.write(1000, 0, 100), pipeline_interrupt);

        BOOST_CHECK_EQUAL(cb.getHeartbeats(), 7u);
        BOOST_CHECK_EQUAL(cb.getPercentComplete(), 60);
    }
}

BOOST_AUTO_TEST_CASE(test_buffer_resize)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Random);
    pdal::drivers::faux::Writer writer;
    writer.setInput(&reader);
    writer.prepare();

    BOOST_CHECK_EQUAL(writer.write(750, 0, 101), 750);
}

BOOST_AUTO_TEST_SUITE_END()
