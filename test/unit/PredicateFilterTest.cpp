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

#include <pdal/filters/Predicate.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/faux/Writer.hpp>

BOOST_AUTO_TEST_SUITE(PredicateFilterTest)

using namespace pdal;

BOOST_AUTO_TEST_CASE(PredicateFilterTest_test1)
{
return;
    Bounds<double> bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Ramp);

    const pdal::Option opt("expression", "a");
    pdal::Options opts;
    opts.add(opt);

    pdal::filters::Predicate filter(reader, opts);
    BOOST_CHECK(filter.getDescription() == "Predicate Filter");
    pdal::drivers::faux::Writer writer(filter, Options::none());
    writer.initialize();

    boost::uint64_t numWritten = writer.write(1000);

    BOOST_CHECK(numWritten == 500);

    const double minX = writer.getMinX();
    const double minY = writer.getMinY();
    const double minZ = writer.getMinZ();
    const double maxX = writer.getMaxX();
    const double maxY = writer.getMaxY();
    const double maxZ = writer.getMaxZ();

    BOOST_CHECK(Utils::compare_approx<double>(minX, 0.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(minY, 0.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(minZ, 0.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(maxX, 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(maxY, 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(maxZ, 1.0, 0.01));

    return;
}

BOOST_AUTO_TEST_SUITE_END()
