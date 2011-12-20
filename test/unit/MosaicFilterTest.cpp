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

#include <pdal/StageIterator.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/filters/Mosaic.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(MosaicFilterTest)

BOOST_AUTO_TEST_CASE(test1)
{
#ifndef PDAL_SRS_ENABLED
    // because mosaicking needs to compare SRSs
    return;
#endif

    Bounds<double> bounds1(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);
    pdal::drivers::faux::Reader reader1(bounds1, 100, pdal::drivers::faux::Reader::Constant);

    Bounds<double> bounds2(100.0, 100.0, 100.0, 200.0, 200.0, 200.0);
    pdal::drivers::faux::Reader reader2(bounds2, 100, pdal::drivers::faux::Reader::Constant);

    Bounds<double> bounds3(200.0, 200.0, 200.0, 300.0, 300.0, 300.0);
    pdal::drivers::faux::Reader reader3(bounds3, 100, pdal::drivers::faux::Reader::Constant);

    std::vector<Stage*> vec;
    vec.push_back(&reader1);
    vec.push_back(&reader2);
    vec.push_back(&reader3);

    pdal::filters::Mosaic mosaic(vec, Options::none());
    BOOST_CHECK(mosaic.getDescription() == "Mosaic Filter");
    mosaic.initialize();

    const Schema& schema = mosaic.getSchema();

    PointBuffer data(schema, 300);

    StageSequentialIterator* iter = mosaic.createSequentialIterator();
    boost::uint32_t numRead = iter->read(data);

    BOOST_CHECK(numRead == 300);


    Schema const& buffer_schema = data.getSchema();
    Dimension const& dimX = buffer_schema.getDimension("X");
    Dimension const& dimY = buffer_schema.getDimension("Y");
    Dimension const& dimZ = buffer_schema.getDimension("Z");
    Dimension const& dimTime = buffer_schema.getDimension("Time");

    for (boost::uint32_t i=0; i<300; i++)
    {
        double x = data.getField<double>(dimX, i);
        double y = data.getField<double>(dimY, i);
        double z = data.getField<double>(dimZ, i);
        boost::uint64_t t = data.getField<boost::uint64_t>(dimTime, i);

        if (i<100)
        {
            BOOST_CHECK(Utils::compare_approx(x, 0.0, (std::numeric_limits<double>::min)()) == true);
            BOOST_CHECK(Utils::compare_approx(y, 0.0, (std::numeric_limits<double>::min)()) == true);
            BOOST_CHECK(Utils::compare_approx(z, 0.0, (std::numeric_limits<double>::min)()) == true);
        }
        else if (i<200)
        {
            BOOST_CHECK(Utils::compare_approx(x, 100.0, (std::numeric_limits<double>::min)()) == true);
            BOOST_CHECK(Utils::compare_approx(y, 100.0, (std::numeric_limits<double>::min)()) == true);
            BOOST_CHECK(Utils::compare_approx(z, 100.0, (std::numeric_limits<double>::min)()) == true);
        }
        else
        {
            BOOST_CHECK(Utils::compare_approx(x, 200.0, (std::numeric_limits<double>::min)()) == true);
            BOOST_CHECK(Utils::compare_approx(y, 200.0, (std::numeric_limits<double>::min)()) == true);
            BOOST_CHECK(Utils::compare_approx(z, 200.0, (std::numeric_limits<double>::min)()) == true);
        }
        BOOST_CHECK(t == i % 100);
    }
    
    delete iter;

    return;
}


BOOST_AUTO_TEST_SUITE_END()
