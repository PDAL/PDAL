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
#include "libpc/MosaicFilter.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(MosaicFilterTest)

BOOST_AUTO_TEST_CASE(test1)
{
    Bounds<double> bounds1(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);
    FauxReader reader1(bounds1, 100, FauxReader::Constant);

    Bounds<double> bounds2(100.0, 100.0, 100.0, 200.0, 200.0, 200.0);
    FauxReader reader2(bounds2, 100, FauxReader::Constant);

    Bounds<double> bounds3(200.0, 200.0, 200.0, 300.0, 300.0, 300.0);
    FauxReader reader3(bounds3, 100, FauxReader::Constant);

    std::vector<Stage*> vec;
    vec.push_back(&reader2);
    vec.push_back(&reader3);

    MosaicFilter mosaic(reader1, vec);
    BOOST_CHECK(mosaic.getName() == "Mosaic Filter");

    const Schema& schema = mosaic.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointData data(layout, 300);
    
    boost::uint32_t numRead = mosaic.read(data);

    BOOST_CHECK(numRead == 300);

    const int offsetT = schema.getDimensionIndex(Dimension::Field_GpsTime);
    const int offsetX = schema.getDimensionIndex(Dimension::Field_X);
    const int offsetY = schema.getDimensionIndex(Dimension::Field_Y);
    const int offsetZ = schema.getDimensionIndex(Dimension::Field_Z);

    for (boost::uint32_t i=0; i<300; i++)
    {
        float x = data.getField<float>(i, offsetX);
        float y = data.getField<float>(i, offsetY);
        float z = data.getField<float>(i, offsetZ);
        boost::uint64_t t = data.getField<boost::uint64_t>(i, offsetT);

        if (i<100)
        {
            BOOST_CHECK(x == 0.0);
            BOOST_CHECK(y == 0.0);
            BOOST_CHECK(z == 0.0);
        }
        else if (i<200)
        {
            BOOST_CHECK(x == 100.0);
            BOOST_CHECK(y == 100.0);
            BOOST_CHECK(z == 100.0);
        }
        else
        {
            BOOST_CHECK(x == 200.0);
            BOOST_CHECK(y == 200.0);
            BOOST_CHECK(z == 200.0);
        }
        BOOST_CHECK(t == i % 100);
    }

    return;
}


BOOST_AUTO_TEST_SUITE_END()
