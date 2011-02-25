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

#include "libpc/LiblasReader.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(LiblasReaderTest)


#define Compare(x,y)    BOOST_CHECK(Utils::compare_approx((x),(y),0.001));
    

BOOST_AUTO_TEST_CASE(test_1)
{
    std::istream* ifs = Utils::openFile("../../test/data/1.2-with-color.las");
    
    LiblasReader reader(*ifs);

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointData data(layout, 3);
    std::size_t offsetX = schema.getDimensionIndex(Dimension::Field_X);
    std::size_t offsetY = schema.getDimensionIndex(Dimension::Field_Y);
    std::size_t offsetZ = schema.getDimensionIndex(Dimension::Field_Z);
    
    {
        boost::uint32_t numRead = reader.readPoints(data);
        BOOST_CHECK(numRead == 3);

        double x0 = data.getField<double>(0, offsetX);
        double y0 = data.getField<double>(0, offsetY);
        double z0 = data.getField<double>(0, offsetZ);
        double x1 = data.getField<double>(1, offsetX);
        double y1 = data.getField<double>(1, offsetY);
        double z1 = data.getField<double>(1, offsetZ);
        double x2 = data.getField<double>(2, offsetX);
        double y2 = data.getField<double>(2, offsetY);
        double z2 = data.getField<double>(2, offsetZ);

        Compare(x0, 637012.240000);
        Compare(y0, 849028.310000);
        Compare(z0, 431.660000);

        Compare(x1, 636896.330000);
        Compare(y1, 849087.700000);
        Compare(z1, 446.390000);

        Compare(x2, 636784.740000);
        Compare(y2, 849106.660000);
        Compare(z2, 426.710000);
    }

    // Can we seek it? Yes, we can!
    reader.seekToPoint(100);
    {
        boost::uint32_t numRead = reader.readPoints(data);
        BOOST_CHECK(numRead == 3);

        double x0 = data.getField<double>(0, offsetX);
        double y0 = data.getField<double>(0, offsetY);
        double z0 = data.getField<double>(0, offsetZ);
        double x1 = data.getField<double>(1, offsetX);
        double y1 = data.getField<double>(1, offsetY);
        double z1 = data.getField<double>(1, offsetZ);
        double x2 = data.getField<double>(2, offsetX);
        double y2 = data.getField<double>(2, offsetY);
        double z2 = data.getField<double>(2, offsetZ);

        Compare(x0, 636661.060000);
        Compare(y0, 849854.130000);
        Compare(z0, 424.900000);

        Compare(x1, 636568.180000);
        Compare(y1, 850179.490000);
        Compare(z1, 441.800000);

        Compare(x2, 636554.630000);
        Compare(y2, 850040.030000);
        Compare(z2, 499.110000);
    }

    // Can we reset it? Yes, we can!
    reader.reset();
    {
        boost::uint32_t numRead = reader.readPoints(data);
        BOOST_CHECK(numRead == 3);

        double x0 = data.getField<double>(0, offsetX);
        double y0 = data.getField<double>(0, offsetY);
        double z0 = data.getField<double>(0, offsetZ);
        double x1 = data.getField<double>(1, offsetX);
        double y1 = data.getField<double>(1, offsetY);
        double z1 = data.getField<double>(1, offsetZ);
        double x2 = data.getField<double>(2, offsetX);
        double y2 = data.getField<double>(2, offsetY);
        double z2 = data.getField<double>(2, offsetZ);

        Compare(x0, 637012.240000);
        Compare(y0, 849028.310000);
        Compare(z0, 431.660000);

        Compare(x1, 636896.330000);
        Compare(y1, 849087.700000);
        Compare(z1, 446.390000);

        Compare(x2, 636784.740000);
        Compare(y2, 849106.660000);
        Compare(z2, 426.710000);
    }
    
    Utils::closeFile(ifs);

    return;
}


BOOST_AUTO_TEST_SUITE_END()
