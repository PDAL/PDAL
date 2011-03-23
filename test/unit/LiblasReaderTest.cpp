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

#include <libpc/drivers/liblas/reader.hpp>
#include "support.hpp"

using namespace libpc;
using namespace libpc::drivers::liblas;


BOOST_AUTO_TEST_SUITE(LiblasReaderTest)


#define Compare(x,y)    BOOST_CHECK(Utils::compare_approx((x),(y),0.001));


static void checkPointXYZ(const PointBuffer& data, size_t index, const Schema& schema, 
                          double xref, double yref, double zref)
{
    int offsetX = schema.getDimensionIndex(Dimension::Field_X);
    int offsetY = schema.getDimensionIndex(Dimension::Field_Y);
    int offsetZ = schema.getDimensionIndex(Dimension::Field_Z);

    boost::int32_t x0raw = data.getField<boost::int32_t>(index, offsetX);
    boost::int32_t y0raw = data.getField<boost::int32_t>(index, offsetY);
    boost::int32_t z0raw = data.getField<boost::int32_t>(index, offsetZ);
    double x0 = schema.getDimension(offsetX).getNumericValue<boost::int32_t>(x0raw);
    double y0 = schema.getDimension(offsetY).getNumericValue<boost::int32_t>(y0raw);
    double z0 = schema.getDimension(offsetZ).getNumericValue<boost::int32_t>(z0raw);
    
    Compare(x0, xref);
    Compare(y0, yref);
    Compare(z0, zref);
}


BOOST_AUTO_TEST_CASE(test_1)
{
    LiblasReader reader(TestConfig::g_data_path + "1.2-with-color.las");
    BOOST_CHECK(reader.getName() == "Liblas Reader");

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointBuffer data(layout, 3);
    
    libpc::SequentialIterator* iter = reader.createSequentialIterator();

    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 3);

        checkPointXYZ(data, 0, schema, 637012.240000, 849028.310000, 431.660000);
        checkPointXYZ(data, 1, schema, 636896.330000, 849087.700000, 446.390000);
        checkPointXYZ(data, 2, schema, 636784.740000, 849106.660000, 426.710000);
    }

    // Can we seek it? Yes, we can!
    iter->skip(97);
    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 3);

        checkPointXYZ(data, 0, schema, 636661.060000, 849854.130000, 424.900000);
        checkPointXYZ(data, 1, schema, 636568.180000, 850179.490000, 441.800000);
        checkPointXYZ(data, 2, schema, 636554.630000, 850040.030000, 499.110000);
    }

    ////// Can we seek to beginning? Yes, we can!
    ////iter->seekToPoint(0);
    ////{
    ////    boost::uint32_t numRead = iter->read(data);
    ////    BOOST_CHECK(numRead == 3);

    ////    checkPointXYZ(data, 0, schema, 637012.240000, 849028.310000, 431.660000);
    ////    checkPointXYZ(data, 1, schema, 636896.330000, 849087.700000, 446.390000);
    ////    checkPointXYZ(data, 2, schema, 636784.740000, 849106.660000, 426.710000);
    ////}
    
    delete iter;

    return;
}


BOOST_AUTO_TEST_SUITE_END()
