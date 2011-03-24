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

#include <libpc/drivers/liblas/Reader.hpp>
#include <libpc/filters/CacheFilter.hpp>
#include "support.hpp"

using namespace libpc;
using namespace libpc::drivers::liblas;
using namespace libpc::filters;


BOOST_AUTO_TEST_SUITE(LiblasReaderTest)


#define Compare(x,y)    BOOST_CHECK(Utils::compare_approx((x),(y),0.001));


static void check_pN(const PointBuffer& data, const Schema& schema, 
                     size_t index, 
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

static void check_p0_p1_p2(const PointBuffer& data, const Schema& schema)
{
    check_pN(data, schema, 0, 637012.240000, 849028.310000, 431.660000);
    check_pN(data, schema, 1, 636896.330000, 849087.700000, 446.390000);
    check_pN(data, schema, 2, 636784.740000, 849106.660000, 426.710000);
}


static void check_p100_p101_p102(const PointBuffer& data, const Schema& schema)
{
    check_pN(data, schema, 0, 636661.060000, 849854.130000, 424.900000);
    check_pN(data, schema, 1, 636568.180000, 850179.490000, 441.800000);
    check_pN(data, schema, 2, 636554.630000, 850040.030000, 499.110000);
}


static void check_p355_p356_p357(const PointBuffer& data, const Schema& schema)
{
    check_pN(data, schema, 0, 636462.600000, 850566.110000, 432.610000);
    check_pN(data, schema, 1, 636356.140000, 850530.480000, 432.680000);
    check_pN(data, schema, 2, 636227.530000, 850592.060000, 428.670000);
}


static void check_p710_p711_p712(const PointBuffer& data, const Schema& schema)
{
    check_pN(data, schema, 0, 638720.670000, 850926.640000, 417.320000);
    check_pN(data, schema, 1, 638672.380000, 851081.660000, 420.670000);
    check_pN(data, schema, 2, 638598.880000, 851445.370000, 422.150000);
}


BOOST_AUTO_TEST_CASE(test_sequential)
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

        check_p0_p1_p2(data, schema);
    }

    // Can we seek it? Yes, we can!
    iter->skip(97);
    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 3);

        check_p100_p101_p102(data, schema);
    }

    delete iter;

    return;
}


BOOST_AUTO_TEST_CASE(test_random)
{
    LiblasReader reader(TestConfig::g_data_path + "1.2-with-color.las");
    BOOST_CHECK(reader.getName() == "Liblas Reader");

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointBuffer data(layout, 3);
    
    libpc::RandomIterator* iter = reader.createRandomIterator();

    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 3);

        check_p0_p1_p2(data, schema);
    }

    // Can we seek it? Yes, we can!
    iter->seek(100);
    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 3);

        check_p100_p101_p102(data, schema);
    }

    // Can we seek to beginning? Yes, we can!
    iter->seek(0);
    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 3);

        check_p0_p1_p2(data, schema);
    }
    
    delete iter;

    return;
}


BOOST_AUTO_TEST_CASE(test_two_iters)
{
    LiblasReader reader(TestConfig::g_data_path + "1.2-with-color.las");
    BOOST_CHECK(reader.getName() == "Liblas Reader");

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    BOOST_CHECK(reader.getNumPoints() == 1065);
    PointBuffer data(layout, 1065);

    {
        libpc::SequentialIterator* iter = reader.createSequentialIterator();
        BOOST_CHECK(iter->getIndex() == 0);

        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 1065);
        BOOST_CHECK(iter->getIndex() == 1065);

        check_p0_p1_p2(data, schema);

        delete iter;
    }

    {
        libpc::RandomIterator* iter = reader.createRandomIterator();
        BOOST_CHECK(iter->getIndex() == 0);

        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 1065);
        BOOST_CHECK(iter->getIndex() == 1065);
        
        check_p0_p1_p2(data, schema);

        delete iter;
    }

    return;
}


BOOST_AUTO_TEST_CASE(test_two_iters_with_cache)
{
    LiblasReader reader(TestConfig::g_data_path + "1.2-with-color.las");
    BOOST_CHECK(reader.getName() == "Liblas Reader");

    BOOST_CHECK(reader.getNumPoints() == 1065);
    BOOST_CHECK(355 * 3 == 1065);

    CacheFilter cache(reader, 1, 355);
    BOOST_CHECK(cache.getNumPoints() == 1065);

    const Schema& schema = cache.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointBuffer data(layout, 355);

    boost::uint32_t numRead;

    {
        libpc::SequentialIterator* iter = cache.createSequentialIterator();
        BOOST_CHECK(iter->getIndex() == 0);

        numRead = iter->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iter->getIndex() == 355);

        check_p0_p1_p2(data, schema);

        numRead = iter->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iter->getIndex() == 710);

        check_p355_p356_p357(data, schema);

        numRead = iter->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iter->getIndex() == 1065);

        check_p710_p711_p712(data, schema);

        delete iter;
    }

    {
        libpc::RandomIterator* iter = cache.createRandomIterator();
        BOOST_CHECK(iter->getIndex() == 0);

        // read the middle third
        iter->seek(355);
        BOOST_CHECK(iter->getIndex() == 355);
        numRead = iter->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iter->getIndex() == 710);

        check_p355_p356_p357(data, schema);

        // read the first third
        iter->seek(0);
        BOOST_CHECK(iter->getIndex() == 0);
        numRead = iter->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iter->getIndex() == 355);

        check_p0_p1_p2(data, schema);

        // read the first third again
        iter->seek(0);
        BOOST_CHECK(iter->getIndex() == 0);
        numRead = iter->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iter->getIndex() == 355);

        check_p0_p1_p2(data, schema);

        // read the last third
        iter->seek(710);
        BOOST_CHECK(iter->getIndex() == 710);
        numRead = iter->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iter->getIndex() == 1065);

        check_p710_p711_p712(data, schema);

        delete iter;
    }

    return;
}


BOOST_AUTO_TEST_CASE(test_simultaneous_iters)
{
    LiblasReader reader(TestConfig::g_data_path + "1.2-with-color.las");
    BOOST_CHECK(reader.getName() == "Liblas Reader");

    BOOST_CHECK(reader.getNumPoints() == 1065);
    BOOST_CHECK(355 * 3 == 1065);

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointBuffer data(layout, 355);

    boost::uint32_t numRead;

    libpc::SequentialIterator* iterS1 = reader.createSequentialIterator();
    BOOST_CHECK(iterS1->getIndex() == 0);

    libpc::SequentialIterator* iterS2 = reader.createSequentialIterator();
    BOOST_CHECK(iterS2->getIndex() == 0);

    libpc::RandomIterator* iterR1 = reader.createRandomIterator();
    BOOST_CHECK(iterR1->getIndex() == 0);

    libpc::RandomIterator* iterR2 = reader.createRandomIterator();
    BOOST_CHECK(iterR2->getIndex() == 0);

    {
        numRead = iterS1->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iterS1->getIndex() == 355);

        check_p0_p1_p2(data, schema);
    }

    {
        iterS2->skip(355);

        numRead = iterS2->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iterS2->getIndex() == 710);

        check_p355_p356_p357(data, schema);
    }

    {
        iterR1->seek(355);
        numRead = iterR1->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iterR1->getIndex() == 710);

        check_p355_p356_p357(data, schema);
    }

    {
        iterR2->seek(0);
        numRead = iterR2->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iterR2->getIndex() == 355);

        check_p0_p1_p2(data, schema);
    }

    {
        iterS1->skip(355);
        numRead = iterS1->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iterS1->getIndex() == 1065);

        check_p710_p711_p712(data, schema);
    }

    {
        iterS2->skip(0);

        numRead = iterS2->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iterS2->getIndex() == 1065);

        check_p710_p711_p712(data, schema);
    }

    {
        iterR1->seek(355);
        numRead = iterR1->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iterR1->getIndex() == 710);

        check_p355_p356_p357(data, schema);
    }

    {
        iterR2->seek(710);
        numRead = iterR2->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iterR2->getIndex() == 1065);

        check_p710_p711_p712(data, schema);
    }

    {
        iterR1->seek(0);
        numRead = iterR1->read(data);
        BOOST_CHECK(numRead == 355);
        BOOST_CHECK(iterR1->getIndex() == 355);

        check_p0_p1_p2(data, schema);
    }

    delete iterS1;
    delete iterS2;
    delete iterR1;
    delete iterR2;

    return;
}

BOOST_AUTO_TEST_SUITE_END()
