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

#include <libpc/Header.hpp>
#include <libpc/PointBuffer.hpp>
#include <libpc/SchemaLayout.hpp>
#include <libpc/drivers/faux/Reader.hpp>
#include <libpc/drivers/faux/Iterator.hpp>

using namespace libpc;

BOOST_AUTO_TEST_SUITE(FauxReaderTest)

BOOST_AUTO_TEST_CASE(test_constant_mode_sequential_iter)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    libpc::drivers::faux::Reader reader(bounds, 1000, libpc::drivers::faux::Reader::Constant);

    BOOST_CHECK_EQUAL(reader.getName(), "Faux Reader");

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointBuffer data(layout, 750);
 
    SequentialIterator* iter = reader.createSequentialIterator();
    boost::uint32_t numRead = iter->read(data);

    BOOST_CHECK_EQUAL(numRead, 750);

    int offsetX = schema.getDimensionIndex(Dimension::Field_X);
    int offsetY = schema.getDimensionIndex(Dimension::Field_Y);
    int offsetZ = schema.getDimensionIndex(Dimension::Field_Z);
    int offsetT = schema.getDimensionIndex(Dimension::Field_Time);

    for (boost::uint32_t i=0; i<numRead; i++)
    {
        double x = data.getField<double>(i, offsetX);
        double y = data.getField<double>(i, offsetY);
        double z = data.getField<double>(i, offsetZ);
        boost::uint64_t t = data.getField<boost::uint64_t>(i, offsetT);

        BOOST_CHECK_CLOSE(x, 2.0, (std::numeric_limits<double>::min)());
        BOOST_CHECK_CLOSE(y, 2.0, (std::numeric_limits<double>::min)());
        BOOST_CHECK_CLOSE(z, 3.0, (std::numeric_limits<double>::min)());
        BOOST_CHECK_EQUAL(t, i);
    }

    delete iter;

    return;
}


BOOST_AUTO_TEST_CASE(test_constant_mode_random_iter)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    libpc::drivers::faux::Reader reader(bounds, 1000, libpc::drivers::faux::Reader::Constant);

    BOOST_CHECK_EQUAL(reader.getName(), "Faux Reader");

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointBuffer data(layout, 10);

    int offsetX = schema.getDimensionIndex(Dimension::Field_X);
    int offsetY = schema.getDimensionIndex(Dimension::Field_Y);
    int offsetZ = schema.getDimensionIndex(Dimension::Field_Z);
    int offsetT = schema.getDimensionIndex(Dimension::Field_Time);

    RandomIterator* iter = reader.createRandomIterator();

    boost::uint32_t numRead = iter->read(data);
    BOOST_CHECK_EQUAL(numRead, 10);

    {
        for (boost::uint32_t i=0; i<numRead; i++)
        {
            double x = data.getField<double>(i, offsetX);
            double y = data.getField<double>(i, offsetY);
            double z = data.getField<double>(i, offsetZ);
            boost::uint64_t t = data.getField<boost::uint64_t>(i, offsetT);

            BOOST_CHECK_CLOSE(x, 1.0, (std::numeric_limits<double>::min)());
            BOOST_CHECK_CLOSE(y, 2.0, (std::numeric_limits<double>::min)());
            BOOST_CHECK_CLOSE(z, 3.0, (std::numeric_limits<double>::min)());
            BOOST_CHECK_EQUAL(t, i);
        }
    }

    numRead = iter->read(data);
    BOOST_CHECK_EQUAL(numRead, 10);

    {
        for (boost::uint32_t i=0; i<numRead; i++)
        {
            double x = data.getField<double>(i, offsetX);
            double y = data.getField<double>(i, offsetY);
            double z = data.getField<double>(i, offsetZ);
            boost::uint64_t t = data.getField<boost::uint64_t>(i, offsetT);

            BOOST_CHECK_CLOSE(x, 1.0, (std::numeric_limits<double>::min)());
            BOOST_CHECK_CLOSE(y, 2.0, (std::numeric_limits<double>::min)());
            BOOST_CHECK_CLOSE(z, 3.0, (std::numeric_limits<double>::min)());
            BOOST_CHECK_EQUAL(t, i+10);
        }
    }

    boost::uint64_t newPos = iter->seek(99);
    BOOST_CHECK_EQUAL(newPos, 99);
    numRead = iter->read(data);
    BOOST_CHECK_EQUAL(numRead, 10);

    {
        for (boost::uint32_t i=0; i<numRead; i++)
        {
            double x = data.getField<double>(i, offsetX);
            double y = data.getField<double>(i, offsetY);
            double z = data.getField<double>(i, offsetZ);
            boost::uint64_t t = data.getField<boost::uint64_t>(i, offsetT);

            BOOST_CHECK_CLOSE(x, 1.0, (std::numeric_limits<double>::min)());
            BOOST_CHECK_CLOSE(y, 2.0, (std::numeric_limits<double>::min)());
            BOOST_CHECK_CLOSE(z, 3.0, (std::numeric_limits<double>::min)());
            BOOST_CHECK_EQUAL(t, i+99);

        }
    }

    newPos = iter->seek(7);
    BOOST_CHECK_EQUAL(newPos, 7);
    numRead = iter->read(data);
    BOOST_CHECK_EQUAL(numRead, 10);

    {
        for (boost::uint32_t i=0; i<numRead; i++)
        {
            double x = data.getField<double>(i, offsetX);
            double y = data.getField<double>(i, offsetY);
            double z = data.getField<double>(i, offsetZ);
            boost::uint64_t t = data.getField<boost::uint64_t>(i, offsetT);

            BOOST_CHECK_CLOSE(x, 1.0, (std::numeric_limits<double>::min)());
            BOOST_CHECK_CLOSE(y, 2.0, (std::numeric_limits<double>::min)());
            BOOST_CHECK_CLOSE(z, 3.0, (std::numeric_limits<double>::min)());
            BOOST_CHECK_EQUAL(t, i+7);

        }
    }

    delete iter;

    return;
}


BOOST_AUTO_TEST_CASE(test_random_mode)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    libpc::drivers::faux::Reader reader(bounds, 1000, libpc::drivers::faux::Reader::Random);

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointBuffer data(layout, 750);

    SequentialIterator* iter = reader.createSequentialIterator();
    boost::uint32_t numRead = iter->read(data);

    BOOST_CHECK_EQUAL(numRead, 750);

    int offsetX = schema.getDimensionIndex(Dimension::Field_X);
    int offsetY = schema.getDimensionIndex(Dimension::Field_Y);
    int offsetZ = schema.getDimensionIndex(Dimension::Field_Z);
    int offsetT = schema.getDimensionIndex(Dimension::Field_Time);

    for (boost::uint32_t i=0; i<numRead; i++)
    {
        double x = data.getField<double>(i, offsetX);
        double y = data.getField<double>(i, offsetY);
        double z = data.getField<double>(i, offsetZ);
        boost::uint64_t t = data.getField<boost::uint64_t>(i, offsetT);
        
        BOOST_CHECK_GE(x, 1.0);
        BOOST_CHECK_LE(x, 101.0);
        
        BOOST_CHECK_GE(y, 2.0);
        BOOST_CHECK_LE(y, 102.0);
        
        BOOST_CHECK_GE(z, 3.0);
        BOOST_CHECK_LE(z, 103.0);
        
        BOOST_CHECK_EQUAL(t, i);
        // BOOST_CHECK(x >= 1.0 && x <= 101.0);
        // BOOST_CHECK(y >= 2.0 && y <= 102.0);
        // BOOST_CHECK(z >= 3.0 && z <= 103.0);
        // BOOST_CHECK(t == i);
    }

    delete iter;

    return;
}

BOOST_AUTO_TEST_CASE(test_ramp_mode_1)
{
    Bounds<double> bounds(0,0,0,4,4,4);
    libpc::drivers::faux::Reader reader(bounds, 2, libpc::drivers::faux::Reader::Ramp);

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointBuffer data(layout, 2);

    SequentialIterator* iter = reader.createSequentialIterator();
    boost::uint32_t numRead = iter->read(data);

    BOOST_CHECK_EQUAL(numRead, 2);

    const int offsetX = schema.getDimensionIndex(Dimension::Field_X);
    const int offsetY = schema.getDimensionIndex(Dimension::Field_Y);
    const int offsetZ = schema.getDimensionIndex(Dimension::Field_Z);
    const int offsetT = schema.getDimensionIndex(Dimension::Field_Time);

    const double x0 = data.getField<double>(0, offsetX);
    const double y0 = data.getField<double>(0, offsetY);
    const double z0 = data.getField<double>(0, offsetZ);
    const boost::uint64_t t0 = data.getField<boost::uint64_t>(0, offsetT);

    const double x1 = data.getField<double>(1, offsetX);
    const double y1 = data.getField<double>(1, offsetY);
    const double z1 = data.getField<double>(1, offsetZ);
    const boost::uint64_t t1 = data.getField<boost::uint64_t>(1, offsetT);

    BOOST_CHECK_CLOSE(x0, 0.0, (std::numeric_limits<double>::min)());
    BOOST_CHECK_CLOSE(y0, 0.0, (std::numeric_limits<double>::min)());
    BOOST_CHECK_CLOSE(z0, 0.0, (std::numeric_limits<double>::min)());
    BOOST_CHECK_EQUAL(t0, 0);

    BOOST_CHECK_CLOSE(x1, 4.0, (std::numeric_limits<double>::min)());
    BOOST_CHECK_CLOSE(y1, 4.0, (std::numeric_limits<double>::min)());
    BOOST_CHECK_CLOSE(z1, 4.0, (std::numeric_limits<double>::min)());
    BOOST_CHECK_EQUAL(t1, 1);


    delete iter;

    return;
}


BOOST_AUTO_TEST_CASE(test_ramp_mode_2)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 152.0, 203.0);
    libpc::drivers::faux::Reader reader(bounds, 750, libpc::drivers::faux::Reader::Ramp);

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointBuffer data(layout, 750);

    SequentialIterator* iter = reader.createSequentialIterator();
    boost::uint32_t numRead = iter->read(data);

    BOOST_CHECK_EQUAL(numRead,750);

    int offsetX = schema.getDimensionIndex(Dimension::Field_X);
    int offsetY = schema.getDimensionIndex(Dimension::Field_Y);
    int offsetZ = schema.getDimensionIndex(Dimension::Field_Z);
    int offsetT = schema.getDimensionIndex(Dimension::Field_Time);

    double delX = (101.0 - 1.0) / (750.0 - 1.0);
    double delY = (152.0 - 2.0) / (750.0 - 1.0);
    double delZ = (203.0 - 3.0) / (750.0 - 1.0);

    for (boost::uint32_t i=0; i<numRead; i++)
    {
        double x = data.getField<double>(i, offsetX);
        double y = data.getField<double>(i, offsetY);
        double z = data.getField<double>(i, offsetZ);
        boost::uint64_t t = data.getField<boost::uint64_t>(i, offsetT);

        BOOST_CHECK_CLOSE(x, 1.0 + delX*i, (std::numeric_limits<double>::min)());
        BOOST_CHECK_CLOSE(y, 2.0 + delY*i, (std::numeric_limits<double>::min)());
        BOOST_CHECK_CLOSE(z, 3.0 + delZ*i, (std::numeric_limits<double>::min)());
        BOOST_CHECK_EQUAL(t, i);

    }

    delete iter;

    return;
}


BOOST_AUTO_TEST_CASE(test_custom_fields)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);

    Dimension dimY(Dimension::Field_Y, Dimension::Uint8);
    Dimension dimX(Dimension::Field_X, Dimension::Uint8);
    std::vector<Dimension> dims;
    dims.push_back(dimY);
    dims.push_back(dimX);

    libpc::drivers::faux::Reader reader(bounds, 1000, libpc::drivers::faux::Reader::Random, dims);

    const Schema& schema = reader.getHeader().getSchema();
    BOOST_CHECK_EQUAL(schema.getDimensions().size(), 2);
    BOOST_CHECK_EQUAL(schema.getDimension(0).getField(), Dimension::Field_Y);
    BOOST_CHECK_EQUAL(schema.getDimension(1).getField(), Dimension::Field_X);

    return;
}

BOOST_AUTO_TEST_SUITE_END()
