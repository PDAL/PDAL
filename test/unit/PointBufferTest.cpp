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

#include <pdal/PointBuffer.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(PointBufferTest)

BOOST_AUTO_TEST_CASE(test_ctor)
{
    Dimension d1(Dimension::Field_X, Dimension::Uint32);
    Dimension d2(Dimension::Field_Y, Dimension::Uint32);
    Schema schema;
    schema.addDimension(d1);
    schema.addDimension(d2);
    SchemaLayout layout(schema);

    PointBuffer data(layout, 10);

    BOOST_CHECK(data.getCapacity() == 10);
    BOOST_CHECK(data.getSchemaLayout() == layout);

    return;
}


PointBuffer* makeTestBuffer()
{
    Dimension d1(Dimension::Field_X, Dimension::Uint8);
    Dimension d2(Dimension::Field_Y, Dimension::Int32);
    Dimension d3(Dimension::Field_Z, Dimension::Double);
    Schema schema;
    schema.addDimension(d1);
    schema.addDimension(d2);
    schema.addDimension(d3);
    SchemaLayout layout(schema);

    std::size_t offX = layout.getDimensionLayout(0).getByteOffset();
    BOOST_CHECK(offX==0);
    std::size_t offY = layout.getDimensionLayout(1).getByteOffset();
    BOOST_CHECK(offY==1);
    std::size_t offZ = layout.getDimensionLayout(2).getByteOffset();
    BOOST_CHECK(offZ==5);

    boost::uint32_t capacity = 17;
    PointBuffer* data = new PointBuffer(layout, capacity);

    BOOST_CHECK(data->getCapacity() == capacity);
    // write the data into the buffer
    for (boost::uint32_t i=0; i<data->getCapacity(); i++)
    {
      const boost::uint8_t x = static_cast<boost::uint8_t>(i)+1;
      const boost::int32_t y = i*10;
      const double z = i * 100;

      data->setField(i, 0, x);
      data->setField(i, 1, y);
      data->setField(i, 2, z);
      data->setNumPoints(i+1);

    }
    BOOST_CHECK(data->getCapacity() ==17);
    BOOST_CHECK(data->getNumPoints() ==17);
    return data;
}


static void verifyTestBuffer(const PointBuffer& data)
{
    // read the data back out
    for (int i=0; i<17; i++)
    {
      const boost::uint8_t x = data.getField<boost::uint8_t>(i, 0);
      const boost::int32_t y = data.getField<boost::int32_t>(i, 1);
      const double z = data.getField<double>(i, 2);

      BOOST_CHECK(x == i+1);
      BOOST_CHECK(y == i*10);

      BOOST_CHECK(Utils::compare_approx(z, static_cast<double>(i)*100.0, (std::numeric_limits<double>::min)()) == true);

    }
}

BOOST_AUTO_TEST_CASE(test_get_set)
{
    PointBuffer* data = makeTestBuffer();
    verifyTestBuffer(*data);
    delete data;
}


BOOST_AUTO_TEST_CASE(test_copy)
{
    PointBuffer* data = makeTestBuffer();
   
    PointBuffer d2(data->getSchemaLayout(), 19);

    d2.copyPointFast(0, 10, *data);
    d2.copyPointFast(18, 11, *data);
    d2.copyPointsFast(1, 0, *data, 17);

    // read the data back out
    {
      const boost::uint8_t x = d2.getField<boost::uint8_t>(0, 0);
      const boost::int32_t y = d2.getField<boost::int32_t>(0, 1);
      const double z = d2.getField<double>(0, 2);

      int ii = 10;


      BOOST_CHECK(x == ii+1);
      BOOST_CHECK(y == ii*10);
      BOOST_CHECK(Utils::compare_approx(z, ii*100.0, (std::numeric_limits<double>::min)()) == true);
    }
    for (int i=1; i<18; i++)
    {
      const boost::uint8_t x = d2.getField<boost::uint8_t>(i, 0);
      const boost::int32_t y = d2.getField<boost::int32_t>(i, 1);
      const double z = d2.getField<double>(i, 2);

      int ii = i-1;
      BOOST_CHECK(x == ii+1);
      BOOST_CHECK(y == ii*10);
      BOOST_CHECK(Utils::compare_approx(z, ii*100.0, (std::numeric_limits<double>::min)()) == true);
    }
    {
      const boost::uint8_t x = d2.getField<boost::uint8_t>(18, 0);
      const boost::int32_t y = d2.getField<boost::int32_t>(18, 1);
      const double z = d2.getField<double>(18, 2);

      int ii = 11;
      BOOST_CHECK(x == ii+1);
      BOOST_CHECK(y == ii*10);
      BOOST_CHECK(Utils::compare_approx(z, ii*100.0, (std::numeric_limits<double>::min)()) == true);
    }

    delete data;
}

BOOST_AUTO_TEST_CASE(test_copy_constructor)
{
    PointBuffer* data = makeTestBuffer();
    
    PointBuffer d2(*data);
    verifyTestBuffer(d2);
    delete data;
}

BOOST_AUTO_TEST_CASE(test_assignment_constructor)
{
    PointBuffer* data = makeTestBuffer();
    
    PointBuffer d2 = *data;
    verifyTestBuffer(d2);
    delete data;
}

BOOST_AUTO_TEST_SUITE_END()
