#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include "libpc/PointData.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(PointDataTest)

BOOST_AUTO_TEST_CASE(test_ctor)
{
    Dimension d1("X", Dimension::Uint32);
    Dimension d2("Y", Dimension::Uint32);
    Schema schema;
    schema.addDimension(d1);
    schema.addDimension(d2);
    SchemaLayout layout(schema);

    PointData data(layout, 10);

    BOOST_CHECK(data.getNumPoints() == 10);
    BOOST_CHECK(data.getSchemaLayout() == layout);

    return;
}


static PointData* makeTestBuffer()
{
    Dimension d1("X", Dimension::Uint8);
    Dimension d2("Y", Dimension::Int32);
    Dimension d3("Z", Dimension::Double);
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

    PointData* data = new PointData(layout, 17);

    // write the data into the buffer
    for (int i=0; i<17; i++)
    {
      const boost::uint8_t x = i+1;
      const boost::int32_t y = i*10;
      const double z = i * 100;

      data->setField(i, 0, x);
      data->setField(i, 1, y);
      data->setField(i, 2, z);
    }

    return data;
}


static void verifyTestBuffer(const PointData& data)
{
    // read the data back out
    for (int i=0; i<17; i++)
    {
      const boost::uint8_t x = data.getField<boost::uint8_t>(i, 0);
      const boost::int32_t y = data.getField<boost::int32_t>(i, 1);
      const double z = data.getField<double>(i, 2);

      BOOST_CHECK(x == i+1);
      BOOST_CHECK(y == i*10);
      BOOST_CHECK(z == i*100);
    }
}

BOOST_AUTO_TEST_CASE(test_get_set)
{
    PointData* data = makeTestBuffer();
    verifyTestBuffer(*data);
    delete data;
}


BOOST_AUTO_TEST_CASE(test_copy)
{
    PointData* data = makeTestBuffer();
   
    PointData d2(data->getSchemaLayout(), 19);

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
      BOOST_CHECK(z == ii*100);
    }
    for (int i=1; i<18; i++)
    {
      const boost::uint8_t x = d2.getField<boost::uint8_t>(i, 0);
      const boost::int32_t y = d2.getField<boost::int32_t>(i, 1);
      const double z = d2.getField<double>(i, 2);

      int ii = i-1;
      BOOST_CHECK(x == ii+1);
      BOOST_CHECK(y == ii*10);
      BOOST_CHECK(z == ii*100);
    }
    {
      const boost::uint8_t x = d2.getField<boost::uint8_t>(18, 0);
      const boost::int32_t y = d2.getField<boost::int32_t>(18, 1);
      const double z = d2.getField<double>(18, 2);

      int ii = 11;
      BOOST_CHECK(x == ii+1);
      BOOST_CHECK(y == ii*10);
      BOOST_CHECK(z == ii*100);
    }

    delete data;
}

BOOST_AUTO_TEST_SUITE_END()
