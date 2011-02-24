#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include "libpc/FauxReader.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(FauxReaderTest)

BOOST_AUTO_TEST_CASE(test_constant)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    FauxReader reader(bounds, 1000, FauxReader::Constant);

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointData data(layout, 750);
    
    boost::uint32_t numRead = reader.readPoints(data);

    BOOST_CHECK(numRead == 750);

    std::size_t offsetX = schema.getDimensionIndex(Dimension::Field_X);
    std::size_t offsetY = schema.getDimensionIndex(Dimension::Field_Y);
    std::size_t offsetZ = schema.getDimensionIndex(Dimension::Field_Z);
    std::size_t offsetT = schema.getDimensionIndex(Dimension::Field_Time);

    for (boost::uint32_t i=0; i<numRead; i++)
    {
        float x = data.getField<float>(i, offsetX);
        float y = data.getField<float>(i, offsetY);
        float z = data.getField<float>(i, offsetZ);
        boost::uint64_t t = data.getField<boost::uint64_t>(i, offsetT);

        BOOST_CHECK(x == 1.0);
        BOOST_CHECK(y == 2.0);
        BOOST_CHECK(z == 3.0);
        BOOST_CHECK(t == i);
    }

    return;
}


BOOST_AUTO_TEST_CASE(test_random)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    FauxReader reader(bounds, 1000, FauxReader::Random);

    const Schema& schema = reader.getHeader().getSchema();
    SchemaLayout layout(schema);

    PointData data(layout, 750);
    
    boost::uint32_t numRead = reader.readPoints(data);

    BOOST_CHECK(numRead == 750);

    std::size_t offsetX = schema.getDimensionIndex(Dimension::Field_X);
    std::size_t offsetY = schema.getDimensionIndex(Dimension::Field_Y);
    std::size_t offsetZ = schema.getDimensionIndex(Dimension::Field_Z);
    std::size_t offsetT = schema.getDimensionIndex(Dimension::Field_Time);

    for (boost::uint32_t i=0; i<numRead; i++)
    {
        float x = data.getField<float>(i, offsetX);
        float y = data.getField<float>(i, offsetY);
        float z = data.getField<float>(i, offsetZ);
        boost::uint64_t t = data.getField<boost::uint64_t>(i, offsetT);

        BOOST_CHECK(x >= 1.0 && x <= 101.0);
        BOOST_CHECK(y >= 2.0 && y <= 102.0);
        BOOST_CHECK(z >= 3.0 && z <= 103.0);
        BOOST_CHECK(t == i);
    }

    return;
}

BOOST_AUTO_TEST_SUITE_END()
