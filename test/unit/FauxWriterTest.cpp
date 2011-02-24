#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include "libpc/FauxReader.hpp"
#include "libpc/FauxWriter.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(FauxWriterTest)

BOOST_AUTO_TEST_CASE(test_1)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    FauxReader reader(bounds, 1000, FauxReader::Constant);

    FauxWriter writer(reader);

    boost::uint64_t numWritten = writer.write(750);

    BOOST_CHECK(numWritten == 750);

    BOOST_CHECK(writer.getMinX() == 1.0);
    BOOST_CHECK(writer.getMinY() == 2.0);
    BOOST_CHECK(writer.getMinZ() == 3.0);
    BOOST_CHECK(writer.getMaxX() == 1.0);
    BOOST_CHECK(writer.getMaxY() == 2.0);
    BOOST_CHECK(writer.getMaxZ() == 3.0);
    BOOST_CHECK(writer.getAvgX() == 1.0);
    BOOST_CHECK(writer.getAvgY() == 2.0);
    BOOST_CHECK(writer.getAvgZ() == 3.0);

    return;
}

BOOST_AUTO_TEST_CASE(test_2)
{
    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    FauxReader reader(bounds, 1000, FauxReader::Random);

    FauxWriter writer(reader);

    boost::uint64_t numWritten = writer.write(750);

    BOOST_CHECK(numWritten == 750);

    // test all the values to +/- 1.0
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMinX(), 1.0, 1.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMinY(), 2.0, 1.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMinZ(), 3.0, 1.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMaxX(), 101.0, 1.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMaxY(), 102.0, 1.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMaxZ(), 103.0, 1.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getAvgX(), 51.0, 1.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getAvgY(), 52.0, 1.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getAvgZ(), 53.0, 1.0));

    return;
}

BOOST_AUTO_TEST_SUITE_END()
