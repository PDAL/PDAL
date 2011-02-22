#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include "libpc/Dimension.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(DimensionTest)

BOOST_AUTO_TEST_CASE(test_ctor)
{
    Dimension d1("X", Dimension::Uint32);

    BOOST_CHECK(d1.getName() == "X");
    BOOST_CHECK(d1.getDataType() == Dimension::Uint32);

    Dimension d2(d1);
    BOOST_CHECK(d1.getName() == "X");
    BOOST_CHECK(d1.getDataType() == Dimension::Uint32);

    Dimension d3 = d1;
    BOOST_CHECK(d1.getName() == "X");
    BOOST_CHECK(d1.getDataType() == Dimension::Uint32);

    BOOST_CHECK(d1 == d1);
    BOOST_CHECK(d1 == d2);
    BOOST_CHECK(d2 == d1);
    BOOST_CHECK(d1 == d3);
    BOOST_CHECK(d3 == d1);

    Dimension d4("Y", Dimension::Uint32);
    BOOST_CHECK(d1 != d4);
    BOOST_CHECK(d4 != d1);
}


BOOST_AUTO_TEST_CASE(test_)
{
    Dimension d1("X", Dimension::Uint32);

}

BOOST_AUTO_TEST_SUITE_END()
