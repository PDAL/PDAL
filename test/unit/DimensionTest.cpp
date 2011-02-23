#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include "libpc/Dimension.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(DimensionTest)

BOOST_AUTO_TEST_CASE(test_ctor)
{
    Dimension d1(Dimension::Field_X, Dimension::Uint32);

    BOOST_CHECK(d1.getField() == Dimension::Field_X);
    BOOST_CHECK(d1.getFieldName() == "X");
    BOOST_CHECK(d1.getDataType() == Dimension::Uint32);

    Dimension d2(d1);
    BOOST_CHECK(d1.getField() == Dimension::Field_X);
    BOOST_CHECK(d1.getDataType() == Dimension::Uint32);

    Dimension d3 = d1;
    BOOST_CHECK(d1.getField() == Dimension::Field_X);
    BOOST_CHECK(d1.getDataType() == Dimension::Uint32);

    BOOST_CHECK(d1 == d1);
    BOOST_CHECK(d1 == d2);
    BOOST_CHECK(d2 == d1);
    BOOST_CHECK(d1 == d3);
    BOOST_CHECK(d3 == d1);

    Dimension d4(Dimension::Field_Y, Dimension::Uint32);
    BOOST_CHECK(d1 != d4);
    BOOST_CHECK(d4 != d1);
}

// BUG: more Dimension tests needed

BOOST_AUTO_TEST_CASE(test_layout)
{
    Dimension d1(Dimension::Field_X, Dimension::Uint32);
    DimensionLayout l1(d1);

    Dimension d2(Dimension::Field_Y, Dimension::Uint32);
    DimensionLayout l2(d2);

    DimensionLayout l3(l1);
    DimensionLayout l4 = l1;

    BOOST_CHECK(l1==l1);
    BOOST_CHECK(l1!=l2);
    BOOST_CHECK(l2!=l1);
    BOOST_CHECK(l1==l3);
    BOOST_CHECK(l3==l1);
    BOOST_CHECK(l1==l4);
    BOOST_CHECK(l4==l1);
}

BOOST_AUTO_TEST_SUITE_END()
