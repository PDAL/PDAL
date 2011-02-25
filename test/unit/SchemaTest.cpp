#ifdef _MSC_VER
#define BOOST_TEST_DYN_LINK
#endif

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include "libpc/Schema.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(SchemaTest)

BOOST_AUTO_TEST_CASE(test_ctor)
{
    Dimension d1(Dimension::Field_X, Dimension::Uint32);
    Dimension d2(Dimension::Field_Y, Dimension::Uint32);

    Schema s1;
    s1.addDimension(d1);
    s1.addDimension(d2);

    Schema s2(s1);
    Schema s3 = s1;

    BOOST_CHECK(s1==s1);
    BOOST_CHECK(s1==s2);
    BOOST_CHECK(s2==s1);
    BOOST_CHECK(s1==s3);
    BOOST_CHECK(s3==s1);

    Schema s4;
    s4.addDimension(d1);
    BOOST_CHECK(s1!=s4);
    BOOST_CHECK(s4!=s1);
}


BOOST_AUTO_TEST_CASE(test_layout)
{
    Dimension d1(Dimension::Field_X, Dimension::Uint32);
    Dimension d2(Dimension::Field_Y, Dimension::Uint32);

    Schema s1;
    s1.addDimension(d1);
    s1.addDimension(d2);
    BOOST_CHECK(s1.getDimensionIndex(Dimension::Field_X) == 0);
    BOOST_CHECK(s1.getDimensionIndex(Dimension::Field_Y) == 1);
    
    Schema s2;
    s2.addDimension(d1);
    BOOST_CHECK(s2.hasDimension(Dimension::Field_X) == true);
    BOOST_CHECK(s2.getDimensionIndex(Dimension::Field_X) == 0);
    BOOST_CHECK(s2.hasDimension(Dimension::Field_Y) == false);

    SchemaLayout l1(s1);
    SchemaLayout l2(l1);
    SchemaLayout l3 = l1;
    SchemaLayout l4(s2);

    BOOST_CHECK(l1==l1);
    BOOST_CHECK(l1==l2);
    BOOST_CHECK(l2==l1);
    BOOST_CHECK(l1==l3);
    BOOST_CHECK(l3==l1);
    BOOST_CHECK(l1!=l4);
    BOOST_CHECK(l4!=l1);
}


BOOST_AUTO_TEST_CASE(test_layout_size)
{
    Dimension d1(Dimension::Field_X, Dimension::Uint32);
    Dimension d2(Dimension::Field_Y, Dimension::Uint32);
    Schema s1;
    s1.addDimension(d1);
    s1.addDimension(d2);
    SchemaLayout sl1(s1);

    const DimensionLayout& dl1 = sl1.getDimensionLayout(0);
    BOOST_CHECK(dl1.getDimension() == d1);
    BOOST_CHECK(dl1.getPosition() == 0);
    BOOST_CHECK(dl1.getByteOffset() == 0);

    const DimensionLayout& dl2 = sl1.getDimensionLayout(1);
    BOOST_CHECK(dl2.getDimension() == d2);
    BOOST_CHECK(dl2.getPosition() == 1);
    BOOST_CHECK(dl2.getByteOffset() == 4);
}


BOOST_AUTO_TEST_SUITE_END()
