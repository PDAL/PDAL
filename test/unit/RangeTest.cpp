#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include "libpc/Range.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(RangeTest)

BOOST_AUTO_TEST_CASE(test_ctor)
{
    Range<boost::uint8_t> r1;
    BOOST_CHECK(r1.getMinimum() == 255);
    BOOST_CHECK(r1.getMaximum() == 0);

    Range<boost::uint8_t> r2(10,20);
    BOOST_CHECK(r2.getMinimum() == 10);
    BOOST_CHECK(r2.getMaximum() == 20);
}

BOOST_AUTO_TEST_CASE(test_equals)
{
    Range<boost::uint8_t> r1(10,20);
    Range<boost::uint8_t> r2(10,20);
    Range<boost::uint8_t> r3(11,20);
    Range<boost::uint8_t> r4(10,21);

    BOOST_CHECK(r1 == r1);
    BOOST_CHECK(r1 == r2);
    BOOST_CHECK(r2 == r1);
    BOOST_CHECK(r1 != r3);
    BOOST_CHECK(r3 != r1);
    BOOST_CHECK(r1 != r4);
    BOOST_CHECK(r4 != r1);
}

BOOST_AUTO_TEST_CASE(test_ctor2)
{
    Range<boost::uint8_t> r1(10,20);
    Range<boost::uint8_t> r2(r1);
    Range<boost::uint8_t> r3 = r1;

    BOOST_CHECK(r1 == r2);
    BOOST_CHECK(r3 == r1);
}

BOOST_AUTO_TEST_CASE(test_accessors)
{
    Range<boost::uint8_t> r1(10,20);

    BOOST_CHECK(r1.getMinimum() == 10);
    BOOST_CHECK(r1.getMaximum() == 20);

    r1.setMinimum(20);
    r1.setMaximum(30);

    BOOST_CHECK(r1.getMinimum() == 20);
    BOOST_CHECK(r1.getMaximum() == 30);
}

BOOST_AUTO_TEST_CASE(test_contains)
{
    Range<boost::uint8_t> r1(10,20);
    BOOST_CHECK(!r1.contains(9));
    BOOST_CHECK(r1.contains(10));
    BOOST_CHECK(r1.contains(11));
    BOOST_CHECK(r1.contains(20));
    BOOST_CHECK(!r1.contains(21));

    Range<boost::uint8_t> r2(10,20);
    BOOST_CHECK(r1.contains(r2));

    Range<boost::uint8_t> r3(9,20);
    BOOST_CHECK(!r1.contains(r3));

    Range<boost::uint8_t> r4(10,21);
    BOOST_CHECK(!r1.contains(r4));

    Range<boost::uint8_t> r5(9,21);
    BOOST_CHECK(!r1.contains(r5));

    BOOST_CHECK(r1.overlaps(r2));
    BOOST_CHECK(r1.overlaps(r3));
    BOOST_CHECK(r1.overlaps(r4));
    BOOST_CHECK(r1.overlaps(r5));
    BOOST_CHECK(r2.overlaps(r1));
    BOOST_CHECK(r3.overlaps(r1));
    BOOST_CHECK(r4.overlaps(r1));
    BOOST_CHECK(r5.overlaps(r1));
}

BOOST_AUTO_TEST_CASE(test_empty)
{
    Range<boost::uint8_t> r1;
    Range<boost::uint8_t> r2(11,22);
    BOOST_CHECK(r1.empty());
    BOOST_CHECK(!r2.empty());
}

BOOST_AUTO_TEST_CASE(test_math)
{
    Range<boost::uint8_t> r1(11,22);
    BOOST_CHECK(r1.length() == 11);

    r1.shift(3);
    BOOST_CHECK(r1.getMinimum() == 14);
    BOOST_CHECK(r1.getMaximum() == 25);
    BOOST_CHECK(r1.length() == 11);

    r1.scale(3);
    BOOST_CHECK(r1.getMinimum() == 42);
    BOOST_CHECK(r1.getMaximum() == 75);
    BOOST_CHECK(r1.length() == 33);
}
    
BOOST_AUTO_TEST_CASE(test_ranges)
{
    Range<boost::uint8_t> r1(11,22);

    r1.clip(Range<boost::uint8_t>(0,100));
    BOOST_CHECK(r1.getMinimum() == 11);
    BOOST_CHECK(r1.getMaximum() == 22);

    r1.clip(Range<boost::uint8_t>(15,50));
    BOOST_CHECK(r1.getMinimum() == 15);
    BOOST_CHECK(r1.getMaximum() == 22);

    r1.clip(Range<boost::uint8_t>(10,18));
    BOOST_CHECK(r1.getMinimum() == 15);
    BOOST_CHECK(r1.getMaximum() == 18);

    r1.grow(10);
    BOOST_CHECK(r1.getMinimum() == 10);
    BOOST_CHECK(r1.getMaximum() == 18);

    r1.grow(20);
    BOOST_CHECK(r1.getMinimum() == 10);
    BOOST_CHECK(r1.getMaximum() == 20);

    r1.grow(15);
    BOOST_CHECK(r1.getMinimum() == 10);
    BOOST_CHECK(r1.getMaximum() == 20);

    r1.grow(Range<boost::uint8_t>(12,18));
    BOOST_CHECK(r1.getMinimum() == 10);
    BOOST_CHECK(r1.getMaximum() == 20);

    r1.grow(Range<boost::uint8_t>(8,22));
    BOOST_CHECK(r1.getMinimum() == 8);
    BOOST_CHECK(r1.getMaximum() == 22);

    r1.grow(8,122);
    BOOST_CHECK(r1.getMinimum() == 8);
    BOOST_CHECK(r1.getMaximum() == 122);
}

// BUG: check operator<<

BOOST_AUTO_TEST_SUITE_END()
