#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "libpc/Bounds.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(BoundsTest)

BOOST_AUTO_TEST_CASE(test_ctor)
{
    Bounds<int> b1;
    BOOST_CHECK(b1.size() == 0);
    BOOST_CHECK(b1.empty());

    Bounds<int> b2(1,2,3,4);
    BOOST_CHECK(b2.size() == 2);
    BOOST_CHECK(!b2.empty());

    Bounds<int> b3(1,2,3,4,5,6);
    BOOST_CHECK(b3.size() == 3);
    BOOST_CHECK(!b3.empty());
}

BOOST_AUTO_TEST_CASE(test_equals)
{
    Bounds<int> b1(1,2,3,4);
    Bounds<int> b2(1,2,3,4);
    Bounds<int> b3(1,2,32,4);
    Bounds<int> b4(1,2,3,4,5,6);

    BOOST_CHECK(b1 == b1);
    BOOST_CHECK(b1 == b2);
    BOOST_CHECK(b2 == b1);
    BOOST_CHECK(b1 != b3);
    BOOST_CHECK(b3 != b1);
    BOOST_CHECK(b1 != b4);
    BOOST_CHECK(b1 != b4);
    BOOST_CHECK(b4 != b1);
}

BOOST_AUTO_TEST_CASE(test_ctor2)
{
    Bounds<int> b1(1,2,3,4);
    Bounds<int> b2(b1);
    BOOST_CHECK(b1==b2);

    Range<int> v1(1,3);
    Range<int> v2(2,4);
    std::vector<Range<int>> rv;
    rv.push_back(v1);
    rv.push_back(v2);
    Bounds<int> b3(rv);
    BOOST_CHECK(b3==b1);
    BOOST_CHECK(b3.dimensions() == rv);

    std::vector<int> vlo;
    vlo.push_back(1);
    vlo.push_back(2);
    std::vector<int> vhi;
    vhi.push_back(3);
    vhi.push_back(4);
    Bounds<int> b4(vlo, vhi);
    BOOST_CHECK(b4==b1);

    Bounds<int> b5 = b1;
    BOOST_CHECK(b5==b1);
}

BOOST_AUTO_TEST_CASE(test_accessor)
{
    Bounds<int> b2(1,2,3,4,5,6);
    BOOST_CHECK(b2.getMinimum(0)==1);
    BOOST_CHECK(b2.getMinimum(1)==2);
    BOOST_CHECK(b2.getMinimum(2)==3);
    BOOST_CHECK(b2.getMaximum(0)==4);
    BOOST_CHECK(b2.getMaximum(1)==5);
    BOOST_CHECK(b2.getMaximum(2)==6);

    Vector<int> vmin(1,2,3);
    Vector<int> vmax(4,5,6);
    BOOST_CHECK(b2.getMinimum()==vmin);
    BOOST_CHECK(b2.getMaximum()==vmax);


}

BOOST_AUTO_TEST_CASE(test_math)
{
  // BUG: check intersects, overlaps, contains, shift, scale, clip, grow, volume
}

// BUG: check operator<<

BOOST_AUTO_TEST_SUITE_END()
