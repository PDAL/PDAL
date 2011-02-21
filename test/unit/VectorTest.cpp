#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "libpc/Vector.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(VectorTest)

BOOST_AUTO_TEST_CASE(test_ctor)
{
    Vector<int> v;
    BOOST_CHECK(v.size() == 0);

    Vector<int> v1(10);
    BOOST_CHECK(v1.size() == 1);

    Vector<int> v2(10,20);
    BOOST_CHECK(v2.size() == 2);

    Vector<int> v3(10,20,30);
    BOOST_CHECK(v3.size() == 3);

    std::vector<int> vec;
    vec.push_back(10);
    vec.push_back(20);
    vec.push_back(30);
    vec.push_back(30);
    Vector<int> vN(vec);
    BOOST_CHECK(vN.size()==4);

}

BOOST_AUTO_TEST_CASE(test_equals)
{
    Vector<int> a(10,20);
    Vector<int> b(10,20);
    Vector<int> c(10,30);
    Vector<int> d(10,20,30);
    
    BOOST_CHECK(a==a);
    BOOST_CHECK(a==b);
    BOOST_CHECK(b==a);
    BOOST_CHECK(a!=c);
    BOOST_CHECK(c!=a);
    BOOST_CHECK(d!=a);
}

BOOST_AUTO_TEST_CASE(test_ctor2)
{
    Vector<int> v3(1,2,3);

    Vector<int> v3x(v3);
    BOOST_CHECK(v3x.size() == 3);

    Vector<int> v3xx = v3x;
    BOOST_CHECK(v3xx.size() == 3);
}

BOOST_AUTO_TEST_CASE(test_accessor)
{
    Vector<int> v(1,2,3);

    v[0] = 11;
    v[1] = 22;
    v[2] = 33;

    BOOST_CHECK(v[0]==11);
    BOOST_CHECK(v[1]==22);
    BOOST_CHECK(v[2]==33);
}

BOOST_AUTO_TEST_CASE(test_math)
{
    Vector<int> v(1,2,3);

    v.shift(10);

    BOOST_CHECK(v[0]==11);
    BOOST_CHECK(v[1]==12);
    BOOST_CHECK(v[2]==13);

    v.scale(2);

    BOOST_CHECK(v[0]==22);
    BOOST_CHECK(v[1]==24);
    BOOST_CHECK(v[2]==26);
}

// BUG: check operator<<

BOOST_AUTO_TEST_SUITE_END()
