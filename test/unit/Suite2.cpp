#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(Suite2)


char* p;
BOOST_AUTO_TEST_CASE(Suite_2_Test_1_memleak)
{
    p = new char[1000];
    //delete[] p;
}


BOOST_AUTO_TEST_CASE(Suite_2_Test_2_assertfalse)
{
    BOOST_CHECK(false);
}


BOOST_AUTO_TEST_CASE(Suite_2_Test_3_uncaughtthrow)
{
    throw 9;
}


BOOST_AUTO_TEST_SUITE_END()
 