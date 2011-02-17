#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(Suite2)

BOOST_AUTO_TEST_CASE(Suite_2_Test_1)
{
    BOOST_CHECK(true);
}

BOOST_AUTO_TEST_CASE(Suite_2_Test_2)
{
    BOOST_CHECK(false);
}

BOOST_AUTO_TEST_SUITE_END()
