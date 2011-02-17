#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(Suite1)

BOOST_AUTO_TEST_CASE(Suite_1_Test_1)
{
    BOOST_TEST_MESSAGE("(hello from Suite_1_Test_1)");

    BOOST_CHECK(true);
}

BOOST_AUTO_TEST_CASE(Suite_1_Test_2)
{
    BOOST_CHECK(true);
}

BOOST_AUTO_TEST_SUITE_END()
