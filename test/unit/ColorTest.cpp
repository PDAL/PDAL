#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include "libpc/Color.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(ColorTest)

BOOST_AUTO_TEST_CASE(test_ctor)
{
    Color c0;
    Color c1(1,2,3);
    boost::array<boost::uint16_t,3> a = {1,2,3};
    Color c2(a);
    Color c3(c2);
    Color c4 = c3;

    BOOST_CHECK(c2!=c0);
    BOOST_CHECK(c2==c1);
    BOOST_CHECK(c2==c2);
    BOOST_CHECK(c3==c2);
    BOOST_CHECK(c4==c2);
}

BOOST_AUTO_TEST_CASE(test_accessors)
{
  Color c0;
  BOOST_CHECK(c0[0]==0);
  BOOST_CHECK(c0[1]==0);
  BOOST_CHECK(c0[2]==0);

  Color c1(1,2,3);
  BOOST_CHECK(c1[0]==1);
  BOOST_CHECK(c1[1]==2);
  BOOST_CHECK(c1[2]==3);
  
  BOOST_CHECK(c1.getRed()==1);
  BOOST_CHECK(c1.getGreen()==2);
  BOOST_CHECK(c1.getBlue()==3);

  c0.setRed(1);
  c0.setGreen(2);
  c0.setBlue(3);

  BOOST_CHECK(c0 == c1);
}


// BUG: interpolate function not tested

BOOST_AUTO_TEST_SUITE_END()
