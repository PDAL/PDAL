/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include <boost/test/unit_test.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <pdal/Bounds.hpp>
#include <pdal/PDALUtils.hpp>
#include <iostream>
#include <sstream>
#include <string>

using namespace pdal;

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
    std::vector< Range<int> > rv;
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
    Bounds<int> r(1,2,3,9,8,7);
    BOOST_CHECK(r.volume() ==8*6*4);

    std::vector<int> shiftVec;
    shiftVec.push_back(10);
    shiftVec.push_back(11);
    shiftVec.push_back(12);
    r.shift(shiftVec);

    Bounds<int> r2(11, 13, 15, 19, 19, 19);
    BOOST_CHECK(r == r2);

    std::vector<int> scaleVec;
    scaleVec.push_back(2);
    scaleVec.push_back(3);
    scaleVec.push_back(4);
    r2.scale(scaleVec);

    Bounds<int> r3(22, 39, 60, 38, 57, 76);
    BOOST_CHECK(r2 == r3);
    BOOST_CHECK(r2.volume() == 16 * 18 * 16);
}

BOOST_AUTO_TEST_CASE(test_clip)
{
    Bounds<int> r1(0,0,10,10);
    Bounds<int> r2(1,1,11,11);
    r1.clip(r2);

    Bounds<int> r3(1,1,10,10);
    BOOST_CHECK(r1==r3);

    Bounds<int> r4(2,4,6,8);
    r1.clip(r4);

    BOOST_CHECK(r1==r4);

    Bounds<int> r5(20,40,60,80);
    r1.clip(r5);

    // BUG: seems wrong -- need to better define semantics of clip, etc
    // .clip() can make an invalid bounds, this should be fixed.
    Bounds<int> r6(20,6, 40,8);

    BOOST_CHECK(r1.getMinimum(0) == 20);
    BOOST_CHECK(r1.getMaximum(0) == 6);
    BOOST_CHECK(r1.getMinimum(1) == 40);
    BOOST_CHECK(r1.getMaximum(1) == 8);
}

BOOST_AUTO_TEST_CASE(test_intersect)
{
    Bounds<int> r1(0,0,10,10);
    Bounds<int> r2(1,1,11,11);
    Bounds<int> r3(100,100,101,101);
    Bounds<int> r4(2,4,6,8);

    BOOST_CHECK(r1.overlaps(r1));

    BOOST_CHECK(r1.overlaps(r2));
    BOOST_CHECK(r2.overlaps(r1));

    BOOST_CHECK(!r1.overlaps(r3));
    BOOST_CHECK(!r3.overlaps(r1));

    BOOST_CHECK(r1.contains(r1));
    BOOST_CHECK(!r1.contains(r2));
    BOOST_CHECK(r1.contains(r4));

    Vector<int> v1(5,6);
    Vector<int> v2(5,60);
    BOOST_CHECK(r1.contains(v1));
    BOOST_CHECK(!r1.contains(v2));
}


BOOST_AUTO_TEST_CASE(test_grow)
{
    Bounds<int> r1(50,51,100,101);
    Bounds<int> r2(0,1,10,201);

    r1.grow(r2);

    Bounds<int> r3(0,1,100,201);
    BOOST_CHECK(r1 == r3);


    Bounds<int> r4(0,1,10,11);
    Vector<int> ptLo(-1,-2);
    Vector<int> ptHi(20,201);
    r4.grow(ptLo);
    r4.grow(ptHi);
    Bounds<int> r5(-1,-2,20,201);
    BOOST_CHECK(r4 == r5);
}


BOOST_AUTO_TEST_CASE(test_static)
{
    Bounds<boost::uint8_t> t = Bounds<boost::uint8_t>::getDefaultSpatialExtent();

    std::vector<boost::uint8_t> minv;
    minv.push_back(0);
    minv.push_back(0);
    minv.push_back(0);
    std::vector<boost::uint8_t> maxv;
    maxv.push_back(255);
    maxv.push_back(255);
    maxv.push_back(255);

    BOOST_CHECK(t.getMinimum() == minv);
    BOOST_CHECK(t.getMaximum() == maxv);
}


BOOST_AUTO_TEST_CASE(test_output)
{
    const Bounds<int> b2(1,2,101,102);
    const Bounds<double> b3(1.1,2.2,3.3,101.1,102.2,103.3);

    std::stringstream ss2(std::stringstream::in | std::stringstream::out);
    std::stringstream ss3(std::stringstream::in | std::stringstream::out);

    ss2 << b2;
    ss3 << b3;

    const std::string out2 = ss2.str();
    const std::string out3 = ss3.str();

    BOOST_CHECK_EQUAL(out2, "([1, 101], [2, 102])");
    BOOST_CHECK_EQUAL(out3, "([1.1, 101.1], [2.2, 102.2], [3.3, 103.3])");
}


BOOST_AUTO_TEST_CASE(BoundsTest_ptree)
{
    const Bounds<int> b2(1,2,101,102);

    std::stringstream ss1(std::stringstream::in | std::stringstream::out);

    boost::property_tree::ptree tree = pdal::utils::toPTree(b2);
    boost::property_tree::write_xml(ss1, tree);

    const std::string out1 = ss1.str();

    static std::string xml_header = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
    const std::string ref = xml_header + "<0><minimum>1</minimum><maximum>101</maximum></0><1><minimum>2</minimum><maximum>102</maximum></1>";

    BOOST_CHECK_EQUAL(ref, out1);
}


BOOST_AUTO_TEST_CASE(test_input)
{
    std::stringstream ss("([1.1, 101.1], [2.2, 102.2], [3.3, 103.3])", std::stringstream::in | std::stringstream::out);

    Bounds<double> rr;
    ss >> rr;

    Bounds<double> r(1.1,2.2,3.3,101.1,102.2,103.3);
    BOOST_CHECK(r == rr);

    std::stringstream empty2_s("", std::stringstream::in | std::stringstream::out);

    Bounds<double> empty2;
    empty2_s >> empty2;
    BOOST_CHECK_EQUAL(true, empty2.empty());
}


BOOST_AUTO_TEST_CASE(test_lexicalcast_whitespace)
{
    const Bounds<double> b1 = boost::lexical_cast< Bounds<double> >("([1,101],[2,102],[3,103])");
    const Bounds<double> b2 = boost::lexical_cast< Bounds<double> >("([1, 101], [2, 102], [3, 103])");

    BOOST_CHECK_EQUAL(b1, b2);
}

BOOST_AUTO_TEST_CASE(test_wkt)
{
    const Bounds<double> b(1.1,2.2,3.3,101.1,102.2,103.3);
    BOOST_CHECK_EQUAL(b.toWKT(1), "POLYGON ((1.1 2.2, 1.1 102.2, 101.1 102.2, 101.1 2.2, 1.1 2.2))");
}

BOOST_AUTO_TEST_SUITE_END()
