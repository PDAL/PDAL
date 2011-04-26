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

#include <libpc/Bounds.hpp>
#include <iostream>
#include <sstream>
#include <string>

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
    Bounds<int> r6(20,40,6,8);
    BOOST_CHECK(r1==r6);
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

    return;
}

BOOST_AUTO_TEST_CASE(test_dump)
{
    Bounds<int> r(1,2,3,7,8,9);
  
    std::ostringstream s;
    s << r;

    BOOST_CHECK(s.str() == "([1 .. 7], [2 .. 8], [3 .. 9])");
    return;
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
    Bounds<double> b2(1,2,101,102);
    Bounds<double> b3(1,2,3,101,102,103);
    
    std::stringstream oss;
    oss << b2;

    std::string b2s = oss.str();

    BOOST_CHECK(b2s == "([1 , 101], [2, 102])");

    return;
}


BOOST_AUTO_TEST_CASE(test_input)
{
    Bounds<int> r(1,2,10,20);
  
    std::stringstream iss;
    iss << "[10 .. 20]";

    Bounds<int> rr;
    iss >> rr;

    BOOST_CHECK(r == rr);

    return;
}

BOOST_AUTO_TEST_SUITE_END()
