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

#include <sstream>

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

    v.set(1, 123);
    BOOST_CHECK(v.get(1)==123);
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

BOOST_AUTO_TEST_CASE(test_dump)
{
    Vector<int> v(1,2,3);
  
    std::ostringstream s;
    s << v;

    BOOST_CHECK(s.str() == "(1, 2, 3)");
    return;
}

BOOST_AUTO_TEST_SUITE_END()
