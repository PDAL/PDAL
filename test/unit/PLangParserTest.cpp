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

#include <pdal/plang/Parser.hpp>

BOOST_AUTO_TEST_SUITE(PLangParserTest)


using pdal::plang::variant_t;
using pdal::plang::Parser;


//---------------------------------------------------------------------------


struct SymItem
{
    SymItem(std::string n, variant_t v) : name(n), value(v) {}
    std::string name;
    variant_t value;
};
typedef std::vector<SymItem> SymList;

static void check_generic(const std::string& text, const SymList& in, const SymList& out)
{
    Parser parser(text);

    bool r = parser.parse();
    assert(r);

    for (SymList::const_iterator iter=in.begin(); iter!=in.end(); iter++)
    {
        const std::string name = iter->name;
        const variant_t value = iter->value;
        parser.setVariableV(name, value);
    }

    bool ok = parser.evaluate();
    assert(ok);

    for (SymList::const_iterator iter=out.begin(); iter!=out.end(); iter++)
    {
        const std::string name = iter->name;
        const variant_t value = iter->value;

        variant_t result = parser.getVariableV(name);
        assert(result == value);
    }

    return;
}


static void check_in1_out1(const std::string& text, 
                           const std::string& in1_name, variant_t in1_value, 
                           const std::string& out1_name, variant_t out1_value)
{
    SymList in;
    in.push_back(SymItem(in1_name, in1_value));
    SymList out;
    out.push_back(SymItem(out1_name, out1_value));

    check_generic(text, in, out);

    return;
}


static void check_in2_out2(const std::string& text, 
                           const std::string& in1_name, variant_t in1_value, 
                           const std::string& in2_name, variant_t in2_value, 
                           const std::string& out1_name, variant_t out1_value,
                           const std::string& out2_name, variant_t out2_value)
{
    SymList in;
    in.push_back(SymItem(in1_name, in1_value));
    in.push_back(SymItem(in2_name, in2_value));
    SymList out;
    out.push_back(SymItem(out1_name, out1_value));
    out.push_back(SymItem(out2_name, out2_value));

    check_generic(text, in, out);

    return;
}


static void check_out1(const std::string& text, 
                       const std::string& out1_name, variant_t out1_value)
{
    SymList in1;
    SymList out1;
    out1.push_back(SymItem(out1_name, out1_value));

    check_generic(text, in1, out1);

    return;
}


//---------------------------------------------------------------------------


BOOST_AUTO_TEST_CASE(PLangParserTest_test1)
{
    check_out1("int32 x ; x = 1;", 
              "x", 1);
    
    check_out1("int32 x ; x = 1; x = 2;", 
              "x", 2);

    check_in1_out1("int32 x ; x = x + 1; ", 
                   "x", (boost::int32_t)88, 
                   "x", (boost::int32_t)89);

    check_in1_out1("uint8 x ; x = x + uint8(1); x = x - uint8(2);", 
                   "x", (boost::uint8_t)88, 
                   "x", (boost::uint8_t)87);

    check_in2_out2("uint8 x ; uint8 y; x = x + uint8(1); y = y + uint8(2);", 
        "x", (boost::uint8_t)11,
        "y", (boost::uint8_t)22,
        "x", (boost::uint8_t)12,
        "y", (boost::uint8_t)24);

    check_out1("int32 x; x=0;", "x", 0);
    check_out1("bool b; b = true;", "b", true);
    check_out1("bool b; b=false;", "b", false);

    check_out1("int32 x; x = 9;", "x", 9);
    check_out1("int32 x_y; x_y = 9;", "x_y", 9);
    check_out1("int32 x_; x_ = 9;", "x_", 9);
    check_out1("int32 x1; x1 = 9;", "x1", 9);
    check_out1("int32 _x; _x = 9;", "_x", 9);
    check_out1("int32 _; _ = 9;", "_", 9);
    check_out1("int32 _1_2_; _1_2_ = 9;", "_1_2_", 9);
    check_out1("int32 __; __ = 9;", "__", 9);

    check_in2_out2("int32 x; int32 X; int32 swap; swap = x; x = X; X = swap;", "x", 9, "X", 90, "x", 90, "X", 9);

    check_out1("float32 e; e = float32(0xff);", "e", 255.0f);

    check_out1("bool b; b = 5 < 8;", "b", true);
    check_out1("bool b; b = 5 <= 8;", "b", true);
    check_out1("bool b; b = 5 > 8;", "b", false);
    check_out1("bool b; b = 5 >= 8;", "b", false);
    check_out1("bool b; b = 5 == 8;", "b", false);
    check_out1("bool b; b = 5 == 5;", "b", true);

    check_out1("uint32 x; x = 0x0f ^ 0xf0;", "x", (unsigned(0x0f ^ 0xf0)));
    check_out1("uint32 x; x = 0x0f & 0xf0;", "x", (unsigned(0x0f & 0xf0)));
    check_out1("uint32 x; x = 0x0f | 0xf0;", "x", (unsigned(0x0f | 0xf0)));
    check_out1("bool x; x = true && true;", "x", true);
    check_out1("bool x; x = true && false;", "x", false);
    check_out1("bool x; x = true || true;", "x", true);
    check_out1("bool x; x = false || false;", "x", false);

    check_out1("bool b; b = 2+3*4 == 2+(3*4);", "b", true);
    check_out1("bool b; b = 2+3*4 != (2+3)*4;", "b", true);

    check_out1("float64 d; d = 6.7 + 1.2 * float64( 7 - 3 );", "d", 6.7+1.2*4);
    
    check_out1("float64 d; d = float64( 7 - 3 );", "d", 4.0);
    check_out1("float64 d; d = float64(int8(8+3));", "d", 11.0);
    
    check_out1("float64 d; d = float64( 7 - 3 ) / float64(11);", "d", ((7.0-3.0)/11.0));  // 0.363636
    check_out1("float64 d; d = float64( 7 - 3 ) / float64(int8(8+3));", "d", ((7.0-3.0)/double(8+3)));  // 0.363636
    
    check_out1("int32 r; r = 1 + 2 + 3;", "r", 6);
#if notdef // See Parser_qi
    check_out1("float64 d; d = float64(1) + float64(2u) + float64(3ul) + float64(4lu) + float64(5l) + 6.0 + float64(7.0f) + float64(0x88) + float64(0x0888l);",
               "d", (1.0 + 2.0 + 3.0 + 4.0 + 5.0 + 6.0 + 7.0 + (double)0x88 + (double)0x0888l));
#endif
    check_out1("float64 d; d = 6.7 + 1.2 * float64( 7 - 3 ) / float64(int8(8+3));", 
              "d", (6.7 + 1.2 * 4.0 / 11.0));

    check_in2_out2("float64 x; float64 y8; x= 6.7 + x + y8;",
                   "x", 1.2, "y8", 3.4,
                   "x", (6.7+1.2+3.4), "y8", 3.4);

    return;
}


BOOST_AUTO_TEST_CASE(PLangParserTest_demo)
{
    const std::string program = 
        "int8 degrees; "
         "float32 radians; "
         "float32 TO_DEGREES; "
         "float32 temp; "
         "TO_DEGREES = 180.0f/3.14159f;"
         "temp = radians * TO_DEGREES;"
         "degrees = int8(temp);";

    Parser parser(program);

    bool ok = parser.parse();
    assert(ok);

    const float TO_RADIANS = 3.14159f/180.0f;
    const float TO_DEGREES = 180.0f/3.14159f;
    const float point_radians[3] = { 120.0f*TO_RADIANS, 60.0f*TO_RADIANS, 20.0f*TO_RADIANS };

    for (int i=0; i<3; i++)
    {
        parser.setVariable("radians", point_radians[i]);

        ok = parser.evaluate();
        assert(ok);
        
        const boost::int8_t deg = parser.getVariable<boost::int8_t>("degrees");
        assert(deg == int(point_radians[i] * TO_DEGREES));
    }

    return;
}

BOOST_AUTO_TEST_SUITE_END()
