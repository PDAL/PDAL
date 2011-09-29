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
#include <boost/cstdint.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <pdal/SchemaLayout.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(SchemaLayoutTest)

BOOST_AUTO_TEST_CASE(test_layout)
{
    Dimension d1(DimensionId::X_i32);
    Dimension d2(DimensionId::Y_i32);

    Schema s1;
    s1.appendDimension(d1);
    s1.appendDimension(d2);
    BOOST_CHECK(s1.getDimensions()[0].getId() == DimensionId::X_i32);
    BOOST_CHECK(s1.getDimensions()[1].getId() == DimensionId::Y_i32);

    BOOST_CHECK(s1.hasDimension(DimensionId::X_f64) == false);
    BOOST_CHECK(s1.hasDimension(DimensionId::Y_f64) == false);

    Schema s2;
    s2.appendDimension(d1);
    BOOST_CHECK(s2.hasDimension(DimensionId::X_i32) == true);
    BOOST_CHECK(s2.getDimensions()[0].getId() == DimensionId::X_i32);
    BOOST_CHECK(s2.hasDimension(DimensionId::Y_i32) == false);

    SchemaLayout l1(s1);
    SchemaLayout l2(l1);
    SchemaLayout l3 = l1;
    SchemaLayout l4(s2);

    BOOST_CHECK(l1==l1);
    BOOST_CHECK(l1==l2);
    BOOST_CHECK(l2==l1);
    BOOST_CHECK(l1==l3);
    BOOST_CHECK(l3==l1);
    BOOST_CHECK(l1!=l4);
    BOOST_CHECK(l4!=l1);
}


BOOST_AUTO_TEST_CASE(test_layout_size)
{
    Dimension d1(DimensionId::X_i32);
    Dimension d2(DimensionId::Y_i32);
    Schema s1;
    s1.appendDimension(d1);
    s1.appendDimension(d2);
    SchemaLayout sl1(s1);

    const DimensionLayout& dl1 = sl1.getDimensionLayout(0);
    BOOST_CHECK(dl1.getDimension() == d1);
    BOOST_CHECK(dl1.getPosition() == 0);
    BOOST_CHECK(dl1.getByteOffset() == 0);

    const DimensionLayout& dl2 = sl1.getDimensionLayout(1);
    BOOST_CHECK(dl2.getDimension() == d2);
    BOOST_CHECK(dl2.getPosition() == 1);
    BOOST_CHECK(dl2.getByteOffset() == 4);
}


BOOST_AUTO_TEST_CASE(SchemaLayoutTest_ptree)
{
    Dimension d1(DimensionId::X_i32);
    Dimension d2(DimensionId::Y_i32);
    Schema s1;
    s1.appendDimension(d1);
    s1.appendDimension(d2);
    SchemaLayout sl1(s1);

    std::stringstream ss1(std::stringstream::in | std::stringstream::out);
  
    boost::property_tree::ptree tree = sl1.toPTree();
    boost::property_tree::write_xml(ss1, tree);

    std::string out1 = ss1.str();

    static std::string xml_header = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
    std::string ref = xml_header + "<dimensionlayout>"
        "<dimension><name>X</name><datatype>Int32</datatype>"
        "<description>x coordinate as a long integer. You must use the scale and offset information of the header to determine the double value.</description>"
        "<bytesize>4</bytesize><endianness>little</endianness><scale>0</scale><isValid>false</isValid></dimension><byteoffset>0</byteoffset><position>0</position>"
        "</dimensionlayout>"
        "<dimensionlayout><dimension><name>Y</name><datatype>Int32</datatype>"
        "<description>y coordinate as a long integer. You must use the scale and offset information of the header to determine the double value.</description>"
        "<bytesize>4</bytesize><endianness>little</endianness><scale>0</scale><isValid>false</isValid></dimension><byteoffset>4</byteoffset><position>1</position></dimensionlayout>";

    boost::algorithm::erase_all(out1, "\n");
    boost::algorithm::erase_all(ref, "\n");
    BOOST_CHECK_EQUAL(ref, out1);

    return;
}


BOOST_AUTO_TEST_SUITE_END()
