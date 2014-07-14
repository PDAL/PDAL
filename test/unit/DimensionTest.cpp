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

#include <pdal/Dimension.hpp>
#include <pdal/Schema.hpp>

#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(DimensionTest)

BOOST_AUTO_TEST_CASE(test_ctor)
{
    Dimension d1("X", dimension::SignedInteger, 4);

    BOOST_CHECK_EQUAL(d1.getName(), "X");
    BOOST_CHECK_EQUAL(d1.getNamespace(), "");
    BOOST_CHECK_EQUAL(d1.getInterpretation(), dimension::SignedInteger);

    Dimension d2(d1);
    BOOST_CHECK_EQUAL(d1.getInterpretation(), d2.getInterpretation());
    BOOST_CHECK_EQUAL(d1.getNamespace(), d2.getNamespace());
    BOOST_CHECK_EQUAL(d1.getUUID(), d2.getUUID());
    BOOST_CHECK_EQUAL(d1.getParent(), d2.getParent());
    BOOST_CHECK_EQUAL(d1.getPosition(), d2.getPosition());

    Dimension d3 = d1;
    BOOST_CHECK_EQUAL(d1.getInterpretation(), d3.getInterpretation());
    BOOST_CHECK_EQUAL(d1.getNamespace(), d3.getNamespace());
    BOOST_CHECK_EQUAL(d1.getUUID(), d3.getUUID());
    BOOST_CHECK_EQUAL(d1.getParent(), d3.getParent());
    BOOST_CHECK_EQUAL(d1.getPosition(), d3.getPosition());

    BOOST_CHECK(d1 == d1);
    BOOST_CHECK(d1 == d2);
    BOOST_CHECK(d2 == d1);
    BOOST_CHECK(d1 == d3);
    BOOST_CHECK(d3 == d1);

    Dimension d4("Y", dimension::SignedInteger, 4);
    BOOST_CHECK(d1 != d4);
    BOOST_CHECK(d4 != d1);
}

BOOST_AUTO_TEST_CASE(test_parents)
{
    Dimension parent("X", dimension::SignedInteger, 4);
    parent.setNamespace("parent");
    parent.createUUID();

    Dimension child("X", dimension::Float, 4);
    child.setNamespace("a.child");
    child.createUUID();
    dimension::id const& p = parent.getUUID();
    child.setParent(p);

    Dimension child2("X", dimension::Float, 8);
    child2.setNamespace("a.child2");
    child2.createUUID();
    child2.setParent(child.getUUID());

    Dimension another("Y", dimension::SignedInteger, 4);

    Schema schema;
    schema.appendDimension(child);
    schema.appendDimension(parent);
    schema.appendDimension(another);
    schema.appendDimension(child2);

    BOOST_CHECK_EQUAL(child.getParent(), parent.getUUID());

    // The parent-child relationship will case getDimension
    // to return the child dim with name 'X', not the
    // first-added parent dim
    Dimension const& c2 = schema.getDimension("X");

    BOOST_CHECK_EQUAL(c2.getUUID(), child2.getUUID());
    
    Dimension const& d = schema.getDimension("a.child2.X");
    BOOST_CHECK_EQUAL(d.getNamespace(), "a.child2");
    BOOST_CHECK_EQUAL(d.getName(), "X");
}


BOOST_AUTO_TEST_CASE(DimensionTest_ptree)
{
    Dimension d1("X", dimension::SignedInteger, 4);

    boost::uuids::string_generator gen;
    std::string id("9bf8d966-0c0d-4c94-a14e-bce97e860bde");
    dimension::id uuid = gen(id);

    d1.setUUID(uuid);


    boost::property_tree::ptree tree = d1.toPTree();

    boost::uint32_t size = tree.get<boost::uint32_t>("bytesize");
    BOOST_CHECK_EQUAL(size, 4u);

    std::string name = tree.get<std::string>("name");
    BOOST_CHECK_EQUAL(name, "X");

    BOOST_CHECK_CLOSE(tree.get<double>("scale"), 1.0f, 0.00001f);
    BOOST_CHECK_CLOSE(tree.get<double>("offset"), 0.0f, 0.00001f);
    BOOST_CHECK_CLOSE(tree.get<double>("maximum"), 0.0f, 0.00001f);
    BOOST_CHECK_CLOSE(tree.get<double>("minimum"), 0.0f, 0.00001f);

    BOOST_CHECK_EQUAL(tree.get<std::string>("namespace"), "");

    BOOST_CHECK_EQUAL(tree.get<std::string>("uuid"), "9bf8d966-0c0d-4c94-a14e-bce97e860bde");
}



BOOST_AUTO_TEST_CASE(DimensionTest_Interpretation)
{
    Dimension x("X", dimension::SignedInteger, 4);
    
    BOOST_CHECK_EQUAL(x.getInterpretationName(), "int32_t");
    dimension::Interpretation interp = Dimension::getInterpretation(x.getInterpretationName());
    
    BOOST_CHECK_EQUAL(interp, pdal::dimension::SignedInteger);
}


BOOST_AUTO_TEST_SUITE_END()

