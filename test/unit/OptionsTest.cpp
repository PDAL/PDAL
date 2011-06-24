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
#include <iostream>

#include <boost/property_tree/xml_parser.hpp>

#include <boost/test/unit_test.hpp>

#include <pdal/Options.hpp>

BOOST_AUTO_TEST_SUITE(OptionsTest)

static std::string xml_header = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
static std::string xml_int_ref = "<name>my_int</name><description>This is my integral option.</description><value>17</value>";
static std::string xml_str_ref = "<name>my_string</name><description>This is my stringy option.</description><value>Yow.</value>";

BOOST_AUTO_TEST_CASE(test_option)
{
    std::ostringstream ostr_i;
    const std::string ref_i = xml_header + xml_int_ref;
    std::ostringstream ostr_s;
    const std::string ref_s = xml_header + xml_str_ref;

    const pdal::OptionNew<int> option_i("my_int", 17, "This is my integral option.");
    BOOST_CHECK(option_i.getName() == "my_int");
    BOOST_CHECK(option_i.getDescription() == "This is my integral option.");
    BOOST_CHECK(option_i.getValue() == 17);

    const pdal::OptionNew<std::string> option_s("my_string", "Yow.", "This is my stringy option.");
    BOOST_CHECK(option_s.getName() == "my_string");
    BOOST_CHECK(option_s.getDescription() == "This is my stringy option.");
    BOOST_CHECK(option_s.getValue() == "Yow.");

    const boost::property_tree::ptree tree_i = option_i.getPTree();
    boost::property_tree::xml_parser::write_xml(ostr_i, tree_i);
    const std::string str_i = ostr_i.str();
    BOOST_CHECK(str_i == ref_i);
   
    const boost::property_tree::ptree tree_s = option_s.getPTree();
    boost::property_tree::xml_parser::write_xml(ostr_s, tree_s);
    const std::string str_s = ostr_s.str();
    BOOST_CHECK(str_s == ref_s);

    return;
}

BOOST_AUTO_TEST_CASE(test_options)
{
    pdal::OptionsNew opts;

    const pdal::OptionNew<int> option_i("my_int", 17, "This is my integral option.");
    opts.add(option_i);

    opts.add("my_string", "Yow.", "This is my stringy option.");

    std::ostringstream ostr;
    const std::string ref = xml_header + "<option>" + xml_int_ref + "</option><option>" + xml_str_ref + "</option>";

    const boost::property_tree::ptree& tree = opts.getPTree();
    boost::property_tree::xml_parser::write_xml(ostr, tree);
    const std::string str = ostr.str();
    BOOST_CHECK(str == ref);

    int val_i = opts.getValue<int>("my_int");
    std::string desc_i = opts.getDescription("my_int");
    std::string val_s = opts.getValue<std::string>("my_string");
    std::string desc_s = opts.getDescription("my_string");
    BOOST_CHECK(val_i == 17);
    BOOST_CHECK(val_s == "Yow.");
    BOOST_CHECK(desc_i == "This is my integral option.");
    BOOST_CHECK(desc_s == "This is my stringy option.");

    return;
}

BOOST_AUTO_TEST_SUITE_END()
