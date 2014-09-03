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
#include <sstream>
#include <iostream>
#include <string>

#include <pdal/Options.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/filters/Crop.hpp>
#include <pdal/drivers/faux/Reader.hpp>

#include <boost/property_tree/xml_parser.hpp>


BOOST_AUTO_TEST_SUITE(OptionsTest)

static std::string xml_header = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
static std::string xml_int_ref = "<Name>my_int</Name><Value>17</Value><Description>This is my integral option.</Description>";
static std::string xml_str_ref = "<Name>my_string</Name><Value>Yow.</Value><Description>This is my stringy option.</Description>";


BOOST_AUTO_TEST_CASE(test_static_options)
{
    using namespace pdal;

    Options ops;
    drivers::faux::Reader reader(ops);
    filters::Crop crop(ops);
    crop.setInput(&reader);
    const Options& opts = crop.getDefaultOptions();

    BOOST_CHECK(opts.hasOption("bounds"));
    BOOST_CHECK(!opts.hasOption("metes"));
    const boost::property_tree::ptree& pt = pdal::utils::toPTree(opts);
    BOOST_CHECK(pt.size() == 3);
}


BOOST_AUTO_TEST_CASE(test_option_writing)
{
    std::ostringstream ostr_i;
    const std::string ref_i = xml_header + xml_int_ref;
    std::ostringstream ostr_s;
    const std::string ref_s = xml_header + xml_str_ref;

    const pdal::Option option_i("my_int", (boost::uint16_t)17, "This is my integral option.");
    BOOST_CHECK(option_i.getName() == "my_int");
    BOOST_CHECK(option_i.getDescription() == "This is my integral option.");
    BOOST_CHECK(option_i.getValue<boost::uint16_t>() == 17);
    BOOST_CHECK(option_i.getValue<std::string>() == "17");

    const pdal::Option option_s("my_string", "Yow.", "This is my stringy option.");
    BOOST_CHECK(option_s.getName() == "my_string");
    BOOST_CHECK(option_s.getDescription() == "This is my stringy option.");
    BOOST_CHECK(option_s.getValue<std::string>() == "Yow.");
    BOOST_CHECK(option_s.getValue<std::string>() == "Yow.");

    const boost::property_tree::ptree tree_i = pdal::utils::toPTree(option_i);
    boost::property_tree::xml_parser::write_xml(ostr_i, tree_i);
    const std::string str_i = ostr_i.str();
    BOOST_CHECK(str_i == ref_i);

    const boost::property_tree::ptree tree_s = pdal::utils::toPTree(option_s);
    boost::property_tree::xml_parser::write_xml(ostr_s, tree_s);
    const std::string str_s = ostr_s.str();
    BOOST_CHECK(str_s == ref_s);
}


BOOST_AUTO_TEST_CASE(test_option_reading)
{
    // from an xml stream
    std::istringstream istr(xml_int_ref);
    boost::property_tree::ptree tree1;
    boost::property_tree::read_xml(istr,tree1);
    pdal::Option opt_from_istr(tree1);

    BOOST_CHECK(opt_from_istr.getName() == "my_int");
    BOOST_CHECK(opt_from_istr.getDescription() == "This is my integral option.");
    BOOST_CHECK(opt_from_istr.getValue<std::string>() == "17");
    BOOST_CHECK(opt_from_istr.getValue<int>() == 17);

    // from a ptree (assumed to be built correctly)
    const boost::property_tree::ptree tree2 = pdal::utils::toPTree(opt_from_istr);
    pdal::Option opt_from_ptree(tree2);

    BOOST_CHECK(opt_from_ptree.getName() == "my_int");
    BOOST_CHECK(opt_from_ptree.getDescription() == "This is my integral option.");
    BOOST_CHECK(opt_from_ptree.getValue<std::string>() == "17");
    BOOST_CHECK(opt_from_ptree.getValue<int>() == 17);
}


BOOST_AUTO_TEST_CASE(test_options_copy_ctor)
{
    pdal::Option opt_i("my_int", 17, "This is my integral option.");
    const pdal::Option opt_s("my_string", "Yow.", "This is my stringy option.");

    pdal::Options opts;
    opts.add(opt_i);
    opts.add(opt_s);

    pdal::Options copy(opts);

    opt_i.setOptions(copy);

    BOOST_CHECK(copy.hasOption("my_int"));
    BOOST_CHECK(copy.hasOption("my_string"));
}

BOOST_AUTO_TEST_CASE(test_options_multi)
{
    pdal::Option opt_i("a", 1, "This is my integral option.");
    const pdal::Option opt_s("b", "2", "This is my stringy option.");

    pdal::Options opts;
    opts.add(opt_i);
    opts.add(opt_s);

    pdal::Option opt;
    opt.setOptions(opts);

    boost::optional<pdal::Options const&> o = opt.getOptions();

    pdal::Option const& i = o->getOption("a");
    BOOST_CHECK_EQUAL(i.getValue<int>(), 1);

    pdal::Option const& s = o->getOption("b");
    BOOST_CHECK_EQUAL(s.getValue<std::string>(), "2");
}

BOOST_AUTO_TEST_CASE(test_options_writing)
{
    pdal::Options opts;

    const pdal::Option option_i("my_int", 17, "This is my integral option.");
    opts.add(option_i);

    opts.add("my_string", "Yow.", "This is my stringy option.");

    std::ostringstream ostr;
    const std::string ref = xml_header + "<Option>" + xml_int_ref + "</Option><Option>" + xml_str_ref + "</Option>";

    const boost::property_tree::ptree& tree = pdal::utils::toPTree(opts);
    boost::property_tree::xml_parser::write_xml(ostr, tree);
    const std::string str = ostr.str();
    BOOST_CHECK(str == ref);

    int val_i = opts.getOption("my_int").getValue<int>();
    std::string desc_i = opts.getOption("my_int").getDescription();
    std::string val_s = opts.getOption("my_string").getValue<std::string>();
    std::string desc_s = opts.getOption("my_string").getDescription();
    BOOST_CHECK(val_i == 17);
    BOOST_CHECK(val_s == "Yow.");
    BOOST_CHECK(desc_i == "This is my integral option.");
    BOOST_CHECK(desc_s == "This is my stringy option.");
}


BOOST_AUTO_TEST_CASE(test_options_reading)
{
    const std::string ref = xml_header + "<Option>" + xml_int_ref + "</Option><Option>" + xml_str_ref + "</Option>";
    std::istringstream istr(ref);

    boost::property_tree::ptree tree;
    boost::property_tree::read_xml(istr,tree);
    pdal::Options opts_from_istr(tree);

    const pdal::Option& opt = opts_from_istr.getOption("my_int");

    BOOST_CHECK(opt.getValue<std::string>() == "17");
    BOOST_CHECK(opt.getValue<int>() == 17);
}


BOOST_AUTO_TEST_CASE(test_valid_options)
{
    pdal::Options opts;

    bool reached = false;
    try
    {
        opts.getOption("foo").getValue<int>();
        reached = false;
    }
    catch (pdal::option_not_found ex)
    {
        BOOST_CHECK(strcmp(ex.what(), "Options::getOption: Required option 'foo' was not found on this stage") == 0);
        reached = true;
    }
    BOOST_CHECK(reached == true);

    bool ok = opts.hasOption("bar");
    BOOST_CHECK(!ok);

    {
        pdal::Options optI;

        optI.add("foo", 19, "foo as an int");
        ok = optI.hasOption("foo");
        BOOST_CHECK(ok);

        const int i1 = optI.getValueOrThrow<int>("foo");
        BOOST_CHECK(i1 == 19);

        optI.add("foo", "nineteen", "foo as a string");
        ok = optI.hasOption("foo");
        BOOST_CHECK(ok);


        // Options is backed by a std::multimap,
        // Adding new options will mean the first will
        // continue to be returned.
        const int i2 = optI.getValueOrThrow<int>("foo");
        BOOST_CHECK(i2 == 19);

        std::vector<pdal::Option> options = optI.getOptions("foo");

        BOOST_CHECK(options[1].getValue<std::string>() == "nineteen");
    }
}


BOOST_AUTO_TEST_CASE(Options_test_add_vs_put)
{
    pdal::Options opts;

    opts.add<int>("a",1);
    opts.add<int>("a",2);
    opts.add<int>("a",3);

    std::vector<pdal::Option> options = opts.getOptions("a");
    BOOST_CHECK(opts.hasOption("a"));
    BOOST_CHECK_EQUAL(options[0].getValue<int>(), 1);
    BOOST_CHECK_EQUAL(options[1].getValue<int>(), 2);
    BOOST_CHECK_EQUAL(options[2].getValue<int>(), 3);
}


BOOST_AUTO_TEST_CASE(Options_test_bool)
{
    pdal::Option a("a","true", "");
    pdal::Option b("b","false", "");
    pdal::Option c("c",true);
    pdal::Option d("d",false);

    bool av = a.getValue<bool>();
    bool bv = b.getValue<bool>();
    bool cv = c.getValue<bool>();
    bool dv = d.getValue<bool>();

    BOOST_CHECK_EQUAL(av, true);
    BOOST_CHECK_EQUAL(bv, false);
    BOOST_CHECK_EQUAL(cv, true);
    BOOST_CHECK_EQUAL(dv, false);
}


BOOST_AUTO_TEST_SUITE_END()
