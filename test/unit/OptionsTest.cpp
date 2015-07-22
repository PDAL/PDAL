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

#include <pdal/pdal_test_main.hpp>

#include <pdal/Options.hpp>
#include <pdal/PDALUtils.hpp>
#include <CropFilter.hpp>
#include <FauxReader.hpp>

#include "Support.hpp"

#include <boost/property_tree/xml_parser.hpp>

static std::string xml_header = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
static std::string xml_int_ref = "<Name>my_int</Name><Value>17</Value><Description>This is my integral option.</Description>";
static std::string xml_str_ref = "<Name>my_string</Name><Value>Yow.</Value><Description>This is my stringy option.</Description>";

using namespace pdal;

TEST(OptionsTest, test_static_options)
{
    Options ops;

    FauxReader reader;
    reader.setOptions(ops);

    CropFilter crop;
    crop.setOptions(ops);
    crop.setInput(reader);
    auto opts = crop.getDefaultOptions();
    EXPECT_EQ(opts.getOptions().size(), 3u);
    EXPECT_TRUE(opts.hasOption("bounds"));
    EXPECT_TRUE(opts.hasOption("inside"));
    EXPECT_TRUE(opts.hasOption("polygon"));
    EXPECT_FALSE(opts.hasOption("metes"));
}

TEST(OptionsTest, test_option_writing)
{
    std::ostringstream ostr_i;
    const std::string ref_i = xml_header + xml_int_ref;
    std::ostringstream ostr_s;
    const std::string ref_s = xml_header + xml_str_ref;

    const Option option_i("my_int", (uint16_t)17,
        "This is my integral option.");
    EXPECT_TRUE(option_i.getName() == "my_int");
    EXPECT_TRUE(option_i.getDescription() == "This is my integral option.");
    EXPECT_TRUE(option_i.getValue<uint16_t>() == 17);
    EXPECT_TRUE(option_i.getValue<std::string>() == "17");

    const Option option_s("my_string", "Yow.", "This is my stringy option.");
    EXPECT_TRUE(option_s.getName() == "my_string");
    EXPECT_TRUE(option_s.getDescription() == "This is my stringy option.");
    EXPECT_TRUE(option_s.getValue<std::string>() == "Yow.");
    EXPECT_TRUE(option_s.getValue<std::string>() == "Yow.");

    const boost::property_tree::ptree tree_i = Utils::toPTree(option_i);
    boost::property_tree::xml_parser::write_xml(ostr_i, tree_i);
    const std::string str_i = ostr_i.str();
    EXPECT_TRUE(str_i == ref_i);

    const boost::property_tree::ptree tree_s = Utils::toPTree(option_s);
    boost::property_tree::xml_parser::write_xml(ostr_s, tree_s);
    const std::string str_s = ostr_s.str();
    EXPECT_TRUE(str_s == ref_s);
}

TEST(OptionsTest, test_option_reading)
{
    // from an xml stream
    std::istringstream istr(xml_int_ref);
    boost::property_tree::ptree tree1;
    boost::property_tree::read_xml(istr,tree1);
    Option opt_from_istr(tree1);

    EXPECT_TRUE(opt_from_istr.getName() == "my_int");
    EXPECT_TRUE(opt_from_istr.getDescription() == "This is my integral option.");
    EXPECT_TRUE(opt_from_istr.getValue<std::string>() == "17");
    EXPECT_TRUE(opt_from_istr.getValue<int>() == 17);

    // from a ptree (assumed to be built correctly)
    const boost::property_tree::ptree tree2 = Utils::toPTree(opt_from_istr);
    Option opt_from_ptree(tree2);

    EXPECT_TRUE(opt_from_ptree.getName() == "my_int");
    EXPECT_TRUE(opt_from_ptree.getDescription() == "This is my integral option.");
    EXPECT_TRUE(opt_from_ptree.getValue<std::string>() == "17");
    EXPECT_TRUE(opt_from_ptree.getValue<int>() == 17);
}

TEST(OptionsTest, test_options_copy_ctor)
{
    Option opt_i("my_int", 17, "This is my integral option.");
    const Option opt_s("my_string", "Yow.", "This is my stringy option.");

    Options opts;
    opts.add(opt_i);
    opts.add(opt_s);

    Options copy(opts);

    opt_i.setOptions(copy);

    EXPECT_TRUE(copy.hasOption("my_int"));
    EXPECT_TRUE(copy.hasOption("my_string"));
}

TEST(OptionsTest, test_options_multi)
{
    Option opt_i("a", 1, "This is my integral option.");
    const Option opt_s("b", "2", "This is my stringy option.");

    Options opts;
    opts.add(opt_i);
    opts.add(opt_s);

    Option opt;
    opt.setOptions(opts);

    boost::optional<Options const&> o = opt.getOptions();

    Option const& i = o->getOption("a");
    EXPECT_EQ(i.getValue<int>(), 1);

    Option const& s = o->getOption("b");
    EXPECT_EQ(s.getValue<std::string>(), "2");
}

TEST(OptionsTest, test_options_writing)
{
    Options opts;

    const Option option_i("my_int", 17, "This is my integral option.");
    opts.add(option_i);

    opts.add("my_string", "Yow.", "This is my stringy option.");

    std::ostringstream ostr;
    const std::string ref = xml_header + "<Option>" + xml_int_ref + "</Option><Option>" + xml_str_ref + "</Option>";

    const boost::property_tree::ptree& tree = Utils::toPTree(opts);
    boost::property_tree::xml_parser::write_xml(ostr, tree);
    const std::string str = ostr.str();
    EXPECT_TRUE(str == ref);

    int val_i = opts.getOption("my_int").getValue<int>();
    std::string desc_i = opts.getOption("my_int").getDescription();
    std::string val_s = opts.getOption("my_string").getValue<std::string>();
    std::string desc_s = opts.getOption("my_string").getDescription();
    EXPECT_TRUE(val_i == 17);
    EXPECT_TRUE(val_s == "Yow.");
    EXPECT_TRUE(desc_i == "This is my integral option.");
    EXPECT_TRUE(desc_s == "This is my stringy option.");
}

TEST(OptionsTest, test_options_reading)
{
    const std::string ref = xml_header + "<Option>" + xml_int_ref + "</Option><Option>" + xml_str_ref + "</Option>";
    std::istringstream istr(ref);

    boost::property_tree::ptree tree;
    boost::property_tree::read_xml(istr,tree);
    Options opts_from_istr(tree);

    const Option& opt = opts_from_istr.getOption("my_int");

    EXPECT_TRUE(opt.getValue<std::string>() == "17");
    EXPECT_TRUE(opt.getValue<int>() == 17);
}

TEST(OptionsTest, test_valid_options)
{
    Options opts;

    bool reached = false;
    try
    {
        opts.getOption("foo").getValue<int>();
    }
    catch (Option::not_found ex)
    {
        EXPECT_EQ((std::string)ex.what(),
            (std::string)"Options::getOption: Required option 'foo' was "
            "not found on this stage");
        reached = true;
    }
    EXPECT_TRUE(reached);

    EXPECT_FALSE(opts.hasOption("bar"));
    {
        Options optI;

        optI.add("foo", 19, "foo as an int");
        EXPECT_TRUE(optI.hasOption("foo"));

        EXPECT_EQ(optI.getValueOrThrow<int>("foo"), 19);

        optI.add("foo", "nineteen", "foo as a string");
        EXPECT_TRUE(optI.hasOption("foo"));

        // Options is backed by a std::multimap,
        // Adding new options will mean the first will
        // continue to be returned.
        EXPECT_EQ(optI.getValueOrThrow<int>("foo"), 19);

        std::vector<Option> options = optI.getOptions("foo");
        EXPECT_EQ(options[1].getValue<std::string>(), "nineteen");
    }
}

TEST(OptionsTest, Options_test_add_vs_put)
{
    Options opts;

    opts.add<int>("a",1);
    opts.add<int>("a",2);
    opts.add<int>("a",3);

    std::vector<Option> options = opts.getOptions("a");
    EXPECT_TRUE(opts.hasOption("a"));
    EXPECT_EQ(options[0].getValue<int>(), 1);
    EXPECT_EQ(options[1].getValue<int>(), 2);
    EXPECT_EQ(options[2].getValue<int>(), 3);
}

TEST(OptionsTest, Options_test_bool)
{
    Option a("a", "true", "");
    Option b("b", "false", "");
    Option c("c", true);
    Option d("d", false);

    bool av = a.getValue<bool>();
    bool bv = b.getValue<bool>();
    bool cv = c.getValue<bool>();
    bool dv = d.getValue<bool>();

    EXPECT_EQ(av, true);
    EXPECT_EQ(bv, false);
    EXPECT_EQ(cv, true);
    EXPECT_EQ(dv, false);
}

TEST(OptionsTest, stringsplit)
{
    Option a("a", "This, is,a, test  ,,");
    std::vector<std::string> slist = a.getValue<std::vector<std::string>>();
    EXPECT_EQ(slist.size(), (size_t)4);
    EXPECT_EQ(slist[0], "This");
    EXPECT_EQ(slist[1], "is");
    EXPECT_EQ(slist[2], "a");
    EXPECT_EQ(slist[3], "test");
}

TEST(OptionsTest, implicitdefault)
{
    Options ops;
    ops.add("a", "This, is,a, test  ,,");
    ops.add("b", 25);

    int i = ops.getValueOrDefault<int>("c");
    EXPECT_EQ(i, 0);
    i = ops.getValueOrDefault<int>("b");
    EXPECT_EQ(i, 25);
    std::vector<std::string> slist =
        ops.getValueOrDefault<std::vector<std::string>>("d");
    EXPECT_EQ(slist.size(), (size_t)0);
    slist = ops.getValueOrDefault<std::vector<std::string>>("a");
    EXPECT_EQ(slist.size(), (size_t)4);
}

TEST(OptionsTest, metadata)
{
    Options ops;
    ops.add("test1", "This is a test");
    ops.add("test2", 56);
    ops.add("test3", 27.5, "Testing test3");

    Option op35("test3.5", 3.5);

    Options subops;
    subops.add("subtest1", "Subtest1");
    subops.add("subtest2", "Subtest2");

    op35.setOptions(subops);

    ops.add(op35);
    ops.add("test4", "Testing option test 4");

    MetadataNode node = ops.toMetadata();

    std::string goodfile(Support::datapath("misc/opts2json_meta.txt"));
    std::string testfile(Support::temppath("opts2json.txt"));
    {
        std::ofstream out(testfile);
        Utils::toJSON(node, out);
    }
    EXPECT_TRUE(Support::compare_files(goodfile, testfile));
}

TEST(OptionsTest, conditional)
{
    CropFilter s;

    Options ops;
    ops.add("foo", "foo");
    ops.add("bar", "bar");
    ops.add("baz", "baz");

    s.setOptions(ops);

    Options condOps;
    condOps.add("foo", "lose");
    condOps.add("bar", "lose");
    condOps.add("baz", "lose");
    condOps.add("foot", "win");
    condOps.add("barf", "win");
    condOps.add("bazel", "win");

    s.addConditionalOptions(condOps);
    ops = s.getOptions();
    EXPECT_EQ(ops.size(), 6u);
    EXPECT_EQ(ops.getValueOrDefault("foo", std::string()), "foo");
    EXPECT_EQ(ops.getValueOrDefault("bar", std::string()), "bar");
    EXPECT_EQ(ops.getValueOrDefault("baz", std::string()), "baz");
    EXPECT_EQ(ops.getValueOrDefault("foot", std::string()), "win");
    EXPECT_EQ(ops.getValueOrDefault("barf", std::string()), "win");
    EXPECT_EQ(ops.getValueOrDefault("bazel", std::string()), "win");
}

