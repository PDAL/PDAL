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

#include "gtest/gtest.h"

#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PDALUtils.hpp>

#include <boost/property_tree/xml_parser.hpp>

static std::string xml_header = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
static std::string xml_int_ref = "<Name>my_int</Name><Value>17</Value><Description>This is my integral option.</Description>";
static std::string xml_str_ref = "<Name>my_string</Name><Value>Yow.</Value><Description>This is my stringy option.</Description>";

using namespace pdal;

namespace
{
    using namespace std;

    static bool hasOption(vector<Option> const& opts, string const& name)
    {
        bool found = false;
        for (auto o : opts)
            if (o.getName() == name)
                found = true;
        return found;
    }
}

TEST(OptionsTest, test_static_options)
{
    Options ops;

    StageFactory f;
    std::unique_ptr<Reader> reader(f.createReader("readers.faux"));
    EXPECT_TRUE(reader.get());
    reader->setOptions(ops);
    std::unique_ptr<Filter> crop(f.createFilter("filters.crop"));
    EXPECT_TRUE(crop.get());
    crop->setOptions(ops);
    crop->setInput(reader.get());
    std::map<std::string, pdal::StageInfo> const& drivers = f.getStageInfos();
    typedef std::map<std::string, pdal::StageInfo>::const_iterator Iterator;
    Iterator i = drivers.find("filters.crop");
    if (i != drivers.end())
    {
      const std::vector<Option> opts = i->second.getProvidedOptions();
      EXPECT_EQ(opts.size(), 3u);
      EXPECT_TRUE(hasOption(opts, "bounds"));
      EXPECT_TRUE(hasOption(opts, "inside"));
      EXPECT_TRUE(hasOption(opts, "polygon"));
      EXPECT_FALSE(hasOption(opts, "metes"));
    }
}

TEST(OptionsTest, test_option_writing)
{
    std::ostringstream ostr_i;
    const std::string ref_i = xml_header + xml_int_ref;
    std::ostringstream ostr_s;
    const std::string ref_s = xml_header + xml_str_ref;

    const Option option_i("my_int", (uint16_t)17, "This is my integral option.");
    EXPECT_TRUE(option_i.getName() == "my_int");
    EXPECT_TRUE(option_i.getDescription() == "This is my integral option.");
    EXPECT_TRUE(option_i.getValue<boost::uint16_t>() == 17);
    EXPECT_TRUE(option_i.getValue<std::string>() == "17");

    const Option option_s("my_string", "Yow.", "This is my stringy option.");
    EXPECT_TRUE(option_s.getName() == "my_string");
    EXPECT_TRUE(option_s.getDescription() == "This is my stringy option.");
    EXPECT_TRUE(option_s.getValue<std::string>() == "Yow.");
    EXPECT_TRUE(option_s.getValue<std::string>() == "Yow.");

    const boost::property_tree::ptree tree_i = utils::toPTree(option_i);
    boost::property_tree::xml_parser::write_xml(ostr_i, tree_i);
    const std::string str_i = ostr_i.str();
    EXPECT_TRUE(str_i == ref_i);

    const boost::property_tree::ptree tree_s = utils::toPTree(option_s);
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
    const boost::property_tree::ptree tree2 = utils::toPTree(opt_from_istr);
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

    const boost::property_tree::ptree& tree = utils::toPTree(opts);
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
        reached = false;
    }
    catch (option_not_found ex)
    {
        EXPECT_TRUE(strcmp(ex.what(), "Options::getOption: Required option 'foo' was not found on this stage") == 0);
        reached = true;
    }
    EXPECT_TRUE(reached == true);

    bool ok = opts.hasOption("bar");
    EXPECT_TRUE(!ok);

    {
        Options optI;

        optI.add("foo", 19, "foo as an int");
        ok = optI.hasOption("foo");
        EXPECT_TRUE(ok);

        const int i1 = optI.getValueOrThrow<int>("foo");
        EXPECT_TRUE(i1 == 19);

        optI.add("foo", "nineteen", "foo as a string");
        ok = optI.hasOption("foo");
        EXPECT_TRUE(ok);

        // Options is backed by a std::multimap,
        // Adding new options will mean the first will
        // continue to be returned.
        const int i2 = optI.getValueOrThrow<int>("foo");
        EXPECT_TRUE(i2 == 19);

        std::vector<Option> options = optI.getOptions("foo");

        EXPECT_TRUE(options[1].getValue<std::string>() == "nineteen");
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
    Option a("a","true", "");
    Option b("b","false", "");
    Option c("c",true);
    Option d("d",false);

    bool av = a.getValue<bool>();
    bool bv = b.getValue<bool>();
    bool cv = c.getValue<bool>();
    bool dv = d.getValue<bool>();

    EXPECT_EQ(av, true);
    EXPECT_EQ(bv, false);
    EXPECT_EQ(cv, true);
    EXPECT_EQ(dv, false);
}
