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

namespace
{
const std::string xml_header = "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
const std::string xml_int_ref = "<Name>my_int</Name><Value>17</Value><Description>This is my integral option.</Description>";
const std::string xml_str_ref = "<Name>my_string</Name><Value>Yow.</Value><Description>This is my stringy option.</Description>";
}

namespace pdal
{

TEST(OptionsTest, test_option_writing)
{
    std::ostringstream ostr_i;
    const std::string ref_i = xml_header + xml_int_ref;
    std::ostringstream ostr_s;
    const std::string ref_s = xml_header + xml_str_ref;

    const Option option_i("my_int", (uint16_t)17);
    EXPECT_TRUE(option_i.getName() == "my_int");
    EXPECT_TRUE(option_i.getValue() == "17");

    const Option option_s("my_string", "Yow.");
    EXPECT_TRUE(option_s.getName() == "my_string");
    EXPECT_TRUE(option_s.getValue() == "Yow.");
    EXPECT_TRUE(option_s.getValue() == "Yow.");
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
    EXPECT_EQ(s.getOptions().getOptions().size(), 6U);
}

TEST(OptionsTest, valid)
{
    EXPECT_TRUE(Option::nameValid("foo_123_bar_baz", false));
    EXPECT_FALSE(Option::nameValid("foo_123_bar-baz", false));
    EXPECT_FALSE(Option::nameValid("Afoo_123_bar_baz", false));
    EXPECT_FALSE(Option::nameValid("1foo_123_bar_baz", false));
    EXPECT_FALSE(Option::nameValid("1foo_123_bar_baz", false));
}

} // namespace pdal
