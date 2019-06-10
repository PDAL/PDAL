/******************************************************************************
* Copyright (c) 2016, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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

#include <nlohmann/json.hpp>

// No wordexp() on windows and I don't feel like doing something special.
#ifndef _WIN32

#include <wordexp.h>
#include <pdal/pdal_internal.hpp>
#include <pdal/util/ProgramArgs.hpp>

using namespace pdal;

#define HANDLE_EXCEPTION(a) \
    EXPECT_THROW(a, arg_error)
/**
    try {\
        a;\
    }\
    catch (arg_error e)\
    {\
        std::cerr << e.m_error << "\n";\
    }\
**/


namespace
{

StringList toStringList(const std::string& s)
{
    StringList slist;

    wordexp_t w;
    wordexp(s.c_str(), &w, WRDE_SHOWERR);
    for (size_t i = 0; i < w.we_wordc; ++i)
        slist.push_back(w.we_wordv[i]);
    wordfree(&w);
    return slist;
}

}

TEST(ProgramArgsTest, t1)
{
    ProgramArgs args;

    std::string m_foo;

    args.add("foo,f", "Foo description", m_foo, "foo");
    StringList s = toStringList("--foo");
    EXPECT_THROW(args.parse(s), arg_error);

    s = toStringList("--foo=TestFoo");
    args.reset();
    args.parse(s);
    EXPECT_EQ(m_foo, "TestFoo");

    s = toStringList("--foo TestBar");
    args.reset();
    args.parse(s);
    EXPECT_EQ(m_foo, "TestBar");

    s = toStringList("-f");
    args.reset();
    EXPECT_THROW(args.parse(s), arg_error);

    s = toStringList("-f -g");
    args.reset();
    EXPECT_THROW(args.parse(s), arg_error);

    s = toStringList("-f Gah");
    args.reset();
    args.parse(s);
    EXPECT_EQ(m_foo, "Gah");

    s = toStringList("--foo=-Foo");
    args.reset();
    args.parse(s);
    EXPECT_EQ(m_foo, "-Foo");
}

TEST(ProgramArgsTest, t2)
{
    ProgramArgs args;

    std::string m_foo;
    int m_bar;
    bool m_baz;

    args.add("foo,f", "Foo description", m_foo, "foo");
    args.add("bar", "Foo description", m_bar, 23);
    args.add("baz,z", "Foo description", m_baz);

    StringList s = toStringList("--foo TEst --bar=45 -z");
    args.parse(s);
    EXPECT_EQ(m_foo, "TEst");
    EXPECT_EQ(m_bar, 45);
    EXPECT_EQ(m_baz, true);

    args.reset();
    s = toStringList("-zf FooTest --bar=55");
    args.parse(s);
    EXPECT_EQ(m_foo, "FooTest");
    EXPECT_EQ(m_bar, 55);
    EXPECT_EQ(m_baz, true);

    args.reset();
    s = toStringList("");
    EXPECT_EQ(m_foo, "foo");
    EXPECT_EQ(m_bar, 23);
    EXPECT_EQ(m_baz, false);

    s = toStringList("--zf Foo");
    EXPECT_THROW(args.parse(s), arg_error);

    s = toStringList("-fz FooTest");
    EXPECT_THROW(args.parse(s), arg_error);

    args.reset();
    s = toStringList("--foo=\"This is a test\"");
    args.parse(s);
    EXPECT_EQ(m_foo, "This is a test");
}

TEST(ProgramArgsTest, t3)
{
    ProgramArgs args;

    std::string m_foo;
    int m_bar;
    bool m_baz;

    args.add("foo,f", "Foo description", m_foo, "foo");
    args.add("bar", "Foo description", m_bar, 23);
    args.add("baz,z", "Foo description", m_baz);

    // Go through exceptions procedurally.

    // Argument 'foo' needs a value and none was provided.
    StringList s = toStringList("--foo");
    HANDLE_EXCEPTION(args.parse(s));

    // Invalid value for argument 'bar'.
    s = toStringList("--bar=foo");
    HANDLE_EXCEPTION(args.parse(s));

    // Argument 'bar' needs a value and none was provided.
    s = toStringList("--bar");
    HANDLE_EXCEPTION(args.parse(s));

    // Argument 'foo' needs a value and none was provided.
    s = toStringList("--foo --baz");
    HANDLE_EXCEPTION(args.parse(s));

    // Unexpected argument 'flub'.
    s = toStringList("--flub");
    HANDLE_EXCEPTION(args.parse(s));

    // Value 'flub' provided for argument 'baz' when none is expected.
    s = toStringList("--baz=flub");
    HANDLE_EXCEPTION(args.parse(s));

    // Unexpected argument '-q'.
    s = toStringList("-zq");
    HANDLE_EXCEPTION(args.parse(s));

    // Unexpected argument '-q'.
    s = toStringList("-q");
    HANDLE_EXCEPTION(args.parse(s));

    // Short option 'f' expects value but appears in option group '-fz'.
    s = toStringList("-fz foo");
    HANDLE_EXCEPTION(args.parse(s));
}

TEST(ProgramArgsTest, t4)
{
    ProgramArgs args;

    std::string m_foo;
    int m_bar;
    bool m_baz;

    // Test for exceptions in adding args.

    // Invalid program option specification
    HANDLE_EXCEPTION(args.add("foo,f,q", "Foo description", m_foo, "foo"));

    // Short option not specified as single character
    HANDLE_EXCEPTION(args.add("foo,flub", "Foo description", m_foo, "foo"));

    // No program option provided.
    HANDLE_EXCEPTION(args.add("", "Foo description", m_foo, "foo"));
}

TEST(ProgramArgsTest, synonym)
{
    ProgramArgs args;

    std::string m_foo;

    args.add("foo,f", "Foo description", m_foo, "foo");
    args.addSynonym("foo", "bar");
    StringList s = toStringList("--bar");
    EXPECT_THROW(args.parse(s), arg_error);

    s = toStringList("--bar=TestFoo");
    args.reset();
    args.parse(s);
    EXPECT_EQ(m_foo, "TestFoo");
}

TEST(ProgramArgsTest, positional)
{
    ProgramArgs args;

    std::string m_foo;
    int m_bar;
    bool m_baz;

    args.add("foo,f", "Foo description", m_foo, "foo").setPositional();
    args.add("bar", "Foo description", m_bar, 23).setPositional();
    Arg& baz = args.add("baz,z", "Foo description", m_baz);
    EXPECT_THROW(baz.setPositional(), arg_error);

    StringList s = toStringList("--foo Foo -z 55");
    args.parse(s);
    EXPECT_EQ(m_foo, "Foo");
    EXPECT_EQ(m_bar, 55);
    EXPECT_EQ(m_baz, true);

    args.reset();

    s = toStringList("-z Flub 66");
    args.parse(s);
    EXPECT_EQ(m_foo, "Flub");
    EXPECT_EQ(m_bar, 66);
    EXPECT_EQ(m_baz, true);

    args.reset();

    s = toStringList("Flub 66");
    args.parse(s);
    EXPECT_EQ(m_foo, "Flub");
    EXPECT_EQ(m_bar, 66);
    EXPECT_EQ(m_baz, false);
}

TEST(ProgramArgsTest, vector)
{
    ProgramArgs args;

    std::string m_foo;
    std::vector<int> m_bar;
    std::vector<int> m_flub;
    bool m_baz;

    args.add("foo,f", "Foo description", m_foo, "foo").setPositional();
    args.add("bar", "Foo description", m_bar).setOptionalPositional();
    args.add("baz,z", "Foo description", m_baz);
    args.add("flub", "Flub description", m_flub, {1, 3, 5});

    StringList s = toStringList("--bar 23 --bar 45 Foo -z");
    args.parse(s);
    EXPECT_EQ(m_foo, "Foo");
    EXPECT_EQ(m_bar.size(), 2u);
    EXPECT_EQ(m_baz, true);
    EXPECT_EQ(m_bar[0], 23);
    EXPECT_EQ(m_bar[1], 45);
    EXPECT_EQ(m_flub.size(), 3u);
    EXPECT_EQ(m_flub[0], 1);
    EXPECT_EQ(m_flub[1], 3);
    EXPECT_EQ(m_flub[2], 5);

    args.reset();
    s = toStringList("Foo");
    args.parse(s);
    EXPECT_EQ(m_bar.size(), 0u);
    EXPECT_EQ(m_foo, "Foo");
    EXPECT_EQ(m_baz, false);
    EXPECT_EQ(m_flub.size(), 3u);

    args.reset();
    s = toStringList("Fool 44 55 66");
    args.parse(s);
    EXPECT_EQ(m_foo, "Fool");
    EXPECT_EQ(m_bar.size(), 3u);
    EXPECT_EQ(m_bar[0], 44);
    EXPECT_EQ(m_bar[1], 55);
    EXPECT_EQ(m_bar[2], 66);
    EXPECT_EQ(m_baz, false);
    EXPECT_EQ(m_flub.size(), 3u);

    args.reset();
    s = toStringList("--bar 23 --flub 2 Foo -z --flub 4");
    args.parse(s);
    EXPECT_EQ(m_foo, "Foo");
    EXPECT_EQ(m_bar.size(), 1u);
    EXPECT_EQ(m_baz, true);
    EXPECT_EQ(m_bar[0], 23);
    EXPECT_EQ(m_flub.size(), 2u);
    EXPECT_EQ(m_flub[0], 2);
    EXPECT_EQ(m_flub[1], 4);
}

TEST(ProgramArgsTest, stringvector)
{
    ProgramArgs args;

    std::string m_foo;
    std::vector<std::string> m_bar;
    bool m_baz;

    args.add("foo,f", "Foo description", m_foo, "foo").setPositional();
    args.add("bar", "Foo description", m_bar).setOptionalPositional();
    args.add("baz,z", "Foo description", m_baz);

    StringList s = toStringList("--bar a,b,c --bar d,e,f Foo -z");
    args.parse(s);
    EXPECT_EQ(m_foo, "Foo");
    EXPECT_EQ(m_bar.size(), 6u);
    EXPECT_EQ(m_baz, true);
    EXPECT_EQ(m_bar[0], "a");
    EXPECT_EQ(m_bar[1], "b");
    EXPECT_EQ(m_bar[2], "c");
    EXPECT_EQ(m_bar[3], "d");
    EXPECT_EQ(m_bar[4], "e");
    EXPECT_EQ(m_bar[5], "f");

    args.reset();
    s = toStringList("Foo");
    args.parse(s);
    EXPECT_EQ(m_bar.size(), 0u);
    EXPECT_EQ(m_foo, "Foo");
    EXPECT_EQ(m_baz, false);

    args.reset();
    s = toStringList("Fool 44 55 66");
    args.parse(s);
    EXPECT_EQ(m_foo, "Fool");
    EXPECT_EQ(m_bar.size(), 3u);
    EXPECT_EQ(m_bar[0], "44");
    EXPECT_EQ(m_bar[1], "55");
    EXPECT_EQ(m_bar[2], "66");
    EXPECT_EQ(m_baz, false);
}

TEST(ProgramArgsTest, vectorfail)
{
    ProgramArgs args;

    std::string m_foo;
    std::vector<int> m_bar;
    bool m_baz;

    // Standard positional arg following optional positional arg disallowed.
    args.add("bar", "Foo description", m_bar).setOptionalPositional();
    args.add("foo,f", "Foo description", m_foo, "foo").setPositional();
    args.add("baz,z", "Foo description", m_baz);

    StringList s;
    EXPECT_THROW(args.parse(s), arg_error);
}

TEST(ProgramArgsTest, parseSimple)
{
    ProgramArgs args;

    std::string m_foo;
    int m_bar;
    bool m_baz;
    StringList m_vec;

    args.add("foo,f", "Foo description", m_foo, "foo").setPositional();
    args.add("vec", "Vec description", m_vec).setPositional();
    args.add("bar", "Foo description", m_bar, 23);
    args.add("baz,z", "Foo description", m_baz);

    StringList s = toStringList("--foo TEst --bar=45 -z");
    args.parseSimple(s);
    EXPECT_EQ(m_foo, "TEst");
    EXPECT_EQ(m_bar, 45);
    EXPECT_EQ(m_baz, true);

    args.reset();
    s = toStringList("-zf FooTest --bar=55");
    args.parseSimple(s);
    EXPECT_EQ(m_foo, "FooTest");
    EXPECT_EQ(m_bar, 55);
    EXPECT_EQ(m_baz, true);

    args.reset();
    s = toStringList("");
    EXPECT_EQ(m_foo, "foo");
    EXPECT_EQ(m_bar, 23);
    EXPECT_EQ(m_baz, false);

    s = toStringList("--bar 55 Foo Barf --holy=Holy --cow=Moo Vec");
    args.parseSimple(s);
    EXPECT_EQ(m_foo, "Foo");
    EXPECT_EQ(m_bar, 55);
    EXPECT_EQ(m_baz, false);
    EXPECT_EQ(m_vec.size(), 2U);
    EXPECT_EQ(m_vec[0], "Barf");
    EXPECT_EQ(m_vec[1], "Vec");
    EXPECT_EQ(s.size(), 2U);
    EXPECT_EQ(s[0], "--holy=Holy");
    EXPECT_EQ(s[1], "--cow=Moo");

    s = toStringList("--bar 55 Foo Barf");
    EXPECT_THROW(args.parse(s), arg_error);
}

TEST(ProgramArgsTest, json)
{
    // Test JSON arguments.  If the arg refers to an underlying NL::json,
    // it should be parsed into that Json::Value.  If a string, then that
    // string should contain the stringified JSON (which should be parsable
    // into equivalent JSON).
    NL::json m_json;
    std::string m_string;
    NL::json verify;
    verify["key"] = 42;

    const std::string stringified("\"{ \\\"key\\\": 42 }\"");

    ProgramArgs args;
    args.add("json,j", "JSON description", m_json);
    args.add("string,s", "String description", m_string);

    // An unset Json::Value should be null.
    {
        StringList s = toStringList("");
        EXPECT_NO_THROW(args.parse(s));
        EXPECT_TRUE(m_json.is_null());
    }

    // Test an object.
    {
        StringList s = toStringList(
                "--json " + stringified + " --string " + stringified);
        EXPECT_NO_THROW(args.parse(s));

        // Verify the JSON value.
        EXPECT_EQ(m_json, verify) << m_json << " != " << verify;

        // Verify that the string value parses into the proper JSON value.
        NL::json fromString;
        EXPECT_NO_THROW(fromString = NL::json::parse(m_string));
        EXPECT_EQ(fromString, verify) << fromString << " != " << verify;
    }

    // Test non-objects.
    {
        args.reset();
        StringList s = toStringList("--json [1,2,3]");
        EXPECT_NO_THROW(args.parse(s));
        verify = NL::json::parse("[1, 2, 3]");
        EXPECT_EQ(m_json, verify);

        args.reset();
        s = toStringList("--json 2");
        EXPECT_NO_THROW(args.parse(s));
        verify = 2;
        EXPECT_TRUE(m_json.is_number());
        EXPECT_EQ(m_json.get<int>(), 2);

        args.reset();
        s = toStringList("--json 3.14");
        EXPECT_NO_THROW(args.parse(s));
        verify = 3.14;
        EXPECT_TRUE(m_json.is_number_float());
        EXPECT_EQ(m_json.get<double>(), 3.14);
    }
}

TEST(ProgramArgsTest, invalidJson)
{
    NL::json m_json;
    std::string m_string;
    NL::json verify;
    verify["key"] = 42;

    ProgramArgs args;
    args.add("json,j", "JSON description", m_json);
    args.add("string,s", "String description", m_string);

    // If the underlying is a string, then invalid JSON is fine during argument
    // parsing, but will fail later if parsed as JSON.
    args.reset();
    StringList s = toStringList("--string \"{ invalid JSON here }\"");
    EXPECT_NO_THROW(args.parse(s));

    // The string was successfully parsed, but contains invalid JSON.
    auto parseJson = [](const std::string& s)
    {
        return NL::json::parse(s);
    };

    EXPECT_THROW(parseJson(m_string), NL::json::parse_error);

    // If the underlying is a Json::Value, then argument parsing should fail
    // up front.
    args.reset();
    s = toStringList("--json \"{ invalid JSON here }\"");
    EXPECT_ANY_THROW(args.parse(s));

    args.reset();
    s = toStringList("--json");
    EXPECT_ANY_THROW(args.parse(s));
}

// Test to make sure negative numbers work in lists.
TEST(ProgramArgsTest, issue_2155)
{
    ProgramArgs args;

    std::vector<int> m_vec;

    args.add("foo", "Vec description", m_vec);

    StringList s = toStringList("--foo=-1 --foo 2 --foo=-3");
    args.parse(s);

    EXPECT_EQ(m_vec.size(), 3U);
    EXPECT_EQ(m_vec[0], -1);
    EXPECT_EQ(m_vec[1], 2);
    EXPECT_EQ(m_vec[2], -3);

    args.reset();

    s = toStringList("--foo=-1 --foo");
    EXPECT_THROW(args.parse(s), arg_val_error);
}
#endif
