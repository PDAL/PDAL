/*
 * JSON schema validator for JSON for modern C++
 *
 * Copyright (c) 2016-2019 Patrick Boettcher <p@yai.se>.
 *
 * SPDX-License-Identifier: MIT
 *
 */
#include <cstdlib>
#include <nlohmann/json-schema.hpp>

#include <iostream>

using nlohmann::json;
using nlohmann::json_uri;

static int errors;

static void EXPECT_EQ(const std::string &a, const std::string &b)
{
	if (a != b) {
		std::cerr << "Failed: '" << a << "' != '" << b << "'\n";
		errors++;
	}
}

static void EXPECT_EQ(const nlohmann::json_uri &a, const std::string &b)
{
	EXPECT_EQ(a.to_string(), b);
}

static void paths(json_uri start,
                  const std::string &full,
                  const std::string &full_path,
                  const std::string &no_path)
{
	EXPECT_EQ(start, full + " # ");

	auto a = start.derive("other.json");
	EXPECT_EQ(a, full_path + "/other.json # ");

	auto b = a.derive("base.json");
	EXPECT_EQ(b, full_path + "/base.json # ");

	auto c = b.derive("subdir/base.json");
	EXPECT_EQ(c, full_path + "/subdir/base.json # ");

	auto d = c.derive("subdir2/base.json");
	EXPECT_EQ(d, full_path + "/subdir/subdir2/base.json # ");

	auto e = c.derive("/subdir2/base.json");
	EXPECT_EQ(e, no_path + "/subdir2/base.json # ");

	auto f = c.derive("new.json");
	EXPECT_EQ(f, full_path + "/subdir/new.json # ");

	auto g = c.derive("/new.json");
	EXPECT_EQ(g, no_path + "/new.json # ");
}

static void pointer_plain_name(json_uri start,
                               const std::string &full,
                               const std::string &full_path,
                               const std::string &no_path)
{
	auto a = start.derive("#/json/path");
	EXPECT_EQ(a, full + " # /json/path");

	a = start.derive("#/json/special_%22");
	EXPECT_EQ(a, full + " # /json/special_\"");

	a = a.derive("#foo");
	EXPECT_EQ(a, full + " # foo");

	a = a.derive("#foo/looks_like_json/poiner/but/isnt");
	EXPECT_EQ(a, full + " # foo/looks_like_json/poiner/but/isnt");
	EXPECT_EQ(a.identifier(), "foo/looks_like_json/poiner/but/isnt");
	EXPECT_EQ(a.pointer().to_string(), "");

	a = a.derive("#/looks_like_json/poiner/and/it/is");
	EXPECT_EQ(a, full + " # /looks_like_json/poiner/and/it/is");
	EXPECT_EQ(a.pointer().to_string(), "/looks_like_json/poiner/and/it/is");
	EXPECT_EQ(a.identifier(), "");
}

int main(void)
{
	json_uri empty("");
	paths(empty,
	      "",
	      "",
	      "");

	json_uri http("http://json-schema.org/draft-07/schema#");
	paths(http,
	      "http://json-schema.org/draft-07/schema",
	      "http://json-schema.org/draft-07",
	      "http://json-schema.org");

	pointer_plain_name(http,
	                   "http://json-schema.org/draft-07/schema",
	                   "http://json-schema.org/draft-07",
	                   "http://json-schema.org");

	return errors;
}
