/*
 * JSON schema validator for JSON for modern C++
 *
 * Copyright (c) 2016-2019 Patrick Boettcher <p@yai.se>.
 *
 * SPDX-License-Identifier: MIT
 *
 */
#include <nlohmann/json-schema.hpp>

#include <fstream>
#include <iostream>
#include <regex>

using nlohmann::json;
using nlohmann::json_uri;
using nlohmann::json_schema::json_validator;

static void loader(const json_uri &uri, json &schema)
{
	if (uri.location() == "http://json-schema.org/draft-07/schema") {
		schema = nlohmann::json_schema::draft7_schema_builtin;
		return;
	}

	std::string fn = JSON_SCHEMA_TEST_SUITE_PATH;
	fn += "/remotes";
	fn += uri.path();
	std::cerr << fn << "\n";

	std::fstream s(fn.c_str());
	if (!s.good())
		throw std::invalid_argument("could not open " + uri.url() + " for schema loading\n");

	try {
		s >> schema;
	} catch (std::exception &e) {
		throw e;
	}
}

// from here
// https://stackoverflow.com/a/34571089/880584
static std::string base64_decode(const std::string &in)
{
	std::string out;

	std::vector<int> T(256, -1);
	for (int i = 0; i < 64; i++)
		T["ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[i]] = i;

	unsigned val = 0;
	int valb = -8;
	for (uint8_t c : in) {
		if (c == '=')
			break;

		if (T[c] == -1) {
			throw std::invalid_argument("base64-decode: unexpected character in encode string: '" + std::string(1, c) + "'");
		}
		val = (val << 6) + T[c];
		valb += 6;
		if (valb >= 0) {
			out.push_back(char((val >> valb) & 0xFF));
			valb -= 8;
		}
	}
	return out;
}

static void content(const std::string &contentEncoding, const std::string &contentMediaType, const json &instance)
{
	std::string content = instance;

	if (contentEncoding == "base64")
		content = base64_decode(instance);
	else if (contentEncoding != "")
		throw std::invalid_argument("unable to check for contentEncoding '" + contentEncoding + "'");

	if (contentMediaType == "application/json")
		auto dummy = json::parse(content); // throws if conversion fails
	else if (contentMediaType != "")
		throw std::invalid_argument("unable to check for contentMediaType '" + contentMediaType + "'");
}

int main(void)
{
	json validation; // a validation case following the JSON-test-suite-schema

	try {
		std::cin >> validation;
	} catch (std::exception &e) {
		std::cout << e.what() << "\n";
		return EXIT_FAILURE;
	}

	size_t total_failed = 0,
	       total = 0;

	for (auto &test_group : validation) {
		size_t group_failed = 0,
		       group_total = 0;

		std::cout << "Testing Group " << test_group["description"] << "\n";

		const auto &schema = test_group["schema"];

		json_validator validator(loader,
		                         nlohmann::json_schema::default_string_format_check,
		                         content);

		validator.set_root_schema(schema);

		for (auto &test_case : test_group["tests"]) {
			std::cout << "  Testing Case " << test_case["description"] << "\n";

			bool valid = true;

			try {
				validator.validate(test_case["data"]);
			} catch (const std::out_of_range &e) {
				valid = false;
				std::cout << "    Test Case Exception (out of range): " << e.what() << "\n";

			} catch (const std::invalid_argument &e) {
				valid = false;
				std::cout << "    Test Case Exception (invalid argument): " << e.what() << "\n";

			} catch (const std::logic_error &e) {
				valid = !test_case["valid"]; /* force test-case failure */
				std::cout << "    Not yet implemented: " << e.what() << "\n";
			}

			if (valid == test_case["valid"])
				std::cout << "      --> Test Case exited with " << valid << " as expected.\n";
			else {
				group_failed++;
				std::cout << "      --> Test Case exited with " << valid << " NOT expected.\n";
			}
			group_total++;
			std::cout << "\n";
		}
		total_failed += group_failed;
		total += group_total;
		std::cout << "Group RESULT: " << test_group["description"] << " "
		          << (group_total - group_failed) << " of " << group_total
		          << " have succeeded - " << group_failed << " failed\n";
		std::cout << "-------------\n";
	}

	std::cout << "Total RESULT: " << (total - total_failed) << " of " << total << " have succeeded - " << total_failed << " failed\n";

	return total_failed;
}
