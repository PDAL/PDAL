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

using nlohmann::json;
using nlohmann::json_uri;
using nlohmann::json_schema::json_validator;

static void usage(const char *name)
{
	std::cerr << "Usage: " << name << " <schema> < <document>\n";
	exit(EXIT_FAILURE);
}

static void loader(const json_uri &uri, json &schema)
{
	std::string filename = "./" + uri.path();
	std::ifstream lf(filename);
	if (!lf.good())
		throw std::invalid_argument("could not open " + uri.url() + " tried with " + filename);
	try {
		lf >> schema;
	} catch (const std::exception &e) {
		throw e;
	}
}

class custom_error_handler : public nlohmann::json_schema::basic_error_handler
{
	void error(const nlohmann::json::json_pointer &ptr, const json &instance, const std::string &message) override
	{
		nlohmann::json_schema::basic_error_handler::error(ptr, instance, message);
		std::cerr << "ERROR: '" << ptr << "' - '" << instance << "': " << message << "\n";
	}
};

int main(int argc, char *argv[])
{
	if (argc != 2)
		usage(argv[0]);

	std::ifstream f(argv[1]);
	if (!f.good()) {
		std::cerr << "could not open " << argv[1] << " for reading\n";
		usage(argv[0]);
	}

	// 1) Read the schema for the document you want to validate
	json schema;
	try {
		f >> schema;
	} catch (const std::exception &e) {
		std::cerr << e.what() << " at " << f.tellg() << " - while parsing the schema\n";
		return EXIT_FAILURE;
	}

	// 2) create the validator and
	json_validator validator(loader,
	                         nlohmann::json_schema::default_string_format_check);

	try {
		// insert this schema as the root to the validator
		// this resolves remote-schemas, sub-schemas and references via the given loader-function
		validator.set_root_schema(schema);
	} catch (const std::exception &e) {
		std::cerr << "setting root schema failed\n";
		std::cerr << e.what() << "\n";
	}

	// 3) do the actual validation of the document
	json document;

	try {
		std::cin >> document;
	} catch (const std::exception &e) {
		std::cerr << "json parsing failed: " << e.what() << " at offset: " << std::cin.tellg() << "\n";
		return EXIT_FAILURE;
	}

	custom_error_handler err;
	validator.validate(document, err);

	if (err) {
		std::cerr << "schema validation failed\n";
		return EXIT_FAILURE;
	}

	std::cerr << "document is valid\n";

	return EXIT_SUCCESS;
}
