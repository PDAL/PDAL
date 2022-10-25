#include <iostream>

#include <nlohmann/json-schema.hpp>

using nlohmann::json;
using nlohmann::json_schema::json_validator;

// The schema is defined based upon a string literal
static json uri_schema = R"(
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "myUri": {
      "type":"string",
      "format": "uri"
    }
  }
})"_json;

// The people are defined with brace initialization
static json good_uri = {{"myUri", "http://hostname.com/"}};
static json bad_uri = {{"myUri", "http:/hostname.com/"}};

static void uri_format_checker(const std::string &format, const std::string &value)
{
	if (format == "uri") {
		if (value.find("://") == std::string::npos)
			throw std::invalid_argument("URI does not contain :// - invalid");
	} else
		throw std::logic_error("Don't know how to validate " + format);
}

int main()
{
	json_validator validator(nullptr, uri_format_checker); // create validator

	try {
		validator.set_root_schema(uri_schema); // insert root-schema
	} catch (const std::exception &e) {
		std::cerr << "Validation of schema failed, here is why: " << e.what() << "\n";
		return EXIT_FAILURE;
	}

	validator.validate(good_uri);

	try {
		validator.validate(bad_uri);
	} catch (const std::exception &e) {
		std::cerr << "Validation expectedly failed, here is why: " << e.what() << "\n";
	}

	return EXIT_SUCCESS;
}
