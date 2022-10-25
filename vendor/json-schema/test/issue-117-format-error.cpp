// issue-00-format-error.cpp

#include "nlohmann/json-schema.hpp"
#include "nlohmann/json.hpp"
#include <iostream>

static int error_count = 0;

#define CHECK_THROW(x, msg)                \
	{                                      \
		bool fail = false;                 \
		try {                              \
			x;                             \
		} catch (std::exception &) {       \
			fail = true;                   \
		}                                  \
		if (fail == false) {               \
			++error_count;                 \
			std::cout << msg << std::endl; \
		}                                  \
	}

#define CHECK_NO_THROW(x, msg)                                        \
	{                                                                 \
		bool fail = false;                                            \
		std::string exception_error;                                  \
		try {                                                         \
			x;                                                        \
		} catch (std::exception & e) {                                \
			fail = true;                                              \
			exception_error = e.what();                               \
		}                                                             \
		if (fail == true) {                                           \
			++error_count;                                            \
			std::cout << msg << ": " << exception_error << std::endl; \
		}                                                             \
	}

using json = nlohmann::json;
using validator = nlohmann::json_schema::json_validator;

json schema_with_format = json::parse(R"(
{
  "type": "object",
  "properties": {
    "str": {
      "type": "string",
      "format": "placeholder"
    }
  }
}
)");

int main()
{
	// check that if we get validator without format checker we get error at schema loading
	validator without_format_checker;

	CHECK_THROW(without_format_checker.set_root_schema(schema_with_format), "validator without format checker must fail at schema loading");

	// check that with format checker all works fine
	validator with_format_checker{nullptr, [](const std::string &, const std::string &) {}};

	CHECK_NO_THROW(with_format_checker.set_root_schema(schema_with_format), "schema must be succesed by validator with format checker");

	CHECK_NO_THROW(with_format_checker.validate(json{{"str", "placeholder"}}), "validator must not throw while validation schema with format");

	return error_count;
}
