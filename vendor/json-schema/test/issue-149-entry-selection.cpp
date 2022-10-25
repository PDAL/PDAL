#include <nlohmann/json-schema.hpp>

#include <iostream>

using nlohmann::json;
using nlohmann::json_uri;
using nlohmann::json_schema::json_validator;

namespace
{

static int error_count;
#define EXPECT_EQ(a, b)                                              \
	do {                                                             \
		if (a != b) {                                                \
			std::cerr << "Failed: '" << a << "' != '" << b << "'\n"; \
			error_count++;                                           \
		}                                                            \
	} while (0)

// The schema is defined based upon a string literal
static json person_schema = R"(
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "type": "integer",
    "definitions": {
        "A": {
            "type": "object",
            "properties": {
                "b": {
                    "$ref": "#/definitions/B"
                }
            }
        },
        "B": {
            "type": "integer"
        }
    }
})"_json;

class store_err_handler : public nlohmann::json_schema::basic_error_handler
{
	void error(const nlohmann::json::json_pointer &ptr, const json &instance, const std::string &message) override
	{
		nlohmann::json_schema::basic_error_handler::error(ptr, instance, message);
		std::cerr << "ERROR: '" << ptr << "' - '" << instance << "': " << message << "\n";
		failed.push_back(ptr);
	}

public:
	std::vector<nlohmann::json::json_pointer> failed;

	void reset() override
	{
		nlohmann::json_schema::basic_error_handler::reset();
		failed.clear();
	}
};

} // namespace

static json_validator validator(person_schema);

int main(void)
{
	store_err_handler err;

	validator.validate(1, err); // OK
	EXPECT_EQ(err.failed.size(), 0);
	err.reset();

	validator.validate("1", err); // no name
	EXPECT_EQ(err.failed.size(), 1);
	err.reset();

	validator.validate(1, err, json_uri("#/definitions/B"));
	EXPECT_EQ(err.failed.size(), 0);
	err.reset();

	validator.validate("1", err, json_uri("#/definitions/B"));
	EXPECT_EQ(err.failed.size(), 1);
	err.reset();

	validator.validate({{"b", 1}}, err, json_uri("#/definitions/A"));
	EXPECT_EQ(err.failed.size(), 0);
	err.reset();

	validator.validate({{"b", "1"}}, err, json_uri("#/definitions/A"));
	EXPECT_EQ(err.failed.size(), 1);
	err.reset();

	return error_count;
}
