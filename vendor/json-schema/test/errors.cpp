#include <nlohmann/json-schema.hpp>

#include <iostream>

static int error_count;

#define EXPECT_EQ(a, b)                                              \
	do {                                                             \
		if (a != b) {                                                \
			std::cerr << "Failed: '" << a << "' != '" << b << "'\n"; \
			error_count++;                                           \
		}                                                            \
	} while (0)

using nlohmann::json;
using nlohmann::json_schema::json_validator;

namespace
{

// The schema is defined based upon a string literal
static json person_schema = R"(
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "title": "A person",
    "properties": {
        "name": {
            "description": "Name",
            "type": "string"
        },
        "age": {
            "description": "Age of the person",
            "type": "number",
            "minimum": 2,
            "maximum": 200
        },
        "phones": {
            "type": "array",
            "items": {
                 "type": "number"
            }
        }
    },
    "required": [
                 "name",
                 "age"
                 ],
    "additionalProperties": false,
    "type": "object"
})"_json;

class store_ptr_err_handler : public nlohmann::json_schema::basic_error_handler
{
	void error(const nlohmann::json::json_pointer &ptr, const json &instance, const std::string &message) override
	{
		nlohmann::json_schema::basic_error_handler::error(ptr, instance, message);
		std::cerr << "ERROR: '" << ptr << "' - '" << instance << "': " << message << "\n";
		failed_pointers.push_back(ptr);
	}

public:
	std::vector<nlohmann::json::json_pointer> failed_pointers;

	void reset() override
	{
		nlohmann::json_schema::basic_error_handler::reset();
		failed_pointers.clear();
	}
};

} // namespace

int main(void)
{
	json_validator validator;

	try {
		validator.set_root_schema(person_schema); // insert root-schema
	} catch (const std::exception &e) {
		std::cerr << "Validation of schema failed, here is why: " << e.what() << "\n";
		return EXIT_FAILURE;
	}

	store_ptr_err_handler err;

	validator.validate({{"age", 42}, {"name", "John"}}, err); // OK
	EXPECT_EQ(err.failed_pointers.size(), 0);
	err.reset();

	validator.validate({{"age", 42}}, err); // no name

	EXPECT_EQ(err.failed_pointers.size(), 1);
	EXPECT_EQ(err.failed_pointers[0].to_string(), "");
	err.reset();

	validator.validate({{"street", "Boulevard"}}, err); // no name and no age
	EXPECT_EQ(err.failed_pointers.size(), 3);
	EXPECT_EQ(err.failed_pointers[0].to_string(), "");
	EXPECT_EQ(err.failed_pointers[1].to_string(), "");
	EXPECT_EQ(err.failed_pointers[2].to_string(), "");
	err.reset();

	validator.validate({{"age", 42}, {"name", 12}}, err); // name must be a string
	EXPECT_EQ(err.failed_pointers.size(), 1);
	EXPECT_EQ(err.failed_pointers[0].to_string(), "/name");
	err.reset();

	validator.validate({
	                       {"age", 42},
	                       {"name", "John"},
	                       {"phones", {1234, "223"}},
	                   },
	                   err); // name must be a string
	EXPECT_EQ(err.failed_pointers.size(), 1);
	EXPECT_EQ(err.failed_pointers[0].to_string(), "/phones/1");
	err.reset();

	validator.validate({
	                       {"age", 42},
	                       {"name", "John"},
	                       {"phones", {0}},
						   {"post-code", 12345},
	                   },
	                   err); // name must be a string
	EXPECT_EQ(err.failed_pointers.size(), 1);
	EXPECT_EQ(err.failed_pointers[0].to_string(), "");
	err.reset();

	return error_count;
}
