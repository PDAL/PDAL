#include <iomanip>
#include <iostream>

#include <nlohmann/json-schema.hpp>

using nlohmann::json;
using nlohmann::json_schema::json_validator;

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
      "address":{
        "type": "object",
        "properties":{
          "street":{
            "type": "string",
            "default": "Abbey Road"
          }
        }
      }
    },
    "required": [
                 "name",
                 "age"
                 ],
    "type": "object"
}

)"_json;

// The people are defined with brace initialization
static json bad_person = {{"age", 42}};
static json good_person = {{"name", "Albert"}, {"age", 42}, {"address", {{"street", "Main Street"}}}};
static json good_defaulted_person = {{"name", "Knut"}, {"age", 69}, {"address", {}}};

int main()
{
	/* json-parse the schema */

	json_validator validator; // create validator

	try {
		validator.set_root_schema(person_schema); // insert root-schema
	} catch (const std::exception &e) {
		std::cerr << "Validation of schema failed, here is why: " << e.what() << "\n";
		return EXIT_FAILURE;
	}

	/* json-parse the people - API of 1.0.0, default throwing error handler */

	for (auto &person : {bad_person, good_person, good_defaulted_person}) {
		std::cout << "About to validate this person:\n"
		          << std::setw(2) << person << std::endl;
		try {
			auto defaultPatch = validator.validate(person); // validate the document - uses the default throwing error-handler
			std::cout << "Validation succeeded\n";
			std::cout << "Patch with defaults: " << defaultPatch.dump(2) << std::endl;
		} catch (const std::exception &e) {
			std::cerr << "Validation failed, here is why: " << e.what() << "\n";
		}
	}

	/* json-parse the people - with custom error handler */
	class custom_error_handler : public nlohmann::json_schema::basic_error_handler
	{
		void error(const nlohmann::json::json_pointer &ptr, const json &instance, const std::string &message) override
		{
			nlohmann::json_schema::basic_error_handler::error(ptr, instance, message);
			std::cerr << "ERROR: '" << ptr << "' - '" << instance << "': " << message << "\n";
		}
	};

	for (auto &person : {bad_person, good_person}) {
		std::cout << "About to validate this person:\n"
		          << std::setw(2) << person << std::endl;

		custom_error_handler err;
		validator.validate(person, err); // validate the document

		if (err)
			std::cerr << "Validation failed\n";
		else
			std::cout << "Validation succeeded\n";
	}

	return EXIT_SUCCESS;
}
