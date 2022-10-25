#include <iostream>
#include <nlohmann/json-schema.hpp>

using nlohmann::json;
using nlohmann::json_schema::json_validator;

static const json person_schema = R"(
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
        "address": {
            "type": "object",
            "default": {},
            "properties": {
                "street": {
                    "type": "string",
                    "default": "Abbey Road"
                }
            }
        },
        "work address": {
            "type": "object",
            "default": null,
            "properties": {
                "street": {
                    "type": "string",
                    "default": "Abbey Road"
                }
            }
        },
        "other address": {
            "type": "object",
            "properties": {
                "street": {
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
    "additionalProperties": false,
    "type": "object"
})"_json;

int main(void)
{
	json_validator validator{};
	validator.set_root_schema(person_schema);

	{
		// add address which is optional that should generate a diff containing a default street
		json person_emtpy_address = R"({
		"name": "Hans",
		"age": 69,
		"address": {}
		})"_json;

		const auto default_patch = validator.validate(person_emtpy_address);

		if (!default_patch.is_array()) {
			std::cerr << "Patch with defaults is expected to be an array" << std::endl;
			return 1;
		}

		if (default_patch.size() != 1) {
			std::cerr << "Patch with defaults is expected to contain one opperation" << std::endl;
			return 1;
		}

		const auto &single_op = default_patch[0];

		if (!single_op.contains("op")) {
			std::cerr << "Patch with defaults is expected to contain opperation entry" << std::endl;
			return 1;
		}

		if (single_op["op"].get<std::string>() != "add") {
			std::cerr << "Patch with defaults is expected to contain add opperation" << std::endl;
			return 1;
		}

		if (!single_op.contains("path")) {
			std::cerr << "Patch with defaults is expected to contain a path" << std::endl;
			return 1;
		}

		const auto &readPath = single_op["path"].get<std::string>();
		if (readPath != "/address/street") {
			std::cerr << "Patch with defaults contains wrong path. It is " << readPath << " and should be "
				<< "/address/street" << std::endl;
			return 1;
		}

		if (!single_op.contains("value")) {
			std::cerr << "Patch with defaults is expected to contain a value" << std::endl;
			return 1;
		}

		if (single_op["value"].get<std::string>() != "Abbey Road") {
			std::cerr << "Patch with defaults contains wrong value" << std::endl;
			return 1;
		}
	}
	{
		// add address which is optional that should generate a diff containing a empty object
		// but not work address which is null or other address which has no default
		json person_missing_address = R"({
		"name": "Hans",
		"age": 69
		})"_json;

		const auto default_patch = validator.validate(person_missing_address);

		if (!default_patch.is_array()) {
			std::cerr << "Patch with defaults is expected to be an array" << std::endl;
			return 1;
		}

		if (default_patch.size() != 1) {
			std::cerr << "Patch with defaults is expected to contain one opperation" << std::endl;
			return 1;
		}

		const auto &single_op = default_patch[0];

		if (!single_op.contains("op")) {
			std::cerr << "Patch with defaults is expected to contain opperation entry" << std::endl;
			return 1;
		}

		if (single_op["op"].get<std::string>() != "add") {
			std::cerr << "Patch with defaults is expected to contain add opperation" << std::endl;
			return 1;
		}

		if (!single_op.contains("path")) {
			std::cerr << "Patch with defaults is expected to contain a path" << std::endl;
			return 1;
		}

		const auto &readPath = single_op["path"].get<std::string>();
		if (readPath != "/address") {
			std::cerr << "Patch with defaults contains wrong path. It is " << readPath << " and should be "
				<< "/address" << std::endl;
			return 1;
		}

		if (!single_op.contains("value")) {
			std::cerr << "Patch with defaults is expected to contain a value" << std::endl;
			return 1;
		}

		if ( !single_op["value"].is_object() || !single_op["value"].empty()) {
			std::cerr << "Patch with defaults contains wrong value" << std::endl;
			return 1;
		}
	}
	return 0;
}
