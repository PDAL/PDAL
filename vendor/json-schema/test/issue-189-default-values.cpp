#include <iostream>
#include <nlohmann/json-schema.hpp>

using nlohmann::json;
using nlohmann::json_uri;
using nlohmann::json_schema::json_validator;

static const json rectangle_schema = R"(
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "properties": {
        "width": {
            "$ref": "#/definitions/length",
            "default": 20
        },
        "height": {
            "$ref": "#/definitions/length"
        }
    },
    "definitions": {
        "length": {
            "type": "integer",
            "default": 10
        }
    }
})"_json;

static const json quad_schema = R"(
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "properties": {
        "width": {
            "$ref": "#/properties/height",
            "default": 20
        },
        "height": {
            "$ref": "#/definitions/length"
        },
        "depth": {
            "$ref": "default_schema#/definitions/defaultLength"
        },
		"time": {
            "$ref": "#/definitions/time"
		}
    },
    "definitions": {
        "length": {
            "$ref": "default_schema#/definitions/defaultLength",
            "default": 10
        },
        "time": {
            "type": "integer",
            "default": 15
        }
    }
})"_json;

static const json default_schema = R"(
{
    "$schema": "http://json-schema.org/draft-07/schema#",
    "definitions": {
        "defaultLength": {
            "default": 5
        }
    }
})"_json;

static void loader(const json_uri &uri, json &schema)
{
	schema = default_schema;
}

int main(void)
{
	json_validator validator(loader);

	validator.set_root_schema(quad_schema);

	{
		json empty_quad = R"({})"_json;

		const auto default_patch = validator.validate(empty_quad);
		const auto actual = empty_quad.patch(default_patch);

		const auto expected = R"({"height":10,"width":20,"depth":5,"time":15})"_json;
		if (actual != expected) {
			std::cerr << "Patch with defaults contains wrong value: '" << actual << "' instead of expected '" << expected.dump() << "'" << std::endl;
			return 1;
		}
	}

	validator.set_root_schema(rectangle_schema);

	{
		json empty_rectangle = R"({})"_json;

		const auto default_patch = validator.validate(empty_rectangle);
		const auto actual = empty_rectangle.patch(default_patch);

		// height must be 10 according to the default specified in the length definition while width must be 10 overridden by the width element
		const auto expected = R"({"height":10,"width":20})"_json;
		if (actual != expected) {
			std::cerr << "Patch with defaults contains wrong value: '" << actual << "' instead of expected '" << expected.dump() << "'" << std::endl;
			return 1;
		}
	}

	return 0;
}
