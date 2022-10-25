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

int main(void)
{
	json_validator validator;

	validator.set_root_schema(person_schema);
	validator.set_root_schema(person_schema);

	return 0;
}
