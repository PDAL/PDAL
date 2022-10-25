#include <nlohmann/json-schema.hpp>

#include <fstream>
#include <iostream>

using nlohmann::json;
using nlohmann::json_uri;
using nlohmann::json_schema::json_validator;

static const auto expected_patch = R"(
[{"op":"add","path":"/0/renderable/bg","value":"Black"}]
)"_json;

static const auto instance = R"(
[
    {
        "name":"player",
        "renderable": {
            "fg":"White"
        }
    }
]
)"_json;

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

int main(void)
{
	json_validator validator(loader);

	std::fstream f("blueprints.schema.json");

	json schema;
	f >> schema;

	validator.set_root_schema(schema);

	auto missing_default_patch = validator.validate(instance);

	std::cerr << missing_default_patch << "\n";
	std::cerr << expected_patch << "\n";

	return missing_default_patch != expected_patch;
}
