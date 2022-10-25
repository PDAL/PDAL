#include <nlohmann/json-schema.hpp>

int main(void)
{
	nlohmann::json nlBase{{"$ref", "#/unknown/keywords"}};
	nlohmann::json_schema::json_validator validator;

	try {
		validator.set_root_schema(nlBase); // this line will log the caught exception
	} catch (const std::exception &e) {

		if (std::string("after all files have been parsed, '<root>' has still the following undefined references: [/unknown/keywords]") == e.what())
			return EXIT_SUCCESS;
	}
	return EXIT_FAILURE;
}
