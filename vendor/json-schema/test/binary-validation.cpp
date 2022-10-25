// bson-validate.cpp

#include <iostream>
#include <nlohmann/json-schema.hpp>
#include <nlohmann/json.hpp>

static int error_count = 0;

#define EXPECT_EQ(a, b)                                              \
	do {                                                             \
		if (a != b) {                                                \
			std::cerr << "Failed: '" << a << "' != '" << b << "'\n"; \
			error_count++;                                           \
		}                                                            \
	} while (0)

#define EXPECT_THROW(foo)            \
	{                                \
		bool ok = false;             \
		try {                        \
			foo;                     \
		} catch (std::exception &) { \
			ok = true;               \
		}                            \
		if (ok == false) {           \
			error_count++;           \
		}                            \
	}

using json = nlohmann::json;
using validator = nlohmann::json_schema::json_validator;

// check binary data validation
const json bson_schema = json::parse(R"(
{
  "type": "object",
  "properties": {
    "standard_string": {
      "type": "string"
    },
    "binary_data": {
      "type": "string",
      "contentEncoding": "binary"
    }
  },
  "additionalProperties": false
}
)");

const json array_of_types = json::parse(R"(
{
  "type": "object",
  "properties": {
    "something": {
      "type": ["string", "number", "boolean"],
      "contentEncoding": "binary"
    }
  }
}
)");

const json array_of_types_without_binary = json::parse(R"(
{
  "type": "object",
  "properties": {
    "something": {
      "type": ["string", "number", "boolean"]
    }
  }
}
)");

class store_ptr_err_handler : public nlohmann::json_schema::basic_error_handler
{
	void error(const nlohmann::json::json_pointer &ptr, const json &, const std::string &message) override
	{
		nlohmann::json_schema::basic_error_handler::error(ptr, "", message);
		std::cerr << "ERROR: '" << ptr << "' - '"
		          << ""
		          << "': " << message << "\n";
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

static void content(const std::string &contentEncoding, const std::string &contentMediaType, const json &instance)
{
	std::cerr << "mediaType: '" << contentMediaType << "', encoding: '" << contentEncoding << "'\n";

	if (contentEncoding == "binary") {
		if (instance.type() != json::value_t::binary) {
			throw std::invalid_argument{"expected binary data"};
		}
	} else {
		if (instance.type() == json::value_t::binary) {
			throw std::invalid_argument{"expected string, but get binary"};
		}
	}
}

int main()
{
	validator val(nullptr, nullptr, content);

	// create some bson doc
	json::binary_t arr;
	std::string as_binary = "hello world";
	std::copy(as_binary.begin(), as_binary.end(), std::back_inserter(arr));

	json binary = json::binary(arr);

	store_ptr_err_handler err;

	/////////////////////////////////////
	val.set_root_schema(bson_schema);

	// all right
	val.validate({{"standard_string", "some string"}, {"binary_data", binary}}, err);
	EXPECT_EQ(err.failed_pointers.size(), 0);
	err.reset();

	// invalid binary data
	val.validate({{"binary_data", "string, but expect binary data"}}, err);
	EXPECT_EQ(err.failed_pointers.size(), 1);
	EXPECT_EQ(err.failed_pointers[0].to_string(), "/binary_data");
	err.reset();

	// also check that simple string not accept binary data
	val.validate({{"standard_string", binary}, {"binary_data", binary}}, err);
	EXPECT_EQ(err.failed_pointers.size(), 1);
	EXPECT_EQ(err.failed_pointers[0].to_string(), "/standard_string");
	err.reset();

	/////////////////////////////////////
	// check with array of types

	// check simple types
	val.set_root_schema(array_of_types);
	val.validate({{"something", 1}}, err);
	val.validate({{"something", false}}, err);
	// TODO when we set `string` in array and set `contentEncoding` = "binary" - what it means? We expected string or binary?
	// Or we expect only binary? Now if you set `contentEncoding` = "binary", then it means that you expect only binary data,
	// not string
	//val.validate({{"something", "string"}}, err); -> produce error about type
	EXPECT_EQ(err.failed_pointers.size(), 0);
	err.reset();

	// check binary data
	val.validate({{"something", binary}}, err);
	EXPECT_EQ(err.failed_pointers.size(), 0);
	err.reset();

	/////////////////////////////////////
	// and check that you can't set binary data if contentEncoding don't set
	val.set_root_schema(array_of_types_without_binary);
	val.validate({{"something", binary}}, err);
	EXPECT_EQ(err.failed_pointers.size(), 1);
	EXPECT_EQ(err.failed_pointers[0].to_string(), "/something");
	err.reset();

	// check that without content callback you get exception with schema with contentEncoding or contentMeditType
	validator val_no_content;

	EXPECT_THROW(val_no_content.set_root_schema(bson_schema));

	return error_count;
}
