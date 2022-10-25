#include <nlohmann/json.hpp>

#include <nlohmann/json-schema.hpp>

#include <iostream>

using nlohmann::json;

auto schema_draft = R"(
 {
       "$id": "http://example.com/root.json",
       "definitions": {
           "A": { "$id": "#foo" },
           "B": {
               "$id": "other.json",
               "definitions": {
                   "X": { "$id": "#bar" },
                   "Y": { "$id": "t/inner.json" }
               }
           },
           "C": {

               "$id": "urn:uuid:ee564b8a-7a87-4125-8c96-e9f123d6766f",
               "definitions": {
                   "Z": { "$id": "#bar" },
                   "9": { "$id": "http://example.com/drole.json" }
               }
           }
       }
   }
)"_json;

/*  # (document root)

         http://example.com/root.json
         http://example.com/root.json#

   #/definitions/A

         http://example.com/root.json#foo
         http://example.com/root.json#/definitions/A

   #/definitions/B

         http://example.com/other.json
         http://example.com/other.json#
         http://example.com/root.json#/definitions/B

   #/definitions/B/definitions/X

         http://example.com/other.json#bar
         http://example.com/other.json#/definitions/X
         http://example.com/root.json#/definitions/B/definitions/X

   #/definitions/B/definitions/Y

         http://example.com/t/inner.json
         http://example.com/t/inner.json#
         http://example.com/other.json#/definitions/Y
         http://example.com/root.json#/definitions/B/definitions/Y

   #/definitions/C

         urn:uuid:ee564b8a-7a87-4125-8c96-e9f123d6766f
         urn:uuid:ee564b8a-7a87-4125-8c96-e9f123d6766f#
         http://example.com/root.json#/definitions/C
		 */

auto schema = R"(
{
	"id": "http://localhost:1234/scope_change_defs2.json",
	"type" : "object",
	"properties": {
		"list": {"$ref": "#/definitions/baz/definitions/bar"}
	},
	"definitions": {
		"baz": {
			"id": "folder/",
			"definitions": {
				"bar": {
					"type": "array",
					"items": {"$ref": "folderInteger.json"}
				}
			}
		}
	}
})"_json;

class json_schema_validator
{
public:
	std::vector<json> schemas_;
	std::map<nlohmann::json_uri, const json *> schema_store_;

public:
	void insert_schema(json &s, std::vector<nlohmann::json_uri> base_uris)
	{
		auto id = s.find("$id");
		if (id != s.end())
			base_uris.push_back(base_uris.back().derive(id.value()));

		for (auto &u : base_uris)
			schema_store_[u] = &s;

		for (auto i = s.begin();
		     i != s.end();
		     ++i) {

			switch (i.value().type()) {
			case json::value_t::object: { // child is object, thus a schema
				std::vector<nlohmann::json_uri> subschema_uri = base_uris;

				for (auto &ss : subschema_uri)
					ss = ss.append(nlohmann::json_uri::escape(i.key()));

				insert_schema(i.value(), subschema_uri);
			} break;

			case json::value_t::string:
				// this schema is a reference
				if (i.key() == "$ref") {
					auto id = base_uris.back().derive(i.value());
					i.value() = id.to_string();
				}

				break;

			default:
				break;
			}
		}
	}
};

int main(void)
{
	json_schema_validator store;

	store.insert_schema(schema_draft, {{"#"}});

	for (auto &i : store.schema_store_) {
		std::cerr << i.first << " " << *i.second << "\n";
	}

	return 0;
}
