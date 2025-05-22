
#include <pdal/pdal_test_main.hpp>

#include <pdal/FileSpec.hpp>
#include <nlohmann/json.hpp>

using namespace pdal;

TEST(FileSpecTest, create)
{
    // create from string
    std::string inFile = "foo.laz";
    FileSpec spec;
    Utils::StatusWithReason status = spec.ingest(inFile);
    EXPECT_EQ(status, true);
    EXPECT_EQ(spec.m_path.string(), inFile);

    // create from json
    nlohmann::json json{};
    json["path"] = inFile;
    json["headers"] = {{"some_header_key","some_header_val"}};
    json["query"] = {{"some_query_key","some_query_val"}};

    StringMap headersMap = json.at("headers").get<StringMap>();
    StringMap queryMap = json.at("query").get<StringMap>();

    spec = FileSpec();
    // copying this here so the json object can be reused: it gets consumed in parse()
    nlohmann::json testJson = json;
    status = spec.parse(testJson);
    EXPECT_EQ(status, true);
    EXPECT_EQ(spec.m_path.string(), inFile);
    EXPECT_EQ(spec.m_headers, headersMap);
    EXPECT_EQ(spec.m_query, queryMap);
}

TEST(FileSpecTest, parse_errors)
{
    nlohmann::json json{};
    json["path"] = "foo.laz";
    json["headers"] = {{"some_header_key","some_header_val"}};
    json["query"] = {{"some_query_key","some_query_val"}};

    FileSpec spec;
    nlohmann::json testJson = json;
    testJson["query"] = {"some_query_key","some_query_val"};
    Utils::StatusWithReason status = spec.parse(testJson);
    EXPECT_EQ(status.what(), "'filename' sub-argument 'query' must be an object of string key-value pairs.");

    spec = FileSpec();
    testJson = json;
    testJson["path"] = {"foo.laz"};
    status = spec.parse(testJson);
    EXPECT_EQ(status.what(), "'filename' object 'path' member must be specified as a string.");

    spec = FileSpec();
    testJson = json;
    testJson["foo"] = "test";
    status = spec.parse(testJson);
    EXPECT_EQ(status.what(), "Invalid item in filename object: {\"foo\":\"test\"}");

    spec = FileSpec();
    testJson = json;
    testJson.erase("path");
    status = spec.parse(testJson);
    EXPECT_EQ(status.what(), "'filename' object must contain 'path' member.");

    spec = FileSpec();
    testJson = {};
    status = spec.parse(testJson);
    EXPECT_EQ(status.what(), "'filename' argument contains no data");
}
