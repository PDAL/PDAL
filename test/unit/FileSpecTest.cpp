
#include <pdal/pdal_test_main.hpp>

#include <pdal/FileSpec.hpp>
#include <pdal/private/FileSpecHelper.hpp>
#include <nlohmann/json.hpp>

using namespace pdal;

TEST(FileSpecTest, create)
{
    // create from string
    std::string inFile = "foo.laz";
    FileSpec spec;
    Utils::StatusWithReason status = spec.ingest(inFile);
    EXPECT_EQ(status, true);
    EXPECT_EQ(spec.filePath().string(), inFile);

    // create from json
    NL::json json{};
    json["path"] = inFile;
    json["headers"] = {{"some_header_key","some_header_val"}};
    json["query"] = {{"some_query_key","some_query_val"}};

    StringMap headersMap = json.at("headers").get<StringMap>();
    StringMap queryMap = json.at("query").get<StringMap>();

    spec = FileSpec(json.dump());
    EXPECT_EQ(status, true);
    EXPECT_EQ(spec.filePath().string(), inFile);
    EXPECT_EQ(spec.headers(), headersMap);
    EXPECT_EQ(spec.query(), queryMap);
}

TEST(FileSpecTest, parse_errors)
{
    NL::json json{};
    json["path"] = "foo.laz";
    json["headers"] = {{"some_header_key","some_header_val"}};
    json["query"] = {{"some_query_key","some_query_val"}};

    FileSpec spec;
    NL::json testJson = json;
    testJson["query"] = {"some_query_key","some_query_val"};
    Utils::StatusWithReason status = FileSpecHelper::parse(spec, testJson);
    EXPECT_EQ(status.what(), "'filename' sub-argument 'query' must be an object of string key-value pairs.");

    spec = FileSpec();
    testJson = json;
    testJson["path"] = {"foo.laz"};
    status = FileSpecHelper::parse(spec, testJson);
    EXPECT_EQ(status.what(), "'filename' object 'path' member must be specified as a string.");

    spec = FileSpec();
    testJson = json;
    testJson["foo"] = "test";
    status = FileSpecHelper::parse(spec, testJson);
    EXPECT_EQ(status.what(), "Invalid item in filename object: {\"foo\":\"test\"}");

    spec = FileSpec();
    testJson = json;
    testJson.erase("path");
    status = FileSpecHelper::parse(spec, testJson);
    EXPECT_EQ(status.what(), "'filename' object must contain 'path' member.");

    spec = FileSpec();
    testJson = {};
    status = FileSpecHelper::parse(spec, testJson);
    EXPECT_EQ(status.what(), "'filename' argument contains no data");
}
