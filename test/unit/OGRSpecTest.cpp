#include <pdal/pdal_test_main.hpp>

#include <pdal/private/OGRSpec.hpp>
#include <pdal/util/FileUtils.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(OGRSpecTest, createFromFile)
{
    NL::json json;
    json["type"] = "ogr";
    json["datasource"] = Support::datapath("autzen/attributes.json");
    json["drivers"] = {"GeoJSON"};
    json["openoptions"] = {"FLATTEN_NESTED_ATTRIBUTES=YES","NATIVE_DATA=YES"};
    json["sql"] = "select \"_ogr_geometry_\" from attributes";
    NL::json subJson;
    subJson["dialect"] = "OGRSQL";
    json["options"] = subJson;
    OGRSpec ogr(json);

    std::vector<pdal::Polygon> polys = ogr.getPolygons();

    EXPECT_NEAR(polys[0].area(), 5.235e-06, 0.001);
    EXPECT_NEAR(polys[1].area(), 1.450e-06, 0.001);
    EXPECT_NEAR(polys[2].area(), 5.407e-07, 0.001);
    EXPECT_NEAR(polys[3].area(), 4.516e-06, 0.001);
    EXPECT_NEAR(polys[4].area(), 1.026e-06, 0.001);

    // sql selection
    NL::json updateJson = json;
    updateJson["sql"] = "select \"_ogr_geometry_\" from attributes WHERE id = 1";
    ogr.update(updateJson);
    EXPECT_EQ(ogr.size(), 1);

    // geometry filter
    updateJson = json;
    subJson["geometry"] = "{\"type\":\"Polygon\",\"coordinates\":[[[-123.071871740947586, 44.058426242457685],[-123.070376025800414, 44.058117017731242],[-123.07060216253906, 44.057465769662898],[-123.072144836409578, 44.057837746292243],[-123.071871740947586, 44.058426242457685]]]}";
    updateJson["options"] = subJson;
    OGRSpec ogrGeom(updateJson);
    EXPECT_EQ(ogrGeom.size(), 1);
}

TEST(OGRSpecTest, parseErrors)
{
    NL::json json;
    json["type"] = "ogr";
    json["datasource"] = "no_path";
    json["sql"] ="select \"_ogr_geometry_\" from attributes"; 

    // removing datasource field
    NL::json updateJson = json;
    updateJson.erase("datasource");
    try
    {
        OGRSpec ogr(updateJson);
    }
    catch(OGRSpec::error const& e)
    {
        EXPECT_EQ("'ogr' option must contain a 'datasource' field!",
            std::string(e.what()));
    }

    // invalid type field
    updateJson = json;
    updateJson["type"] = "test";
    try
    {
        OGRSpec ogr(updateJson);
    }
    catch(OGRSpec::error const& e)
    {
        EXPECT_EQ("'ogr' option must have 'type':'ogr' specified!",
            std::string(e.what()));
    }

    // invalid field
    updateJson = json;
    updateJson["foo"] = "test";
    try
    {
        OGRSpec ogr(updateJson);
    }
    catch(OGRSpec::error const& e)
    {
        EXPECT_EQ("unexpected field 'foo' in OGR JSON!", std::string(e.what())); 
    }

    // invalid value
    updateJson = json;
    updateJson["sql"] = "";
    try
    {
        OGRSpec ogr(updateJson);
    }
    catch(OGRSpec::error const& e)
    {
        EXPECT_EQ("invalid value for field 'sql' in OGR JSON!", std::string(e.what()));
    }

    // input json array
    std::stringstream dump;
    dump << "[" << json.dump() << "]";
    try
    {
        OGRSpec ogr(dump.str());
    }
    catch(OGRSpec::error const& e)
    {
        EXPECT_EQ("'ogr' option must be a JSON object with 'type':'ogr' specified!",
            std::string(e.what()));
    }

    // initializing w/ non-json string
    try
    {
        OGRSpec ogr(std::string("invalid json string"));
    }
    catch(OGRSpec::error const& e)
    {
        EXPECT_EQ("Failed to parse OGR JSON with error:  parse error "
            "at line 1, column 1: syntax error while parsing value - "
            "invalid literal; last read: 'i'", std::string(e.what()));
    }
}

