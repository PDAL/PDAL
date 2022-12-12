/******************************************************************************
 * Copyright (c) 2022, Kyle Mann (kyle@hobu.co)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following
 * conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
 *       names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior
 *       written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 ****************************************************************************/

#include <algorithm>
#include <nlohmann/json.hpp>
#include <pdal/pdal_test_main.hpp>

#include <io/StacReader.hpp>
#include <io/EptReader.hpp>

#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/SrsBounds.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>

#include "Support.hpp"

using namespace pdal;


TEST(StacReaderTest, remote_item_test)
{
    Options options;

    options.add("filename", "https://s3-us-west-2.amazonaws.com/usgs-lidar-stac/ept/MD_GoldenBeach_2012.json");
    options.add("asset_names", "ept.json");

    StageFactory f;
    Stage& reader = *f.createStage("readers.stac");
    reader.setOptions(options);

    QuickInfo qi = reader.preview();

    EXPECT_EQ(qi.m_pointCount, 4860658);
}


TEST(StacReaderTest, catalog_test)
{
    Options options;

    options.add("filename", Support::datapath("stac/remote_catalog.json"));
    options.add("asset_names", "ept.json");

    StageFactory f;
    Stage& reader = *f.createStage("readers.stac");
    reader.setOptions(options);

    QuickInfo qi = reader.preview();

    EXPECT_EQ(qi.m_pointCount, 36174643520);
}

TEST(StacReaderTest, nested_catalog_test)
{
    Options options;
    options.add("filename", Support::datapath("stac/multi_type_catalog.json"));
    options.add("catalog_ids", "3dep");
    options.add("item_ids", "MD_GoldenBeach_2012");
    options.add("asset_names", "ept.json");

    StageFactory f;
    Stage& reader = *f.createStage("readers.stac");
    reader.setOptions(options);

    QuickInfo qi = reader.preview();

    EXPECT_EQ(qi.m_pointCount, 4860658);
}

TEST(StacReaderTest, multiple_readers_test)
{
    Options options;
    std::string reader_args = "["
        "{"
            "\"type\": \"readers.ept\", "
            "\"resolution\": 100,"
            "\"bounds\":\"([-10429500, -10429000], [5081800, 5082300])\""
        "},"
        "{"
            "\"type\": \"readers.copc\","
            "\"resolution\": 80"
        "}"
    "]";
    options.add("filename", Support::datapath("stac/multi_type_catalog.json"));
    options.add("reader_args", reader_args);
    options.add("asset_names", "data");
    options.add("asset_names", "ept.json");
    options.add("item_ids", "IA_SouthCentral_1_2020");
    options.add("item_ids", "Autzen Classified");
    options.add("properties", "{\"pc:encoding\": [\"ept\",\"application/vnd.laszip+copc\"]}");

    StageFactory f;
    Stage& reader = *f.createStage("readers.stac");
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();

    EXPECT_EQ(view->size(), 61307);
}

TEST(StacReaderTest, id_prune_test)
{
    Options options;

    options.add("filename", Support::datapath("stac/remote_catalog.json"));
    options.add("item_ids", "MD_GoldenBeach_2012");
    options.add("item_ids", "USGS_LPC\\w{0,}");
    options.add("asset_names", "ept.json");

    StageFactory f;
    Stage& reader = *f.createStage("readers.stac");
    reader.setOptions(options);

    QuickInfo qi = reader.preview();


    NL::json jsonMetadata = NL::json::parse(Utils::toJSON(qi.m_metadata));
    EXPECT_TRUE(jsonMetadata.contains("stac_ids"));
    std::vector<std::string> idList = jsonMetadata["stac_ids"].get<std::vector<std::string>>();

    EXPECT_TRUE(std::find(idList.begin(), idList.end(), "MD_GoldenBeach_2012") != idList.end());
    EXPECT_TRUE(std::find(idList.begin(), idList.end(), "USGS_LPC_AK_Anchorage_2015_LAS_2017") != idList.end());
    EXPECT_TRUE(std::find(idList.begin(), idList.end(), "USGS_LPC_AK_FairbanksNSB_QL1_2017_LAS_2018") != idList.end());
    EXPECT_EQ(qi.m_pointCount, 36134211758);
}

TEST(StacReaderTest, date_prune_accept_test)
{
    //Test using standard datetime measures
    {
    Options options;

    options.add("filename", Support::datapath("stac/MD_GoldenBeach_2012.json"));
    options.add("asset_names", "ept.json");
    options.add("date_ranges", "[\"2022-11-11T0:00:0Z\",\"2022-11-20T0:00:0Z\"]");
    options.add("date_ranges", "[\"2022-05-21T0:00:0Z\",\"2022-05-20T0:00:0Z\"]");

    StageFactory f;
    Stage& reader = *f.createStage("readers.stac");
    reader.setOptions(options);

    QuickInfo qi = reader.preview();

    NL::json jsonMetadata = NL::json::parse(Utils::toJSON(qi.m_metadata));
    EXPECT_TRUE(jsonMetadata.contains("stac_ids"));
    std::vector<std::string> idList = jsonMetadata["stac_ids"].get<std::vector<std::string>>();
    EXPECT_TRUE(std::find(idList.begin(), idList.end(), "MD_GoldenBeach_2012") != idList.end());
    EXPECT_EQ(qi.m_pointCount, 4860658);
    }

    {
    //Test usage of start_datetime and end_datetime
    Options options;

    options.add("filename", Support::datapath("stac/GoldenBeach_datetime_test.json"));
    options.add("asset_names", "ept.json");
    options.add("date_ranges", "[\"2022-11-07T0:00:0Z\",\"2022-11-20T0:00:0Z\"]");

    StageFactory f;
    Stage& reader = *f.createStage("readers.stac");
    reader.setOptions(options);

    QuickInfo qi = reader.preview();

    NL::json jsonMetadata = NL::json::parse(Utils::toJSON(qi.m_metadata));
    EXPECT_TRUE(jsonMetadata.contains("stac_ids"));
    std::vector<std::string> idList = jsonMetadata["stac_ids"].get<std::vector<std::string>>();
    EXPECT_TRUE(std::find(idList.begin(), idList.end(), "MD_GoldenBeach_2012") != idList.end());
    EXPECT_EQ(qi.m_pointCount, 4860658);
    }
}

TEST(StacReaderTest, date_prune_reject_test)
{
    Options options;

    options.add("filename", Support::datapath("stac/MD_GoldenBeach_2012.json"));
    options.add("asset_names", "ept.json");
    options.add("date_ranges", "[\"2022-10-01T0:00:0Z\",\"2022-10-20T0:00:0Z\"]");
    options.add("date_ranges", "[\"2022-05-21T0:00:0Z\",\"2022-05-20T0:00:0Z\"]");

    StageFactory f;
    Stage& reader = *f.createStage("readers.stac");
    reader.setOptions(options);

    EXPECT_THROW(QuickInfo qi = reader.preview(), pdal_error);
}

TEST(StacReaderTest, bounds_prune_accept_test)
{
    Options options;
    std::string bounds = "([-79.0,-74.0],[38.0,39.0])";

    options.add("filename", Support::datapath("stac/MD_GoldenBeach_2012.json"));
    options.add("asset_names", "ept.json");
    options.add("bounds", bounds);

    StageFactory f;
    Stage& reader = *f.createStage("readers.stac");
    reader.setOptions(options);

    QuickInfo qi = reader.preview();

    NL::json jsonMetadata = NL::json::parse(Utils::toJSON(qi.m_metadata));
    EXPECT_TRUE(jsonMetadata.contains("stac_ids"));
    std::vector<std::string> idList = jsonMetadata["stac_ids"].get<std::vector<std::string>>();
    EXPECT_TRUE(std::find(idList.begin(), idList.end(), "MD_GoldenBeach_2012") != idList.end());
    EXPECT_EQ(qi.m_pointCount, 4860658);

}

TEST(StacReaderTest, bounds_prune_reject_test)
{
    Options options;
    std::string bounds = "([50,51],[-10,0])";

    options.add("filename", Support::datapath("stac/MD_GoldenBeach_2012.json"));
    options.add("asset_names", "ept.json");
    options.add("bounds", bounds);

    StageFactory f;
    Stage& reader = *f.createStage("readers.stac");
    reader.setOptions(options);

    EXPECT_THROW(QuickInfo qi = reader.preview(), pdal_error);
}

#ifndef _WIN32
TEST(StacReaderTest, schema_validate_test)
{
    Options options;

    options.add("filename", Support::datapath("stac/MD_GoldenBeach_2012.json"));
    options.add("asset_names", "ept.json");
    options.add("validate_schema", "true");

    StageFactory f;
    Stage& reader = *f.createStage("readers.stac");
    reader.setOptions(options);

    QuickInfo qi = reader.preview();
    EXPECT_TRUE(qi.valid());
    EXPECT_EQ(qi.m_pointCount, 4860658);
}
#endif