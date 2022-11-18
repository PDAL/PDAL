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

#include "../io/stacReader.hpp"
#include <io/EptReader.hpp>

#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/SrsBounds.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>

#include "Support.hpp"

using namespace pdal;


// TEST(StacReaderTest, remote_item_test)
// {
//     Options options;

//     options.add("filename", "https://s3-us-west-2.amazonaws.com/usgs-lidar-stac/ept/MD_GoldenBeach_2012.json");
//     options.add("asset_name", "ept.json");

//     StageFactory f;
//     Stage& reader = *f.createStage("readers.stac");
//     reader.setOptions(options);

//     PointTable table;
//     reader.prepare(table);
//     QuickInfo qi = reader.preview();

//     EXPECT_EQ(qi.m_pointCount, 4860658);
// }


// TEST(StacReaderTest, catalog_test)
// {
//     Options options;

//     options.add("filename", Support::datapath("stac/catalog.json"));
//     options.add("asset_name", "ept.json");

//     StageFactory f;
//     Stage& reader = *f.createStage("readers.stac");
//     reader.setOptions(options);

//     PointTable table;
//     reader.prepare(table);
//     QuickInfo qi = reader.preview();

//     EXPECT_EQ(qi.m_pointCount, 36174643520);
// }

TEST(StacReaderTest, id_prune_test)
{
    Options options;

    options.add("filename", Support::datapath("stac/catalog.json"));
    options.add("ids", "MD_GoldenBeach_2012");
    options.add("ids", "USGS_LPC_AK_Anchorage_2015_LAS_2017");
    options.add("asset_name", "ept.json");

    StageFactory f;
    Stage& reader = *f.createStage("readers.stac");
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    QuickInfo qi = reader.preview();
    std::vector<std::string> idList = qi.m_metadata["id"].get<std::vector<std::string>>();

    EXPECT_TRUE(std::find(idList.begin(), idList.end(), "MD_GoldenBeach_2012") != idList.end());
    EXPECT_TRUE(std::find(idList.begin(), idList.end(), "USGS_LPC_AK_Anchorage_2015_LAS_2017") != idList.end());
    EXPECT_EQ(qi.m_pointCount, 12332042294);
}



// TEST(StacReaderTest, dry_run)
// {
//     Options options;
//     options.add("filename", "https://s3-us-west-2.amazonaws.com/usgs-lidar-stac/ept/catalog.json");
//     options.add("asset_name", "ept.json");
//     options.add("dry_run", true);

//     StageFactory f;
//     Stage& reader = *f.createStage("readers.stac");
//     reader.setOptions(options);
// }