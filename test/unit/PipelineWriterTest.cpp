/******************************************************************************
* Copyright (c) 2019, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/pdal_test_main.hpp>

#include <sstream>

#include "Support.hpp"

#include <pdal/PipelineWriter.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/util/FileUtils.hpp>

#include <nlohmann/json.hpp>

using namespace pdal;

// Make sure we handle duplicate stages properly.
TEST(PipelineManagerTest, issue_2458)
{
    std::string in = R"(
        [
            "in.las",
            "in2.las",
            "out.las"
        ]
    )";

    PipelineManager mgr;
    std::istringstream iss(in);
    mgr.readPipeline(iss);

    std::ostringstream oss;
    PipelineWriter::writePipeline(mgr.getStage(), oss);

    std::string out = oss.str();
    EXPECT_TRUE(out.find("readers_las1") != std::string::npos);
    EXPECT_TRUE(out.find("readers_las2") != std::string::npos);
    EXPECT_TRUE(out.find("writers_las1") != std::string::npos);
}

// Make sure options with Bounds & SpatialReference types are serialized correctly
TEST(PipelineManagerTest, serialize)
{
    std::string inPipeline(Support::configuredpath("pipeline/serialize.json"));
    std::string outPipeline(Support::temppath("serialized_output.json"));

    // We need to read the pipeline, then serialize it before executing
    PipelineManager mgr;
    mgr.readPipeline(inPipeline);
    Stage* stage = mgr.getStage();

    std::ostringstream oss;
    PipelineWriter::writePipeline(stage, oss);
    NL::json root = NL::json::parse(oss.str());

    // reader stage should be at idx 0
    NL::json readerStage = root["pipeline"][0];
    EXPECT_EQ(readerStage.at("type").get<std::string>(), "readers.las");

    NL::json filespecJson = readerStage["filename"];
    EXPECT_TRUE(filespecJson.is_object());
    EXPECT_EQ(filespecJson.at("path").get<std::string>(), 
        Support::datapath("las/epsg_4326.las"));

    // reprojection filter should be at idx 2
    NL::json reproStage = root["pipeline"][2];
    EXPECT_EQ(reproStage.at("type").get<std::string>(), "filters.reprojection");

    NL::json projJson = reproStage["out_srs"];
    EXPECT_TRUE(projJson.is_object());
    EXPECT_EQ(projJson.at("$schema").get<std::string>(), 
        "https://proj.org/schemas/v0.7/projjson.schema.json");
    
    EXPECT_EQ(mgr.execute(), 4775);

    // Make sure the serialized pipeline is valid JSON & creates an identical result
    PipelineManager mgr2;
    std::istringstream iss(oss.str());
    mgr2.readPipeline(iss);
    EXPECT_EQ(mgr2.execute(), 4775);
}