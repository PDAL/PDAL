/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>
#include "Support.hpp"

#include <iostream>
#include <sstream>
#include <string>

static std::string appName()
{
    const std::string app = Support::binpath(Support::exename("pdal") + " pipeline");
    return app;
}

// most pipelines (those with a writer) will be invoked via `pdal pipeline`
static void run_pipeline(std::string const& pipeline)
{
    const std::string cmd = Support::binpath(Support::exename("pdal") + " pipeline");

    std::string output;
    std::string file(Support::configuredpath(pipeline));
    int stat = pdal::Utils::run_shell_command(cmd + " " + file, output);
    EXPECT_EQ(0, stat);
    if (stat)
        std::cerr << output << std::endl;
}

// // pipeines with no writer will be invoked via `pdal info`
// static void run_info(std::string const& pipeline)
// {
//     const std::string cmd = Support::binpath(Support::exename("pdal") + " info");
//
//     std::string output;
//     std::string file(Support::configuredpath(pipeline));
//     int stat = pdal::Utils::run_shell_command(cmd + " " + file, output);
//     EXPECT_EQ(0, stat);
//     if (stat)
//         std::cerr << output << std::endl;
// }

#ifdef PDAL_COMPILER_MSVC
TEST(pipelineBaseTest, no_input)
{
    const std::string cmd = appName();

    std::string output;
    int stat = pdal::Utils::run_shell_command(cmd, output);
    EXPECT_EQ(stat, 1);

    const std::string expected = "Usage error: input file name required";
    EXPECT_EQ(output.substr(0, expected.length()), expected);
}
#endif


TEST(pipelineBaseTest, common_opts)
{
    const std::string cmd = appName();

    std::string output;
    int stat = pdal::Utils::run_shell_command(cmd + " -h", output);
    EXPECT_EQ(stat, 0);

    // We used to accept --version as a kernel option, rather than an
    // application option.  Make sure it now throws an error.
    stat = pdal::Utils::run_shell_command(cmd + " --version 2>&1", output);
    EXPECT_TRUE(output.find("Unexpected argument") != std::string::npos);
    EXPECT_NE(stat, 0);
}

class json : public testing::TestWithParam<const char*> {};

TEST_P(json, pipeline)
{ run_pipeline(GetParam()); }

INSTANTIATE_TEST_CASE_P(base, json,
testing::Values(
  "pipeline/crop.json",
  "pipeline/p2g-writer.json",
  "pipeline/pipeline_metadata_reader.json",
  "pipeline/pipeline_metadata_writer.json",
  "pipeline/pipeline_mississippi.json",
  "pipeline/pipeline_mississippi_reverse.json",
  "pipeline/pipeline_multioptions.json",
  "pipeline/pipeline_read.json",
  "pipeline/pipeline_read_notype.json",
  "pipeline/pipeline_write.json",
  "pipeline/pipeline_write2.json"
));

INSTANTIATE_TEST_CASE_P(io, json,
testing::Values(
  "bpf/bpf.json",
  "bpf/bpf2nitf.json",
  "io/text-writer-csv.json",
  "io/text-writer-geojson.json",
  "io/text-writer-space-delimited.json",
  "nitf/chipper.json",
  "nitf/conversion.json",
  "nitf/las2nitf.json",
  "nitf/write_laz.json",
  "nitf/write_options.json",
  "qfit/conversion.json",
  "sbet/pipeline.json"
));

INSTANTIATE_TEST_CASE_P(filters, json,
testing::Values(
  "autzen/autzen-interpolate.json",
  "autzen/hag.json",
  "filters/attribute.json",
  "filters/chip.json",
  "filters/chipper.json",
  "filters/colorize-multi.json",
  "filters/colorize.json",
  "filters/crop_wkt.json",
  "filters/crop_wkt_2d.json",
  "filters/crop_wkt_2d_classification.json",
  "filters/decimate.json",
  "filters/ferry.json",
  "filters/hexbin-info.json",
  "filters/hexbin.json",
  "filters/merge.json",
  "filters/range_z.json",
  "filters/range_z_classification.json",
  "filters/range_classification.json",
  "filters/reproject.json",
  "filters/sort.json",
  "filters/splitter.json",
  "filters/stats.json",
  "hole/crop.json",
  "plang/from-module.json",
  "plang/predicate-embed.json",
  "plang/predicate-keep-ground-and-unclass.json",
  "plang/predicate-keep-last-return.json",
  "plang/predicate-keep-specified-returns.json",
  "plang/programmable-update-y-dims.json"
));

// TEST(pipelineFiltersTest, DISABLED_crop_reproject)
// { run_pipeline("filters/crop_reproject.xml"); }

// TEST(pipelineIcebridgeTest, DISABLED_icebridge)
// { run_pipeline("icebridge/pipeline.xml"); }

// TEST(pipelineNitfTest, DISABLED_reader)
// { run_info("nitf/reader.xml"); }

// skip oracle tests for now

// TEST(pipelineQfitTest, DISABLED_little_endian_conversion)
// { run_pipeline("qfit/little-endian-conversion.xml"); }

// TEST(pipelineQfitTest, DISABLED_pipeline)
// { run_pipeline("qfit/pipeline.xml"); }

// TEST(pipelineQfitTest, DISABLED_reader)
// { run_info("qfit/reader.xml"); }

// skip soci tests for now

// TEST(pipelineSQLiteTest, DISABLED_reader)
// { run_pipeline("io/sqlite-reader.xml"); }

// TEST(pipelineSQLiteTest, DISABLED_writer)
// { run_pipeline("io/sqlite-writer.xml"); }
