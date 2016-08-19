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

#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>
#include "Support.hpp"

#include <iostream>
#include <sstream>
#include <string>

static std::string appName()
{
    const std::string app = Support::binpath(Support::exename("pdal") +
        " pipeline");
    return app;
}

// most pipelines (those with a writer) will be invoked via `pdal pipeline`
static void run_pipeline(std::string const& pipeline)
{
    const std::string cmd = appName();

    std::string output;
    std::string file(Support::configuredpath(pipeline));
    int stat = pdal::Utils::run_shell_command(cmd + " " + file, output);
    EXPECT_EQ(0, stat);
    if (stat)
        std::cerr << output << std::endl;
}

// most pipelines (those with a writer) will be invoked via `pdal pipeline`
static void run_pipeline_stdin(std::string const& pipeline)
{
    const std::string cmd = appName();

    std::string output;
    std::string file(Support::configuredpath(pipeline));
    int stat = pdal::Utils::run_shell_command(cmd + " --stdin < " + file,
        output);
    EXPECT_EQ(0, stat);
    if (stat)
        std::cerr << output << std::endl;
}

#ifdef PDAL_COMPILER_MSVC
TEST(pipelineBaseTest, no_input)
{
    const std::string cmd = appName();

    std::string output;
    int stat = pdal::Utils::run_shell_command(cmd, output);
    EXPECT_EQ(-1, stat);

    const std::string expected = "usage: pdal pipeline [options] [input]";
    EXPECT_EQ(expected, output.substr(0, expected.length()));
}
#endif


TEST(pipelineBaseTest, common_opts)
{
    const std::string cmd = appName();

    std::string output;
    int stat = pdal::Utils::run_shell_command(cmd + " -h", output);
    EXPECT_EQ(stat, 0);
}

class json : public testing::TestWithParam<const char*> {};

TEST_P(json, pipeline)
{
    run_pipeline(GetParam());
}

TEST(json, pipeline_stdin)
{
    run_pipeline_stdin("pipeline/las2csv.json");
    run_pipeline_stdin("pipeline/bpf2las.json");
}

INSTANTIATE_TEST_CASE_P(base, json,
                        testing::Values(
                            // "autzen/autzen-interpolate.json",
                            "pipeline/attribute.json",
                            "pipeline/bpf2las.json",
                            "pipeline/chipper.json",
                            "pipeline/colorize-multi.json",
                            "pipeline/colorize.json",
                            "pipeline/crop-hole.json",
                            "pipeline/crop_wkt.json",
                            "pipeline/crop_wkt_2d.json",
                            "pipeline/decimate.json",
                            "pipeline/ferry-reproject.json",
                            "pipeline/las2csv.json",
                            "pipeline/las2geojson.json",
                            "pipeline/las2space-delimited.json",
                            "pipeline/merge.json",
                            "pipeline/metadata_reader.json",
                            "pipeline/metadata_writer.json",
                            "pipeline/mississippi.json",
                            "pipeline/mississippi_reverse.json",
                            // "pipeline/qfit2las.json",
                            "pipeline/range_z.json",
                            "pipeline/range_z_classification.json",
                            "pipeline/range_classification.json",
                            "pipeline/sbet2txt.json",
                            "pipeline/sort.json",
                            "pipeline/splitter.json",
                            "pipeline/stats.json",
                            "pipeline/transformation.json"
                        ));

class jsonWithNITF : public testing::TestWithParam<const char*> {};

TEST_P(jsonWithNITF, pipeline)
{
    pdal::StageFactory f;
    pdal::Stage* s1 = f.createStage("readers.nitf");
    pdal::Stage* s2 = f.createStage("writers.nitf");
    if (s1 && s2)
        run_pipeline(GetParam());
    else
        std::cerr << "WARNING: could not create readers.nitf or writers.nitf, skipping test" << std::endl;
}


INSTANTIATE_TEST_CASE_P(plugins, jsonWithNITF,
                        testing::Values(
                            "pipeline/bpf2nitf.json",
                            "pipeline/las2nitf.json",
                            "pipeline/las2nitf-2.json",
                            "pipeline/las2nitf-crop-with-options.json",
                            "pipeline/nitf2las.json"
                        ));

class jsonWithP2G : public testing::TestWithParam<const char*> {};

TEST_P(jsonWithP2G, pipeline)
{
    pdal::StageFactory f;
    pdal::Stage* s = f.createStage("writers.p2g");
    if (s)
        run_pipeline(GetParam());
    else
        std::cerr << "WARNING: could not create writers.p2g, skipping test" << std::endl;
}

INSTANTIATE_TEST_CASE_P(plugins, jsonWithP2G,
                        testing::Values(
                            "pipeline/p2g-writer.json"
                        ));

class jsonWithHexer : public testing::TestWithParam<const char*> {};

TEST_P(jsonWithHexer, pipeline)
{
    pdal::StageFactory f;
    pdal::Stage* s = f.createStage("filters.hexbin");
    if (s)
        run_pipeline(GetParam());
    else
        std::cerr << "WARNING: could not create filters.hexbin, skipping test" << std::endl;
}

INSTANTIATE_TEST_CASE_P(plugins, jsonWithHexer,
                        testing::Values(
                            "pipeline/hexbin-info.json",
                            "pipeline/hexbin.json"
                        ));

class jsonWithLAZ : public testing::TestWithParam<const char*> {};

TEST_P(jsonWithLAZ, pipeline)
{
#if defined PDAL_HAVE_LASZIP || defined PDAL_HAVE_LAZPERF
    run_pipeline(GetParam());
#else
    std::cerr << "WARNING: no LAZ support, skipping test" << std::endl;
#endif
}

INSTANTIATE_TEST_CASE_P(plugins, jsonWithLAZ,
                        testing::Values(
                            "pipeline/crop.json",
                            "pipeline/crop-stats.json"
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
