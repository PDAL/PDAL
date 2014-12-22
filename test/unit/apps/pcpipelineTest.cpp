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

#include "gtest/gtest.h"

#include <pdal/FileUtils.hpp>
#include "Support.hpp"

#include <iostream>
#include <sstream>
#include <string>

static std::string appName()
{
    const std::string app = Support::binpath(Support::exename("pdal") + " pipeline");
    return app;
}

/*
static void test_pipeline(std::string const& pipeline)
{
    const std::string cmd = appName();

    std::string output;
    std::string file(Support::datapath(pipeline));
    int stat = pdal::Utils::run_shell_command(cmd + " " + file, output);
    EXPECT_EQ(0, stat);
    if (stat)
        std::cerr << output << std::endl;
}
*/

static void test_configured_pipeline(std::string const& pipeline)
{
    const std::string cmd = Support::binpath(Support::exename("pdal") + " pipeline");

    std::string output;
    std::string file(Support::configuredpath(pipeline));
    int stat = pdal::Utils::run_shell_command(cmd + " " + file, output);
    EXPECT_EQ(0, stat);
    if (stat)
        std::cerr << output << std::endl;
}

static void test_configured_pipeline_no_writer(std::string const& pipeline)
{
    const std::string cmd = Support::binpath(Support::exename("pdal") + " info");

    std::string output;
    std::string file(Support::configuredpath(pipeline));
    int stat = pdal::Utils::run_shell_command(cmd + " -s < " + file, output);
    EXPECT_EQ(0, stat);
    if (stat)
        std::cerr << output << std::endl;
}

/*
static void test_pipeline_no_writer(std::string const& pipeline)
{
    const std::string cmd = Support::binpath(Support::exename("pdal") + " info");

    std::string output;
    std::string file(Support::datapath(pipeline));
    int stat = pdal::Utils::run_shell_command(cmd + " -s < " + file, output);
    EXPECT_EQ(0, stat);
    if (stat)
        std::cerr << output << std::endl;
}
*/

#ifdef PDAL_COMPILER_MSVC
TEST(pipelineTest, no_input)
{
    const std::string cmd = appName();

    std::string output;
    int stat = pdal::Utils::run_shell_command(cmd, output);
    EXPECT_EQ(stat, 1);

    const std::string expected = "Usage error: input file name required";
    EXPECT_EQ(output.substr(0, expected.length()), expected);
}
#endif


TEST(pipelineTest, common_opts)
{
    const std::string cmd = appName();

    std::string output;
    int stat = pdal::Utils::run_shell_command(cmd + " -h", output);
    EXPECT_EQ(stat, 0);

    stat = pdal::Utils::run_shell_command(cmd + " --version", output);
    EXPECT_EQ(stat, 0);
}

TEST(pipelineBpfTest, bpf)
{ test_configured_pipeline("bpf/bpf.xml"); }

TEST(pipelineBpfTest, bpf2nitf)
{ test_configured_pipeline("bpf/bpf2nitf.xml"); }

TEST(pipelineP2gTest, writer)
{ test_configured_pipeline("io/p2g-writer.xml"); }

TEST(pipelineSQLiteTest, DISABLED_reader)
{ test_configured_pipeline("io/sqlite-reader.xml"); }

TEST(pipelineSQLiteTest, DISABLED_writer)
{ test_configured_pipeline("io/sqlite-writer.xml"); }

TEST(pipelineTextTest, csv_writer)
{ test_configured_pipeline("io/text-writer-csv.xml"); }

TEST(pipelineTextTest, geojson_writer)
{ test_configured_pipeline("io/text-writer-geojson.xml"); }

TEST(pipelineTextTest, space_delimited_writer)
{ test_configured_pipeline("io/text-writer-space-delimited.xml"); }

TEST(pipelineFiltersTest, DISABLED_attribute)
{ test_configured_pipeline("filters/attribute.xml"); }

TEST(pipelineFiltersTest, chip)
{ test_configured_pipeline("filters/chip.xml"); }

TEST(pipelineFiltersTest, chipper)
{ test_configured_pipeline("filters/chipper.xml"); }

TEST(pipelineFiltersTest, DISABLED_colorize_multi)
{ test_configured_pipeline("filters/colorize-multi.xml"); }

TEST(pipelineFiltersTest, DISABLED_colorize)
{ test_configured_pipeline("filters/colorize.xml"); }

TEST(pipelineFiltersTest, DISABLED_crop_reproject)
{ test_configured_pipeline("filters/crop_reproject.xml"); }

TEST(pipelineFiltersTest, crop_wkt)
{ test_configured_pipeline("filters/crop_wkt.xml"); }

TEST(pipelineFiltersTest, crop_wkt_2d)
{ test_configured_pipeline("filters/crop_wkt_2d.xml"); }

TEST(pipelineFiltersTest, crop_wkt_2d_classification)
{ test_configured_pipeline("filters/crop_wkt_2d_classification.xml"); }

TEST(pipelineFiltersTest, DISABLED_decimate)
{ test_configured_pipeline("filters/decimate.xml"); }

TEST(pipelineFiltersTest, ferry)
{ test_configured_pipeline("filters/ferry.xml"); }

TEST(pipelineFiltersTest, hexbin_info)
{ test_configured_pipeline_no_writer("filters/hexbin-info.xml"); }

TEST(pipelineFiltersTest, hexbin)
{ test_configured_pipeline("filters/hexbin.xml"); }

TEST(pipelineFiltersTest, merge)
{ test_configured_pipeline_no_writer("filters/merge.xml"); }

TEST(pipelineFiltersTest, reproject)
{ test_configured_pipeline("filters/reproject.xml"); }

TEST(pipelineFiltersTest, DISABLED_sort)
{ test_configured_pipeline_no_writer("filters/sort.xml"); }

TEST(pipelineFiltersTest, splitter)
{ test_configured_pipeline("filters/splitter.xml"); }

TEST(pipelineFiltersTest, stats)
{ test_configured_pipeline("filters/stats.xml"); }

TEST(pipelineHoleTest, crop)
{ test_configured_pipeline("hole/crop.xml"); }

TEST(pipelineIcebridgeTest, icebridge)
{ test_configured_pipeline("icebridge/pipeline.xml"); }

TEST(pipelineNitfTest, chipper)
{ test_configured_pipeline_no_writer("nitf/chipper.xml"); }

TEST(pipelineNitfTest, conversion)
{ test_configured_pipeline("nitf/conversion.xml"); }

TEST(pipelineNitfTest, las2nitf)
{ test_configured_pipeline("nitf/las2nitf.xml"); }

TEST(pipelineNitfTest, DISABLED_reader)
{ test_configured_pipeline_no_writer("nitf/reader.xml"); }

TEST(pipelineNitfTest, write_laz)
{ test_configured_pipeline("nitf/write_laz.xml"); }

TEST(pipelineNitfTest, write_options)
{ test_configured_pipeline("nitf/write_options.xml"); }

// skip oracle tests for now

TEST(pipelineTest, drop_color)
{ test_configured_pipeline("pipeline/drop_color.xml"); }

TEST(pipelineTest, interpolate)
{ test_configured_pipeline("pipeline/pipeline_interpolate.xml"); }

TEST(pipelineTest, DISABLED_metadata_reader)
{ test_configured_pipeline_no_writer("pipeline/pipeline_metadata_reader.xml"); }

TEST(pipelineTest, metadata_writer)
{ test_configured_pipeline("pipeline/pipeline_metadata_writer.xml"); }

TEST(pipelineTest, mississippi)
{ test_configured_pipeline("pipeline/pipeline_mississippi.xml"); }

TEST(pipelineTest, mississippi_reverse)
{ test_configured_pipeline("pipeline/pipeline_mississippi_reverse.xml"); }

TEST(pipelineTest, multioptions)
{ test_configured_pipeline_no_writer("pipeline/pipeline_multioptions.xml"); }

TEST(pipelineTest, read)
{ test_configured_pipeline_no_writer("pipeline/pipeline_read.xml"); }

TEST(pipelineTest, read_notype)
{ test_configured_pipeline_no_writer("pipeline/pipeline_read_notype.xml"); }

TEST(pipelineTest, readcomments)
{ test_configured_pipeline_no_writer("pipeline/pipeline_readcomments.xml"); }

TEST(pipelineTest, write)
{ test_configured_pipeline("pipeline/pipeline_write.xml"); }

TEST(pipelineTest, write2)
{ test_configured_pipeline("pipeline/pipeline_write2.xml"); }

TEST(pipelineTest, pipeline_writecomments)
{ test_configured_pipeline("pipeline/pipeline_writecomments.xml"); }

TEST(pipelinePLangTest, DISABLED_from_module)
{ test_configured_pipeline_no_writer("plang/from-module.xml"); }

TEST(pipelinePLangTest, DISABLED_predicate_embed)
{ test_configured_pipeline_no_writer("plang/predicate-embed.xml"); }

TEST(pipelinePLangTest, predicate_keep_ground_and_unclass)
{ test_configured_pipeline("plang/predicate-keep-ground-and-unclass.xml"); }

TEST(pipelinePLangTest, predicate_keep_last_return)
{ test_configured_pipeline("plang/predicate-keep-last-return.xml"); }

TEST(pipelinePLangTest, predicate_keep_specified_returns)
{ test_configured_pipeline("plang/predicate-keep-specified-returns.xml"); }

TEST(pipelinePLangTest, DISABLED_programmabled_update_y_dims)
{ test_configured_pipeline_no_writer("plang/programmable-update-y-dims.xml"); }

TEST(pipelineQfitTest, DISABLED_conversion)
{ test_configured_pipeline("qfit/conversion.xml"); }

TEST(pipelineQfitTest, DISABLED_little_endian_conversion)
{ test_configured_pipeline("qfit/little-endian-conversion.xml"); }

TEST(pipelineQfitTest, DISABLED_pipeline)
{ test_configured_pipeline("qfit/pipeline.xml"); }

TEST(pipelineQfitTest, DISABLED_reader)
{ test_configured_pipeline_no_writer("qfit/reader.xml"); }

TEST(pipelineSbetTest, pipeline)
{ test_configured_pipeline("sbet/pipeline.xml"); }

// skip soci tests for now
