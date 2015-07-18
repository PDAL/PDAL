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

// pipeines with no writer will be invoked via `pdal info`
static void run_info(std::string const& pipeline)
{
    const std::string cmd = Support::binpath(Support::exename("pdal") + " info");

    std::string output;
    std::string file(Support::configuredpath(pipeline));
    int stat = pdal::Utils::run_shell_command(cmd + " -s < " + file, output);
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

    stat = pdal::Utils::run_shell_command(cmd + " --version", output);
    EXPECT_EQ(stat, 0);
}

TEST(pipelineBaseTest, drop_color)
{ run_pipeline("pipeline/drop_color.xml"); }

TEST(pipelineBaseTest, interpolate)
{ run_pipeline("pipeline/pipeline_interpolate.xml"); }

TEST(pipelineBaseTest, DISABLED_metadata_reader)
{ run_info("pipeline/pipeline_metadata_reader.xml"); }

TEST(pipelineBaseTest, metadata_writer)
{ run_pipeline("pipeline/pipeline_metadata_writer.xml"); }

TEST(pipelineBaseTest, mississippi)
{ run_pipeline("pipeline/pipeline_mississippi.xml"); }

TEST(pipelineBaseTest, mississippi_reverse)
{ run_pipeline("pipeline/pipeline_mississippi_reverse.xml"); }

TEST(pipelineBaseTest, multioptions)
{ run_info("pipeline/pipeline_multioptions.xml"); }

TEST(pipelineBaseTest, read)
{ run_info("pipeline/pipeline_read.xml"); }

TEST(pipelineBaseTest, read_notype)
{ run_info("pipeline/pipeline_read_notype.xml"); }

TEST(pipelineBaseTest, readcomments)
{ run_info("pipeline/pipeline_readcomments.xml"); }

TEST(pipelineBaseTest, write)
{ run_pipeline("pipeline/pipeline_write.xml"); }

TEST(pipelineBaseTest, write2)
{ run_pipeline("pipeline/pipeline_write2.xml"); }

TEST(pipelineBaseTest, pipeline_writecomments)
{ run_pipeline("pipeline/pipeline_writecomments.xml"); }

TEST(pipelineBpfTest, bpf)
{ run_pipeline("bpf/bpf.xml"); }

TEST(pipelineBpfTest, bpf2nitf)
{ run_pipeline("bpf/bpf2nitf.xml"); }

TEST(pipelineFiltersTest, DISABLED_attribute)
{ run_pipeline("filters/attribute.xml"); }

TEST(pipelineFiltersTest, chip)
{ run_pipeline("filters/chip.xml"); }

TEST(pipelineFiltersTest, chipper)
{ run_pipeline("filters/chipper.xml"); }

TEST(pipelineFiltersTest, DISABLED_colorize_multi)
{ run_pipeline("filters/colorize-multi.xml"); }

TEST(pipelineFiltersTest, colorize)
{ run_pipeline("filters/colorize.xml"); }

TEST(pipelineFiltersTest, DISABLED_crop_reproject)
{ run_pipeline("filters/crop_reproject.xml"); }

TEST(pipelineFiltersTest, crop_wkt)
{ run_pipeline("filters/crop_wkt.xml"); }

TEST(pipelineFiltersTest, crop_wkt_2d)
{ run_pipeline("filters/crop_wkt_2d.xml"); }

TEST(pipelineFiltersTest, crop_wkt_2d_classification)
{ run_pipeline("filters/crop_wkt_2d_classification.xml"); }

TEST(pipelineFiltersTest, DISABLED_decimate)
{ run_pipeline("filters/decimate.xml"); }

TEST(pipelineFiltersTest, ferry)
{ run_pipeline("filters/ferry.xml"); }

TEST(pipelineFiltersTest, hexbin_info)
{ run_info("filters/hexbin-info.xml"); }

TEST(pipelineFiltersTest, hexbin)
{ run_pipeline("filters/hexbin.xml"); }

TEST(pipelineFiltersTest, merge)
{ run_info("filters/merge.xml"); }

TEST(pipelineFiltersTest, range_z)
{ run_info("filters/range_z.xml"); }

TEST(pipelineFiltersTest, range_z_classification)
{ run_info("filters/range_z_classification.xml"); }

TEST(pipelineFiltersTest, range_classification)
{ run_info("filters/range_classification.xml"); }

TEST(pipelineFiltersTest, reproject)
{ run_pipeline("filters/reproject.xml"); }

TEST(pipelineFiltersTest, DISABLED_sort)
{ run_info("filters/sort.xml"); }

TEST(pipelineFiltersTest, splitter)
{ run_pipeline("filters/splitter.xml"); }

TEST(pipelineFiltersTest, stats)
{ run_pipeline("filters/stats.xml"); }

TEST(pipelineHoleTest, crop)
{ run_pipeline("hole/crop.xml"); }

TEST(pipelineIcebridgeTest, icebridge)
{ run_pipeline("icebridge/pipeline.xml"); }

TEST(pipelineNitfTest, chipper)
{ run_info("nitf/chipper.xml"); }

TEST(pipelineNitfTest, conversion)
{ run_pipeline("nitf/conversion.xml"); }

TEST(pipelineNitfTest, las2nitf)
{ run_pipeline("nitf/las2nitf.xml"); }

TEST(pipelineNitfTest, DISABLED_reader)
{ run_info("nitf/reader.xml"); }

TEST(pipelineNitfTest, write_laz)
{ run_pipeline("nitf/write_laz.xml"); }

TEST(pipelineNitfTest, write_options)
{ run_pipeline("nitf/write_options.xml"); }

// skip oracle tests for now

TEST(pipelineP2gTest, writer)
{ run_pipeline("io/p2g-writer.xml"); }

TEST(pipelinePLangTest, DISABLED_from_module)
{ run_info("plang/from-module.xml"); }

TEST(pipelinePLangTest, DISABLED_predicate_embed)
{ run_info("plang/predicate-embed.xml"); }

TEST(pipelinePLangTest, predicate_keep_ground_and_unclass)
{ run_pipeline("plang/predicate-keep-ground-and-unclass.xml"); }

TEST(pipelinePLangTest, predicate_keep_last_return)
{ run_pipeline("plang/predicate-keep-last-return.xml"); }

TEST(pipelinePLangTest, predicate_keep_specified_returns)
{ run_pipeline("plang/predicate-keep-specified-returns.xml"); }

TEST(pipelinePLangTest, DISABLED_programmabled_update_y_dims)
{ run_info("plang/programmable-update-y-dims.xml"); }

TEST(pipelineQfitTest, DISABLED_conversion)
{ run_pipeline("qfit/conversion.xml"); }

TEST(pipelineQfitTest, DISABLED_little_endian_conversion)
{ run_pipeline("qfit/little-endian-conversion.xml"); }

TEST(pipelineQfitTest, DISABLED_pipeline)
{ run_pipeline("qfit/pipeline.xml"); }

TEST(pipelineQfitTest, DISABLED_reader)
{ run_info("qfit/reader.xml"); }

TEST(pipelineSbetTest, pipeline)
{ run_pipeline("sbet/pipeline.xml"); }

// skip soci tests for now

TEST(pipelineSQLiteTest, DISABLED_reader)
{ run_pipeline("io/sqlite-reader.xml"); }

TEST(pipelineSQLiteTest, DISABLED_writer)
{ run_pipeline("io/sqlite-writer.xml"); }

TEST(pipelineTextTest, csv_writer)
{ run_pipeline("io/text-writer-csv.xml"); }

TEST(pipelineTextTest, geojson_writer)
{ run_pipeline("io/text-writer-geojson.xml"); }

TEST(pipelineTextTest, space_delimited_writer)
{ run_pipeline("io/text-writer-space-delimited.xml"); }
