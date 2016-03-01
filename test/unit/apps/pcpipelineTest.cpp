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
    int stat = pdal::Utils::run_shell_command(cmd + " " + file, output);
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

    // We used to accept --version as a kernel option, rather than an
    // application option.  Make sure it now throws an error.
    stat = pdal::Utils::run_shell_command(cmd + " --version 2>&1", output);
    EXPECT_TRUE(output.find("Unexpected argument") != std::string::npos);
    EXPECT_NE(stat, 0);
}

TEST(pipelineBaseTest, drop_color)
{ run_pipeline("pipeline/drop_color.xml"); }

TEST(pipelineBaseTest, drop_colorJSON)
{ run_pipeline("pipeline/drop_color.json"); }

TEST(pipelineBaseTest, interpolate)
{ run_pipeline("pipeline/pipeline_interpolate.xml"); }

TEST(pipelineBaseTest, interpolateJSON)
{ run_pipeline("pipeline/pipeline_interpolate.json"); }

TEST(pipelineBaseTest, metadata_reader)
{ run_info("pipeline/pipeline_metadata_reader.xml"); }

TEST(pipelineBaseTest, metadata_readerJSON)
{ run_info("pipeline/pipeline_metadata_reader.json"); }

TEST(pipelineBaseTest, metadata_writer)
{ run_pipeline("pipeline/pipeline_metadata_writer.xml"); }

TEST(pipelineBaseTest, DISABLED_metadata_writerJSON)
{ run_pipeline("pipeline/pipeline_metadata_writer.json"); }

TEST(pipelineBaseTest, mississippi)
{ run_pipeline("pipeline/pipeline_mississippi.xml"); }

TEST(pipelineBaseTest, mississippiJSON)
{ run_pipeline("pipeline/pipeline_mississippi.json"); }

TEST(pipelineBaseTest, mississippi_reverse)
{ run_pipeline("pipeline/pipeline_mississippi_reverse.xml"); }

TEST(pipelineBaseTest, mississippi_reverseJSON)
{ run_pipeline("pipeline/pipeline_mississippi_reverse.json"); }

TEST(pipelineBaseTest, multioptions)
{ run_info("pipeline/pipeline_multioptions.xml"); }

TEST(pipelineBaseTest, multioptionsJSON)
{ run_info("pipeline/pipeline_multioptions.json"); }

TEST(pipelineBaseTest, read)
{ run_info("pipeline/pipeline_read.xml"); }

TEST(pipelineBaseTest, readJSON)
{ run_info("pipeline/pipeline_read.json"); }

TEST(pipelineBaseTest, read_notype)
{ run_info("pipeline/pipeline_read_notype.xml"); }

TEST(pipelineBaseTest, read_notypeJSON)
{ run_info("pipeline/pipeline_read_notype.json"); }

TEST(pipelineBaseTest, readcomments)
{ run_info("pipeline/pipeline_readcomments.xml"); }

TEST(pipelineBaseTest, write)
{ run_pipeline("pipeline/pipeline_write.xml"); }

TEST(pipelineBaseTest, writeJSON)
{ run_pipeline("pipeline/pipeline_write.json"); }

TEST(pipelineBaseTest, write2)
{ run_pipeline("pipeline/pipeline_write2.xml"); }

TEST(pipelineBaseTest, write2JSON)
{ run_pipeline("pipeline/pipeline_write2.json"); }

TEST(pipelineBaseTest, pipeline_writecomments)
{ run_pipeline("pipeline/pipeline_writecomments.xml"); }

TEST(pipelineBpfTest, bpf)
{ run_pipeline("bpf/bpf.xml"); }

TEST(pipelineBpfTest, bpfJSON)
{ run_pipeline("bpf/bpf.json"); }

TEST(pipelineBpfTest, bpf2nitf)
{ run_pipeline("bpf/bpf2nitf.xml"); }

TEST(pipelineBpfTest, DISABLED_bpf2nitfJSON)
{ run_pipeline("bpf/bpf2nitf.json"); }

TEST(pipelineFiltersTest, DISABLED_attribute)
{ run_pipeline("filters/attribute.xml"); }

TEST(pipelineFiltersTest, attributeJSON)
{ run_pipeline("filters/attribute.json"); }

TEST(pipelineFiltersTest, chip)
{ run_pipeline("filters/chip.xml"); }

TEST(pipelineFiltersTest, chipJSON)
{ run_pipeline("filters/chip.json"); }

TEST(pipelineFiltersTest, chipper)
{ run_pipeline("filters/chipper.xml"); }

TEST(pipelineFiltersTest, chipperJSON)
{ run_pipeline("filters/chipper.json"); }

TEST(pipelineFiltersTest, DISABLED_colorize_multi)
{ run_pipeline("filters/colorize-multi.xml"); }

// references a non-existent autzen-warped.tif
TEST(pipelineFiltersTest, DISABLED_colorize_multiJSON)
{ run_pipeline("filters/colorize-multi.json"); }

TEST(pipelineFiltersTest, colorize)
{ run_pipeline("filters/colorize.xml"); }

TEST(pipelineFiltersTest, colorizeJSON)
{ run_pipeline("filters/colorize.json"); }

TEST(pipelineFiltersTest, DISABLED_crop_reproject)
{ run_pipeline("filters/crop_reproject.xml"); }

TEST(pipelineFiltersTest, crop_wkt)
{ run_pipeline("filters/crop_wkt.xml"); }

TEST(pipelineFiltersTest, crop_wktJSON)
{ run_pipeline("filters/crop_wkt.json"); }

TEST(pipelineFiltersTest, crop_wkt_2d)
{ run_pipeline("filters/crop_wkt_2d.xml"); }

TEST(pipelineFiltersTest, crop_wkt_2dJSON)
{ run_pipeline("filters/crop_wkt_2d.json"); }

TEST(pipelineFiltersTest, crop_wkt_2d_classification)
{ run_pipeline("filters/crop_wkt_2d_classification.xml"); }

TEST(pipelineFiltersTest, crop_wkt_2d_classificationJSON)
{ run_pipeline("filters/crop_wkt_2d_classification.json"); }

TEST(pipelineFiltersTest, decimate)
{ run_pipeline("filters/decimate.xml"); }

TEST(pipelineFiltersTest, decimateJSON)
{ run_pipeline("filters/decimate.json"); }

TEST(pipelineFiltersTest, ferry)
{ run_pipeline("filters/ferry.xml"); }

TEST(pipelineFiltersTest, ferryJSON)
{ run_pipeline("filters/ferry.json"); }

TEST(pipelineFiltersTest, hexbin_info)
{ run_info("filters/hexbin-info.xml"); }

TEST(pipelineFiltersTest, hexbin_infoJSON)
{ run_info("filters/hexbin-info.json"); }

TEST(pipelineFiltersTest, hexbin)
{ run_pipeline("filters/hexbin.xml"); }

TEST(pipelineFiltersTest, hexbinJSON)
{ run_pipeline("filters/hexbin.json"); }

TEST(pipelineFiltersTest, merge)
{ run_info("filters/merge.xml"); }

TEST(pipelineFiltersTest, mergeJSON)
{ run_info("filters/merge.json"); }

TEST(pipelineFiltersTest, range_z)
{ run_info("filters/range_z.xml"); }

TEST(pipelineFiltersTest, range_zJSON)
{ run_info("filters/range_z.json"); }

TEST(pipelineFiltersTest, range_z_classification)
{ run_info("filters/range_z_classification.xml"); }

TEST(pipelineFiltersTest, range_z_classificationJSON)
{ run_info("filters/range_z_classification.json"); }

TEST(pipelineFiltersTest, range_classification)
{ run_info("filters/range_classification.xml"); }

TEST(pipelineFiltersTest, range_classificationJSON)
{ run_info("filters/range_classification.json"); }

TEST(pipelineFiltersTest, reproject)
{ run_pipeline("filters/reproject.xml"); }

TEST(pipelineFiltersTest, reprojectJSON)
{ run_pipeline("filters/reproject.json"); }

TEST(pipelineFiltersTest, sort)
{ run_info("filters/sort.xml"); }

TEST(pipelineFiltersTest, sortJSON)
{ run_info("filters/sort.json"); }

TEST(pipelineFiltersTest, splitter)
{ run_pipeline("filters/splitter.xml"); }

TEST(pipelineFiltersTest, splitterJSON)
{ run_pipeline("filters/splitter.json"); }

TEST(pipelineFiltersTest, stats)
{ run_pipeline("filters/stats.xml"); }

TEST(pipelineFiltersTest, statsJSON)
{ run_pipeline("filters/stats.json"); }

TEST(pipelineHoleTest, crop)
{ run_pipeline("hole/crop.xml"); }

TEST(pipelineHoleTest, cropJSON)
{ run_pipeline("hole/crop.json"); }

TEST(pipelineIcebridgeTest, DISABLED_icebridge)
{ run_pipeline("icebridge/pipeline.xml"); }

TEST(pipelineNitfTest, chipper)
{ run_info("nitf/chipper.xml"); }

TEST(pipelineNitfTest, chipperJSON)
{ run_info("nitf/chipper.json"); }

TEST(pipelineNitfTest, conversion)
{ run_pipeline("nitf/conversion.xml"); }

TEST(pipelineNitfTest, conversionJSON)
{ run_pipeline("nitf/conversion.json"); }

TEST(pipelineNitfTest, las2nitf)
{ run_pipeline("nitf/las2nitf.xml"); }

TEST(pipelineNitfTest, DISABLED_las2nitfJSON)
{ run_pipeline("nitf/las2nitf.json"); }

TEST(pipelineNitfTest, DISABLED_reader)
{ run_info("nitf/reader.xml"); }

TEST(pipelineNitfTest, write_laz)
{ run_pipeline("nitf/write_laz.xml"); }

TEST(pipelineNitfTest, write_lazJSON)
{ run_pipeline("nitf/write_laz.json"); }

TEST(pipelineNitfTest, write_options)
{ run_pipeline("nitf/write_options.xml"); }

TEST(pipelineNitfTest, DISABLED_write_optionsJSON)
{ run_pipeline("nitf/write_options.json"); }

// skip oracle tests for now

TEST(pipelineP2gTest, writer)
{ run_pipeline("io/p2g-writer.xml"); }

TEST(pipelineP2gTest, writerJSON)
{ run_pipeline("io/p2g-writer.json"); }

TEST(pipelinePLangTest, DISABLED_from_module)
{ run_info("plang/from-module.xml"); }

TEST(pipelinePLangTest, DISABLED_predicate_embed)
{ run_info("plang/predicate-embed.xml"); }

TEST(pipelinePLangTest, predicate_keep_ground_and_unclass)
{ run_pipeline("plang/predicate-keep-ground-and-unclass.xml"); }

TEST(pipelinePLangTest, predicate_keep_ground_and_unclassJSON)
{ run_pipeline("plang/predicate-keep-ground-and-unclass.json"); }

TEST(pipelinePLangTest, predicate_keep_last_return)
{ run_pipeline("plang/predicate-keep-last-return.xml"); }

TEST(pipelinePLangTest, predicate_keep_last_returnJSON)
{ run_pipeline("plang/predicate-keep-last-return.json"); }

TEST(pipelinePLangTest, predicate_keep_specified_returns)
{ run_pipeline("plang/predicate-keep-specified-returns.xml"); }

TEST(pipelinePLangTest, predicate_keep_specified_returnsJSON)
{ run_pipeline("plang/predicate-keep-specified-returns.json"); }

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

TEST(pipelineSbetTest, pipelineJSON)
{ run_pipeline("sbet/pipeline.json"); }

// skip soci tests for now

TEST(pipelineSQLiteTest, DISABLED_reader)
{ run_pipeline("io/sqlite-reader.xml"); }

TEST(pipelineSQLiteTest, DISABLED_writer)
{ run_pipeline("io/sqlite-writer.xml"); }

TEST(pipelineTextTest, csv_writer)
{ run_pipeline("io/text-writer-csv.xml"); }

TEST(pipelineTextTest, csv_writerJSON)
{ run_pipeline("io/text-writer-csv.json"); }

TEST(pipelineTextTest, geojson_writer)
{ run_pipeline("io/text-writer-geojson.xml"); }

TEST(pipelineTextTest, geojson_writerJSON)
{ run_pipeline("io/text-writer-geojson.json"); }

TEST(pipelineTextTest, space_delimited_writer)
{ run_pipeline("io/text-writer-space-delimited.xml"); }

TEST(pipelineTextTest, space_delimited_writerJSON)
{ run_pipeline("io/text-writer-space-delimited.json"); }
