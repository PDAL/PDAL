/******************************************************************************
* Copyright (c) 2019 TileDB, Inc
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#define NOMINMAX

#include <stdio.h>
#include <sys/types.h>

#include <nlohmann/json.hpp>

#include <pdal/pdal_test_main.hpp>
#include <io/FauxReader.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"
#include "../io/TileDBWriter.hpp"

namespace pdal
{
    size_t count = 100;
    class TileDBWriterTest : public ::testing::Test
    {
    protected:
        virtual void SetUp()
        {
            Options options;
            options.add("mode", "ramp");
            options.add("count", count);
            m_reader.setOptions(options);
            m_reader2.setOptions(options);
        }

        FauxReader m_reader;
        FauxReader m_reader2;

    };

    TEST_F(TileDBWriterTest, constructor)
    {
        TileDBWriter writer;
    }

    TEST_F(TileDBWriterTest, findStage)
    {
        StageFactory factory;
        Stage* stage(factory.createStage("writers.tiledb"));
        EXPECT_TRUE(stage);
        EXPECT_TRUE(stage->pipelineStreamable());
    }

    TEST_F(TileDBWriterTest, write)
    {
        tiledb::Context ctx;
        tiledb::VFS vfs(ctx);
        std::string pth = Support::temppath("tiledb_test_out");

        Options options;
        std::string sidecar = pth + "/pdal.json";
        options.add("array_name", pth);
        options.add("chunk_size", 80);

        if (vfs.is_dir(pth))
        {
            vfs.remove_dir(pth);
        }

        TileDBWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);

        // check the sidecar exists
        EXPECT_TRUE(pdal::Utils::fileExists(sidecar));

        tiledb::Array array(ctx, pth, TILEDB_READ);
        auto domain = array.non_empty_domain<double>();
        std::vector<double> subarray;

        for (const auto& kv: domain)
        {
            subarray.push_back(kv.second.first);
            subarray.push_back(kv.second.second);
        }

        tiledb::Query q(ctx, array, TILEDB_READ);
        q.set_subarray(subarray);

        auto max_el = array.max_buffer_elements(subarray);
        std::vector<double> coords(max_el[TILEDB_COORDS].second);
        q.set_coordinates(coords);
        q.submit();
        array.close();

        EXPECT_EQ(m_reader.count() * 3, coords.size());

        ASSERT_DOUBLE_EQ(subarray[0], 0.0);
        ASSERT_DOUBLE_EQ(subarray[2], 0.0);
        ASSERT_DOUBLE_EQ(subarray[4], 0.0);
        ASSERT_DOUBLE_EQ(subarray[1], 1.0);
        ASSERT_DOUBLE_EQ(subarray[3], 1.0);
        ASSERT_DOUBLE_EQ(subarray[5], 1.0);
    }

    TEST_F(TileDBWriterTest, write_append)
    {
        tiledb::Context ctx;
        tiledb::VFS vfs(ctx);
        std::string pth = Support::temppath("tiledb_test_append_out");

        Options options;
        std::string sidecar = pth + "/pdal.json";
        options.add("array_name", pth);
        options.add("chunk_size", 80);

        if (vfs.is_dir(pth))
        {
            vfs.remove_dir(pth);
        }

        TileDBWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);

        // check the sidecar exists so that the execute has completed
        EXPECT_TRUE(pdal::Utils::fileExists(sidecar));

        options.add("append", true);
        TileDBWriter append_writer;
        append_writer.setOptions(options);
        append_writer.setInput(m_reader2);

        FixedPointTable table2(100);
        append_writer.prepare(table2);
        append_writer.execute(table2);

        tiledb::Array array(ctx, pth, TILEDB_READ);
        auto domain = array.non_empty_domain<double>();
        std::vector<double> subarray;

        for (const auto& kv: domain)
        {
            subarray.push_back(kv.second.first);
            subarray.push_back(kv.second.second);
        }

        tiledb::Query q(ctx, array, TILEDB_READ);
        q.set_subarray(subarray);

        auto max_el = array.max_buffer_elements(subarray);
        std::vector<double> coords(max_el[TILEDB_COORDS].second);
        q.set_coordinates(coords);
        q.submit();
        array.close();

        EXPECT_EQ((m_reader.count() * 3) + (m_reader2.count() * 3), coords.size());
    }


    TEST_F(TileDBWriterTest, write_simple_compression)
    {
        tiledb::Context ctx;
        tiledb::VFS vfs(ctx);
        std::string pth = Support::temppath("tiledb_test_compress_simple_out");

        Options options;
        options.add("array_name", pth);
        options.add("compression", "zstd");
        options.add("compression_level", 50);

        if (vfs.is_dir(pth))
        {
            vfs.remove_dir(pth);
        }

        TileDBWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);

        tiledb::Array array(ctx, pth, TILEDB_READ);

        tiledb::FilterList fl = array.schema().coords_filter_list();
        EXPECT_EQ(fl.nfilters(), 1U);

        tiledb::Filter f = fl.filter(0);
        EXPECT_EQ(f.filter_type(), TILEDB_FILTER_ZSTD);
        int32_t compressionLevel;
        f.get_option(TILEDB_COMPRESSION_LEVEL, &compressionLevel);
        EXPECT_EQ(compressionLevel, 50);
    }

    TEST_F(TileDBWriterTest, write_options)
    {
        tiledb::Context ctx;
        tiledb::VFS vfs(ctx);
        std::string pth = Support::temppath("tiledb_test_write_options");

        Options options;
        NL::json jsonOptions;

        // add an array filter
        jsonOptions["coords"] = {
            {{"compression", "bit-shuffle"}},
            {{"compression", "zstd"}, {"compression_level", 50}}
        };
        jsonOptions["OffsetTime"]["compression"] = "rle";

        options.add("array_name", pth);
        options.add("filters", jsonOptions);

        if (vfs.is_dir(pth))
        {
            vfs.remove_dir(pth);
        }

        TileDBWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);

        tiledb::Array array(ctx, pth, TILEDB_READ);

        tiledb::FilterList fl = array.schema().coords_filter_list();
        EXPECT_EQ(fl.nfilters(), 2U);

        tiledb::Filter f1 = fl.filter(0);
        tiledb::Filter f2 = fl.filter(1);
        EXPECT_EQ(f1.filter_type(), TILEDB_FILTER_BITSHUFFLE);
        EXPECT_EQ(f2.filter_type(), TILEDB_FILTER_ZSTD);
        int32_t compressionLevel;
        f2.get_option(TILEDB_COMPRESSION_LEVEL, &compressionLevel);
        EXPECT_EQ(compressionLevel, 50);

        tiledb::Attribute att = array.schema().attributes().begin()->second;
        tiledb::FilterList flAtts = att.filter_list();
        EXPECT_EQ(flAtts.nfilters(), 1U);
        tiledb::Filter fAtt = flAtts.filter(0);
        EXPECT_EQ(fAtt.filter_type(), TILEDB_FILTER_RLE);
    }

   TEST_F(TileDBWriterTest, dup_options)
    {
        tiledb::Context ctx;
        tiledb::VFS vfs(ctx);
        std::string pth = Support::temppath("tiledb_test_write_options");

        Options options;
        NL::json jsonOptions;

        // add an array filter
        jsonOptions["coords"] = {
            {{"compression", "bit-shuffle"}},
            {{"compression", "gzip"}, {"compression_level", 9}}
        };

        options.add("array_name", pth);
        options.add("compression", "zstd");
        options.add("compression_level", 50);
        options.add("filters", jsonOptions);

        if (vfs.is_dir(pth))
        {
            vfs.remove_dir(pth);
        }

        TileDBWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);

        tiledb::Array array(ctx, pth, TILEDB_READ);

        tiledb::FilterList fl = array.schema().coords_filter_list();
        EXPECT_EQ(fl.nfilters(), 1U);

        tiledb::Filter f = fl.filter(0);
        EXPECT_EQ(f.filter_type(), TILEDB_FILTER_ZSTD);
        int32_t compressionLevel;
        f.get_option(TILEDB_COMPRESSION_LEVEL, &compressionLevel);
        EXPECT_EQ(compressionLevel, 50);
    }
}
