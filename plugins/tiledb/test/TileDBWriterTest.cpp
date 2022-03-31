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

#include <filters/StatsFilter.hpp>
#include <pdal/pdal_test_main.hpp>
#include <io/FauxReader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>

#include "Support.hpp"
#include "../io/TileDBWriter.hpp"

namespace pdal
{

    Options getTileDBOptions()
    {
        Options options;

        options.add("x_tile_size", 1);
        options.add("y_tile_size", 1);
        options.add("z_tile_size", 1);
        options.add("x_domain_st", 0.);
        options.add("x_domain_end", 1.);
        options.add("y_domain_st", 0.);
        options.add("y_domain_end", 1.);
        options.add("z_domain_st", 0.);
        options.add("z_domain_end", 1.);

        return options;
    }

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
        std::string pth = Support::temppath("tiledb_test_out");
        int result_num = 0;

        Options options = getTileDBOptions();
        std::string sidecar = pth + "/pdal.json";
        options.add("array_name", pth);
        options.add("chunk_size", 80);

        if (FileUtils::directoryExists(pth))
            FileUtils::deleteDirectory(pth);

        TileDBWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);

        tiledb::Array array(ctx, pth, TILEDB_READ);

        tiledb_datatype_t v_type = TILEDB_UINT8;
        const void* v_r;
        uint32_t v_num;
        array.get_metadata("_pdal", &v_type, &v_num, &v_r);
        NL::json meta = NL::json::parse(static_cast<const char*>(v_r));
        EXPECT_TRUE(meta.count("writers.tiledb") > 0);

        auto domain = array.non_empty_domain<double>();
        std::vector<double> subarray;

        for (const auto& kv: domain)
        {
            subarray.push_back(kv.second.first);
            subarray.push_back(kv.second.second);
        }

        tiledb::Query q(ctx, array, TILEDB_READ);
        q.set_subarray(subarray);

        std::vector<double> xs(count);
        std::vector<double> ys(count);
        std::vector<double> zs(count);
        
        q.set_buffer("X", xs)
            .set_buffer("Y", ys)
            .set_buffer("Z", zs);

        q.submit();
        array.close();

        result_num = (int)q.result_buffer_elements()["X"].second;

        EXPECT_EQ(m_reader.count(), result_num);

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
        std::string pth = Support::temppath("tiledb_test_append_out");
        int result_num = 0;

        Options options = getTileDBOptions();
        std::string sidecar = pth + "/pdal.json";
        options.add("array_name", pth);
        options.add("chunk_size", 80);

        if (FileUtils::directoryExists(pth))
            FileUtils::deleteDirectory(pth);

        TileDBWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        FixedPointTable table(count);
        writer.prepare(table);
        writer.execute(table);

        options.add("append", true);
        TileDBWriter append_writer;
        append_writer.setOptions(options);
        append_writer.setInput(m_reader2);

        FixedPointTable table2(count);
        append_writer.prepare(table2);
        append_writer.execute(table2);

        tiledb::Array array(ctx, pth, TILEDB_READ);
        auto domain = array.non_empty_domain<double>();
        std::vector<double> subarray;

        tiledb_datatype_t v_type = TILEDB_UINT8;
        const void* v_r;
        uint32_t v_num;
        array.get_metadata("_pdal", &v_type, &v_num, &v_r);
        NL::json meta = NL::json::parse(static_cast<const char*>(v_r));
        EXPECT_TRUE(meta.count("writers.tiledb") > 0);

        for (const auto& kv: domain)
        {
            subarray.push_back(kv.second.first);
            subarray.push_back(kv.second.second);
        }

        tiledb::Query q(ctx, array, TILEDB_READ);
        q.set_subarray(subarray);

        std::vector<double> xs(count * 2);
        std::vector<double> ys(count * 2);
        std::vector<double> zs(count * 2);
        
        q.set_buffer("X", xs)
            .set_buffer("Y", ys)
            .set_buffer("Z", zs);

        q.submit();
        array.close();

        result_num = (int)q.result_buffer_elements()["X"].second;

        EXPECT_EQ(m_reader.count() + m_reader2.count(), result_num);
    }


    TEST_F(TileDBWriterTest, write_simple_compression)
    {
        tiledb::Context ctx;
        std::string pth = Support::temppath("tiledb_test_compress_simple_out");

        Options options = getTileDBOptions();
        options.add("array_name", pth);
        options.add("compression", "zstd");
        options.add("compression_level", 7);


        if (FileUtils::directoryExists(pth))
            FileUtils::deleteDirectory(pth);

        TileDBWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);

        tiledb::Array array(ctx, pth, TILEDB_READ);

        tiledb::FilterList fl = array.schema().domain().dimension("X").filter_list();
        EXPECT_EQ(fl.nfilters(), 1U);

        tiledb::Filter f = fl.filter(0);
        EXPECT_EQ(f.filter_type(), TILEDB_FILTER_ZSTD);
        int32_t compressionLevel;
        f.get_option(TILEDB_COMPRESSION_LEVEL, &compressionLevel);
        EXPECT_EQ(compressionLevel, 7);
    }

    TEST_F(TileDBWriterTest, write_options)
    {
        tiledb::Context ctx;
        std::string pth = Support::temppath("tiledb_test_write_options");

        Options options = getTileDBOptions();
        NL::json jsonOptions;

        // add an array filter
        jsonOptions["X"] = {
            {{"compression", "bit-shuffle"}},
            {{"compression", "zstd"}, {"compression_level", 7}}
        };
        jsonOptions["OffsetTime"]["compression"] = "rle";

        options.add("array_name", pth);
        options.add("filters", jsonOptions);

        if (FileUtils::directoryExists(pth))
            FileUtils::deleteDirectory(pth);

        TileDBWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);

        tiledb::Array array(ctx, pth, TILEDB_READ);

        tiledb::FilterList fl = array.schema().domain().dimension("X").filter_list();
        EXPECT_EQ(fl.nfilters(), 2U);

        tiledb::Filter f1 = fl.filter(0);
        tiledb::Filter f2 = fl.filter(1);
        EXPECT_EQ(f1.filter_type(), TILEDB_FILTER_BITSHUFFLE);
        EXPECT_EQ(f2.filter_type(), TILEDB_FILTER_ZSTD);
        int32_t compressionLevel;
        f2.get_option(TILEDB_COMPRESSION_LEVEL, &compressionLevel);
        EXPECT_EQ(compressionLevel, 7);

        tiledb::Attribute att = array.schema().attributes().begin()->second;
        tiledb::FilterList flAtts = att.filter_list();
        EXPECT_EQ(flAtts.nfilters(), 1U);
        tiledb::Filter fAtt = flAtts.filter(0);
        EXPECT_EQ(fAtt.filter_type(), TILEDB_FILTER_RLE);
    }

   TEST_F(TileDBWriterTest, dup_options)
    {
        tiledb::Context ctx;
        std::string pth = Support::temppath("tiledb_test_write_options");

        Options options = getTileDBOptions();
        NL::json jsonOptions;

        // add an array filter
        jsonOptions["X"] = {
            {{"compression", "bit-shuffle"}},
            {{"compression", "gzip"}, {"compression_level", 9}}
        };

        options.add("array_name", pth);
        options.add("compression", "zstd");
        options.add("compression_level", 7);
        options.add("filters", jsonOptions);

        if (FileUtils::directoryExists(pth))
            FileUtils::deleteDirectory(pth);

        TileDBWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);

        tiledb::Array array(ctx, pth, TILEDB_READ);

        tiledb::FilterList fl = array.schema().domain().dimension("X").filter_list();
        EXPECT_EQ(fl.nfilters(), 2U);

        tiledb::Filter f1 = fl.filter(0);
        tiledb::Filter f2 = fl.filter(1);
        EXPECT_EQ(f1.filter_type(), TILEDB_FILTER_BITSHUFFLE);
        EXPECT_EQ(f2.filter_type(), TILEDB_FILTER_GZIP);
        int32_t compressionLevel;
        f2.get_option(TILEDB_COMPRESSION_LEVEL, &compressionLevel);
        EXPECT_EQ(compressionLevel, 9);
    }

    TEST_F(TileDBWriterTest, default_options)
    {
        tiledb::Context ctx;
        std::string pth = Support::temppath("tiledb_test_write_options");

        Options options = getTileDBOptions();
        options.add("array_name", pth);

        if (FileUtils::directoryExists(pth))
            FileUtils::deleteDirectory(pth);

        TileDBWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);

        tiledb::Array array(ctx, pth, TILEDB_READ);

        tiledb::FilterList fl = array.schema().domain().dimension("X").filter_list();
        EXPECT_EQ(fl.nfilters(), 1U);

        tiledb::Filter f1 = fl.filter(0);
        EXPECT_EQ(f1.filter_type(), TILEDB_FILTER_ZSTD);
        int32_t compressionLevel;
        f1.get_option(TILEDB_COMPRESSION_LEVEL, &compressionLevel);
        EXPECT_EQ(compressionLevel, 7);

        tiledb::Attribute att = array.schema().attributes().begin()->second;
        tiledb::FilterList flAtts = att.filter_list();
        EXPECT_EQ(flAtts.nfilters(), 0U);
    }

    TEST_F(TileDBWriterTest, write_timestamp)
    {
        std::string pth = Support::temppath("tiledb_test_ts_out");

        if (FileUtils::directoryExists(pth))
            FileUtils::deleteDirectory(pth);

        Options options = getTileDBOptions();
        options.add("array_name", pth);
        options.add("timestamp", 1);

        TileDBWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);

        options.replace("timestamp", 2);
        options.add("append", true);
        TileDBWriter append_writer;
        append_writer.setOptions(options);
        append_writer.setInput(m_reader2);

        FixedPointTable table2(100);
        append_writer.prepare(table2);
        append_writer.execute(table2);

        PipelineManager mgr1;
        PipelineManager mgr2;

        Options optsR;
        optsR.add("filename", pth);
        optsR.add("timestamp", 1);
        Stage& rdr1 = mgr1.addReader("readers.tiledb");
        rdr1.setOptions(optsR);
        Stage& filter1 = mgr1.addFilter("filters.stats");
        filter1.setInput(rdr1);
        EXPECT_EQ(100U, mgr1.execute());

        optsR.replace("timestamp", 2);

        Stage& rdr2 = mgr2.addReader("readers.tiledb");
        rdr2.setOptions(optsR);
        Stage& filter2 = mgr2.addFilter("filters.stats");
        filter2.setInput(rdr2);
        EXPECT_EQ(200U, mgr2.execute());

        PipelineManager mgrSlice;
        optsR.remove(Option("timestamp", 2));
        optsR.add("start_timestamp", 2);
        optsR.add("end_timestamp", 2);

        Stage& rdrSlice = mgrSlice.addReader("readers.tiledb");
        rdrSlice.setOptions(optsR);
        Stage& filterSlice = mgrSlice.addFilter("filters.stats");
        filterSlice.setInput(rdrSlice);
        EXPECT_EQ(100U, mgrSlice.execute());
    }

    TEST_F(TileDBWriterTest, dup_points)
    {
        Options reader_options;
        FauxReader reader;
        BOX3D bounds(1.0, 1.0, 1.0, 2.0, 2.0, 2.0);
        reader_options.add("bounds", bounds);
        reader_options.add("mode", "constant");
        reader_options.add("count", count);
        reader.setOptions(reader_options);

        tiledb::Context ctx;
        std::string pth = Support::temppath("tiledb_test_dups");

        Options writer_options = getTileDBOptions();
        writer_options.add("array_name", pth);

        if (FileUtils::directoryExists(pth))
            FileUtils::deleteDirectory(pth);

        TileDBWriter writer;
        writer.setOptions(writer_options);
        writer.setInput(reader);

        FixedPointTable table(count);
        writer.prepare(table);
        writer.execute(table);

        tiledb::Array array(ctx, pth, TILEDB_READ);
        auto domain = array.non_empty_domain<double>();
        std::vector<double> subarray;

        for (const auto& kv: domain)
        {
            subarray.push_back(kv.second.first);
            subarray.push_back(kv.second.second);
        }

        tiledb::Query q(ctx, array, TILEDB_READ);

        // intentionally oversize buffers and check the result count
        std::vector<double> xs(count*2);
        std::vector<double> ys(count*2);
        std::vector<double> zs(count*2);

        q.set_subarray(subarray)
            .set_buffer("X", xs)
            .set_buffer("Y", ys)
            .set_buffer("Z", zs);

        q.submit();
        array.close();

        auto result_num = (int)q.result_buffer_elements()["X"].second;
        EXPECT_EQ(reader.count(), result_num);

        for (int i = 0; i < result_num; i++)
            EXPECT_EQ(xs[i], 1.0); // points are always at the minimum of the box
    }

    TEST_F(TileDBWriterTest, sf_curve)
    {
        Options reader_options;
        FauxReader reader;
        BOX3D bounds(1.0, 1.0, 1.0, 2.0, 2.0, 2.0);
        reader_options.add("bounds", bounds);
        reader_options.add("mode", "constant");
        reader_options.add("count", count);
        reader.setOptions(reader_options);

        tiledb::Context ctx;
        std::string pth = Support::temppath("tiledb_test_sf_curve");

        Options writer_options;
        writer_options.add("array_name", pth);

        if (FileUtils::directoryExists(pth))
            FileUtils::deleteDirectory(pth);

        TileDBWriter writer;
        writer.setOptions(writer_options);
        writer.setInput(reader);

        FixedPointTable table(count);
        writer.prepare(table);

        // check that error is thrown if the tile size or domain is not specified
        EXPECT_THROW(writer.execute(table), pdal_error);
    }

    TEST_F(TileDBWriterTest, tile_sizes)
    {
        Options reader_options;
        FauxReader reader;
        BOX3D bounds(1.0, 1.0, 1.0, 2.0, 2.0, 2.0);
        reader_options.add("bounds", bounds);
        reader_options.add("mode", "constant");
        reader_options.add("count", count);
        reader.setOptions(reader_options);

        tiledb::Context ctx;
        std::string pth = Support::temppath("tiledb_test_sf_curve_ts");

        if (FileUtils::directoryExists(pth))
            FileUtils::deleteDirectory(pth);

        Options writer_options = getTileDBOptions();
        writer_options.add("array_name", pth);

        TileDBWriter writer;
        writer.setOptions(writer_options);
        writer.setInput(reader);

        FixedPointTable table(count);
        writer.prepare(table);
        writer.execute(table);

        EXPECT_EQ(true,
            tiledb::Object::object(ctx, pth).type() == tiledb::Object::Type::Array);

        tiledb::Array array(ctx, pth, TILEDB_READ);
        EXPECT_EQ(true,
            array.schema().cell_order() == TILEDB_ROW_MAJOR);
        array.close();
    }

    TEST_F(TileDBWriterTest, sf_curve_stats)
    {
        tiledb::Context ctx;
        std::string pth = Support::temppath("tiledb_test_sf_curve_stats");

        if (FileUtils::directoryExists(pth))
            FileUtils::deleteDirectory(pth);

        PipelineManager mgr;

        Options optsR;
        optsR.add("filename", Support::datapath("las/1.2-with-color.las"));
        Stage& reader = mgr.addReader("readers.las");
        reader.setOptions(optsR);

        Stage& filter = mgr.addFilter("filters.stats");
        filter.setInput(reader);

        Options optsW;
        optsW.add("array_name", pth);
        Stage& writer = mgr.addWriter("writers.tiledb");
        writer.setInput(filter);
        writer.setOptions(optsW);

        point_count_t np = mgr.execute();
        EXPECT_EQ(1065U, np);

        tiledb::Array array(ctx, pth, TILEDB_READ);
        EXPECT_EQ(true,
            array.schema().cell_order() == TILEDB_HILBERT);

        tiledb_datatype_t v_type = TILEDB_UINT8;
        const void* v_r;
        uint32_t v_num;
        array.get_metadata("_pdal", &v_type, &v_num, &v_r);
        NL::json metaDoc = NL::json::parse(static_cast<const char*>(v_r));

        EXPECT_TRUE(metaDoc.contains("filters.stats"));
        EXPECT_TRUE(metaDoc.contains("writers.tiledb"));

        array.close();
    }
}
