/******************************************************************************
 * Copyright (c) 2021 TileDB, Inc.
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

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <pdal/pdal_test_main.hpp>
#include <pdal/util/FileUtils.hpp>
#include <stdio.h>

#include "Support.hpp"

#include "../io/TileDBReader.hpp"
#include "../io/TileDBUtils.hpp"
#include "../io/TileDBWriter.hpp"
#include "XYZTmUtils.hpp"

namespace pdal
{

Options getTileDBOptions()
{
    Options options;

    options.add("x_domain_st", 0);
    options.add("x_domain_end", 10);
    options.add("y_domain_st", 0);
    options.add("y_domain_end", 10);
    options.add("z_domain_st", 0);
    options.add("z_domain_end", 10);
    options.add("time_domain_st", 0);
    options.add("time_domain_end", 10);
    options.add("x_tile_size", 10);
    options.add("y_tile_size", 10);
    options.add("z_tile_size", 10);
    options.add("time_tile_size", 10);

    return options;
}

const size_t count = 100;
DomainBounds rdr_bounds(0, 0, 0, 0, 10, 10, 10, 10);

class TileDBWriterTimeDimTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        tiledb::Context ctx;

        data_path = Support::temppath("xyztm_tdb_array");

        if (Utils::fileExists(data_path))
        {
            tiledb::Object::remove(ctx, data_path);
        }

        Options options;
        options.add("count", count);
        options.add("bounds", rdr_bounds);
        options.add("xyz_mode", "ramp");

        m_reader.setOptions(options);
        m_reader2.setOptions(options);
    }
    std::string data_path;
    XYZTimeFauxReader m_reader;
    XYZTimeFauxReader m_reader2;
};

TEST_F(TileDBWriterTimeDimTest, test_dims_no_tm_and_use_tm)
{
    tiledb::Context ctx;

    std::string out_path1 = Support::temppath("xyztm_tdb_no_tm");

    if (FileUtils::directoryExists(out_path1))
        FileUtils::deleteDirectory(out_path1);

    Options options1 = getTileDBOptions();
    options1.add("array_name", out_path1);

    TileDBWriter writer1;
    writer1.setOptions(options1);
    writer1.setInput(m_reader);

    FixedPointTable table1(count);
    writer1.prepare(table1);
    writer1.execute(table1);

    tiledb::Array array_no_time(ctx, out_path1, TILEDB_READ);
    auto domain_no_time = array_no_time.non_empty_domain<double>();

    EXPECT_EQ(domain_no_time.size(), 3);

    std::string out_path2 = Support::temppath("xyztm_tdb_use_tm");

    if (FileUtils::directoryExists(out_path2))
        FileUtils::deleteDirectory(out_path2);

    Options options2 = getTileDBOptions();
    options2.add("array_name", out_path2);
    options2.add("use_time_dim", true);

    TileDBWriter writer2;
    writer2.setOptions(options2);
    writer2.setInput(m_reader);

    FixedPointTable table2(count);
    writer2.prepare(table2);
    writer2.execute(table2);

    tiledb::Array array_with_time(ctx, out_path2, TILEDB_READ);
    auto domain_with_time = array_with_time.non_empty_domain<double>();

    EXPECT_EQ(domain_with_time.size(), 4);

    std::vector<std::string> dim_names{"X", "Y", "Z", "GpsTime"};

    size_t idx;
    for (idx = 0; idx != 3; idx++)
    {
        EXPECT_EQ(dim_names.at(idx), domain_no_time.at(idx).first);
    }

    idx = 0;
    for (const auto& name : dim_names)
    {
        EXPECT_EQ(name, domain_with_time.at(idx++).first);
    }
}

TEST_F(TileDBWriterTimeDimTest, use_time_synonym)
{
    tiledb::Context ctx;
    std::string out_path = Support::temppath("xyztm_tdb_array");

    if (FileUtils::directoryExists(out_path))
        FileUtils::deleteDirectory(out_path);

    Options options = getTileDBOptions();
    options.add("array_name", out_path);
    options.add("use_time", true);

    TileDBWriter writer;
    writer.setOptions(options);
    writer.setInput(m_reader);

    FixedPointTable table(count);
    writer.prepare(table);
    writer.execute(table);

    tiledb::Array array(ctx, out_path, TILEDB_READ);
    auto domain = array.non_empty_domain<double>();
    EXPECT_EQ(domain.size(), 4);
    EXPECT_EQ(domain.at(3).first, "GpsTime");
}

TEST_F(TileDBWriterTimeDimTest, write_with_time)
{
    tiledb::Context ctx;
    std::string out_path = Support::temppath("xyztm_tdb_arr_write");
    int result_num = 0;

    Options options = getTileDBOptions();
    options.add("array_name", out_path);
    options.add("chunk_size", 80);
    options.add("use_time_dim", true);

    if (FileUtils::directoryExists(out_path))
        FileUtils::deleteDirectory(out_path);

    TileDBWriter writer;
    writer.setOptions(options);
    writer.setInput(m_reader);

    FixedPointTable table(count);
    writer.prepare(table);
    writer.execute(table);

    tiledb::Array array(ctx, out_path, TILEDB_READ);
    auto domain = array.non_empty_domain<double>();
    std::vector<double> subarray;

    for (const auto& kv : domain)
    {
        subarray.push_back(kv.second.first);
        subarray.push_back(kv.second.second);
    }

    tiledb::Query q(ctx, array, TILEDB_READ);
    q.set_subarray(subarray);

    std::vector<double> xs(count);
    std::vector<double> ys(count);
    std::vector<double> zs(count);
    std::vector<double> ts(count);

    q.set_buffer("X", xs).set_buffer("Y", ys).set_buffer("Z", zs).set_buffer(
        "GpsTime", ts);

    q.submit();
    array.close();

    result_num = (int)q.result_buffer_elements()["X"].second;

    EXPECT_EQ(m_reader.count(), result_num);

    ASSERT_DOUBLE_EQ(subarray[0], 0.0);
    ASSERT_DOUBLE_EQ(subarray[2], 0.0);
    ASSERT_DOUBLE_EQ(subarray[4], 0.0);
    ASSERT_DOUBLE_EQ(subarray[6], 0.0);
    ASSERT_DOUBLE_EQ(subarray[1], 10.0);
    ASSERT_DOUBLE_EQ(subarray[3], 10.0);
    ASSERT_DOUBLE_EQ(subarray[5], 10.0);
    ASSERT_DOUBLE_EQ(subarray[7], 10.0);
}

TEST_F(TileDBWriterTimeDimTest, write_append_with_time)
{
    tiledb::Context ctx;
    std::string out_path = Support::temppath("xyztm_tdb_test_append");
    int result_num = 0;

    Options options = getTileDBOptions();
    options.add("array_name", out_path);
    options.add("chunk_size", 80);
    options.add("use_time_dim", true);

    if (FileUtils::directoryExists(out_path))
        FileUtils::deleteDirectory(out_path);

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

    tiledb::Array array(ctx, out_path, TILEDB_READ);
    auto domain = array.non_empty_domain<double>();
    std::vector<double> subarray;

    for (const auto& kv : domain)
    {
        subarray.push_back(kv.second.first);
        subarray.push_back(kv.second.second);
    }

    tiledb::Query q(ctx, array, TILEDB_READ);
    q.set_subarray(subarray);

    std::vector<double> xs(count * 2);
    std::vector<double> ys(count * 2);
    std::vector<double> zs(count * 2);
    std::vector<double> ts(count * 2);

    q.set_buffer("X", xs).set_buffer("Y", ys).set_buffer("Z", zs).set_buffer(
        "GpsTime", ts);

    q.submit();
    array.close();

    result_num = (int)q.result_buffer_elements()["X"].second;
    EXPECT_EQ(m_reader.count() + m_reader2.count(), result_num);
}

TEST_F(TileDBWriterTimeDimTest, write_options_with_time_dim)
{
    tiledb::Context ctx;

    std::string out_path = Support::temppath("xyztm_tdb_writer_options");

    if (FileUtils::directoryExists(out_path))
        FileUtils::deleteDirectory(out_path);

    Options options = getTileDBOptions();
    options.add("array_name", data_path);
    options.add("use_time_dim", true);

    TileDBWriter writer1;
    writer1.setOptions(options);
    writer1.setInput(m_reader);

    FixedPointTable table1(count);
    writer1.prepare(table1);
    writer1.execute(table1);

    Options reader_options;
    reader_options.add("array_name", data_path);

    TileDBReader reader;
    reader.setOptions(reader_options);

    Options writer_options = getTileDBOptions();
    writer_options.add("array_name", out_path);
    writer_options.add("use_time_dim", true);

    TileDBWriter writer2;
    writer2.setOptions(writer_options);
    writer2.setInput(reader);

    FixedPointTable table2(count);
    writer2.prepare(table2);
    writer2.execute(table2);

    tiledb::Array array(ctx, out_path, TILEDB_READ);
    auto domain = array.non_empty_domain<double>();
    EXPECT_EQ(domain.size(), 4);
}

TEST_F(TileDBWriterTimeDimTest, time_first_or_last)
{
    tiledb::Context ctx;

    std::string out_path1 = Support::temppath("xyztm_tm_first");

    if (FileUtils::directoryExists(out_path1))
        FileUtils::deleteDirectory(out_path1);

    Options options1 = getTileDBOptions();
    options1.add("array_name", out_path1);
    options1.add("use_time_dim", true);
    options1.add("time_first", true);

    TileDBWriter writer1;
    writer1.setOptions(options1);
    writer1.setInput(m_reader);

    FixedPointTable table1(count);
    writer1.prepare(table1);
    writer1.execute(table1);

    tiledb::Array array1(ctx, out_path1, TILEDB_READ);
    auto domain = array1.non_empty_domain<double>();
    std::string first_dim_name = domain.at(0).first;

    EXPECT_EQ(first_dim_name, "GpsTime");

    std::string out_path2 = Support::temppath("xyztm_tm_last");

    if (FileUtils::directoryExists(out_path2))
        FileUtils::deleteDirectory(out_path2);

    Options options2 = getTileDBOptions();
    options2.add("array_name", out_path2);
    options2.add("use_time_dim", true);

    TileDBWriter writer2;
    writer2.setOptions(options2);
    writer2.setInput(m_reader);

    FixedPointTable table2(count);
    writer2.prepare(table2);
    writer2.execute(table2);

    tiledb::Array array2(ctx, out_path2, TILEDB_READ);
    auto domain2 = array2.non_empty_domain<double>();
    std::string last_dim_name = domain2.at(3).first;

    EXPECT_EQ(last_dim_name, "GpsTime");
}

TEST_F(TileDBWriterTimeDimTest, append_write_with_time)
{
    tiledb::Context ctx;
    int result_num = 0;

    std::string out_path = Support::temppath("xyztm_append");

    if (FileUtils::directoryExists(out_path))
        FileUtils::deleteDirectory(out_path);

    Options options1 = getTileDBOptions();
    options1.add("array_name", out_path);
    options1.add("use_time_dim", true);

    TileDBWriter writer1;
    writer1.setOptions(options1);
    writer1.setInput(m_reader);

    FixedPointTable table1(count);
    writer1.prepare(table1);
    writer1.execute(table1);

    Options reader_options;
    reader_options.add("count", count);
    reader_options.add("xyz_mode", "ramp");
    reader_options.add("bounds", rdr_bounds);
    reader_options.add("density", 2.0);
    reader_options.add("use_time", false);

    XYZTimeFauxReader reader;
    reader.setOptions(reader_options);

    Options options2 = getTileDBOptions();
    options2.add("array_name", out_path);
    options2.add("append", true);

    TileDBWriter writer2;
    writer2.setOptions(options2);
    writer2.setInput(reader);

    FixedPointTable table2(count);
    writer2.prepare(table2);
    writer2.execute(table2);

    tiledb::Array array(ctx, out_path, TILEDB_READ);
    auto domain = array.non_empty_domain<double>();

    std::vector<double> subarray;
    for (const auto& kv : domain)
    {
        subarray.push_back(kv.second.first);
        subarray.push_back(kv.second.second);
    }

    tiledb::Query q(ctx, array, TILEDB_READ);
    q.set_subarray(subarray);

    std::vector<double> xs(count * 2);
    std::vector<double> ys(count * 2);
    std::vector<double> zs(count * 2);
    std::vector<double> ts(count * 2);

    q.set_buffer("X", xs).set_buffer("Y", ys).set_buffer("Z", zs).set_buffer(
        "GpsTime", ts);

    q.submit();
    array.close();

    result_num = (int)q.result_buffer_elements()["X"].second;

    EXPECT_EQ(result_num, reader.count() + m_reader.count());
}

} // namespace pdal
