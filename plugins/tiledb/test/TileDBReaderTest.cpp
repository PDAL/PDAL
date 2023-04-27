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

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <pdal/Filter.hpp>
#include <pdal/pdal_test_main.hpp>
#include <pdal/util/FileUtils.hpp>

#include <io/FauxReader.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"

#include "../io/TileDBReader.hpp"
#include "../io/TileDBWriter.hpp"

namespace pdal
{

const SpatialReference utm16("EPSG:26916");

const size_t count = 100;

class TileDBReaderTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        FauxReader rdr;
        TileDBWriter writer;
        Options writer_options;
        Options reader_options;

        if (FileUtils::directoryExists(data_path))
            FileUtils::deleteDirectory(data_path);

        writer_options.add("array_name", data_path);
        writer_options.add("x_tile_size", 1);
        writer_options.add("y_tile_size", 1);
        writer_options.add("z_tile_size", 1);

        reader_options.add("mode", "ramp");
        reader_options.add("count", count);
        rdr.setOptions(reader_options);

        writer.setOptions(writer_options);
        writer.setInput(rdr);
        writer.setSpatialReference(utm16);

        FixedPointTable table(10);
        writer.prepare(table);
        writer.execute(table);
    }

    std::string data_path = Support::temppath("tiledb_array");
};

TEST_F(TileDBReaderTest, constructor)
{
    TileDBReader reader;
}

TEST_F(TileDBReaderTest, findStage)
{
    StageFactory factory;
    Stage* stage(factory.createStage("readers.tiledb"));
    EXPECT_TRUE(stage);
    EXPECT_TRUE(stage->pipelineStreamable());
}

TEST_F(TileDBReaderTest, read_bbox)
{
    Options options;
    options.add("array_name", data_path);
    options.add("bbox3d", "([0, 0.5], [0, 0.5], [0, 0.5])");

    TileDBReader reader;
    reader.setOptions(options);

    FixedPointTable table(count);
    reader.prepare(table);
    reader.execute(table);
    EXPECT_EQ(table.numPoints(), 50);
}

TEST_F(TileDBReaderTest, read_zero_bbox)
{
    Options options;
    options.add("array_name", data_path);
    options.add("bbox3d", "([1.1, 1.2], [1.1, 1.2], [1.1, 1.2])");

    TileDBReader reader;
    reader.setOptions(options);

    FixedPointTable table(count);
    reader.prepare(table);
    reader.execute(table);
    EXPECT_EQ(table.numPoints(), 0);
}

TEST_F(TileDBReaderTest, read)
{
    class Checker : public Filter, public Streamable
    {
    public:
        std::string getName() const
        {
            return "checker";
        }

    private:
        bool processOne(PointRef& point)
        {
            static int cnt = 0;
            if (cnt == 0)
            {
                EXPECT_NEAR(0.f, point.getFieldAs<double>(Dimension::Id::X),
                            1e-1);
                EXPECT_NEAR(0.f, point.getFieldAs<double>(Dimension::Id::Y),
                            1e-1);
                EXPECT_NEAR(0.f, point.getFieldAs<double>(Dimension::Id::Z),
                            1e-1);
                EXPECT_EQ(0, point.getFieldAs<int>(Dimension::Id::OffsetTime));
            }
            if (cnt == 1)
            {
                EXPECT_NEAR(0.010101f,
                            point.getFieldAs<double>(Dimension::Id::X), 1e-5);
                EXPECT_NEAR(0.010101f,
                            point.getFieldAs<double>(Dimension::Id::Y), 1e-5);
                EXPECT_NEAR(0.010101f,
                            point.getFieldAs<double>(Dimension::Id::Z), 1e-5);
                EXPECT_EQ(1, point.getFieldAs<int>(Dimension::Id::OffsetTime));
            }
            if (cnt == 2)
            {
                EXPECT_NEAR(0.020202f,
                            point.getFieldAs<double>(Dimension::Id::X), 1e-5);
                EXPECT_NEAR(0.020202f,
                            point.getFieldAs<double>(Dimension::Id::Y), 1e-5);
                EXPECT_NEAR(0.020202f,
                            point.getFieldAs<double>(Dimension::Id::Z), 1e-5);
                EXPECT_EQ(2, point.getFieldAs<int>(Dimension::Id::OffsetTime));
            }
            cnt++;
            return true;
        }
    };

    tiledb::Context ctx;
    Options options;
    options.add("array_name", data_path);

    tiledb::Array array(ctx, data_path, TILEDB_READ);
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

    q.set_buffer("X", xs).set_buffer("Y", ys).set_buffer("Z", zs);

    q.submit();
    array.close();
    auto result_num = (int)q.result_buffer_elements()["X"].second;

    TileDBReader reader;
    reader.setOptions(options);

    FixedPointTable table(count);

    Checker c;
    c.setInput(reader);
    c.prepare(table);
    c.execute(table);
    EXPECT_TRUE(reader.getSpatialReference().equals(utm16));
}

TEST_F(TileDBReaderTest, spatial_reference)
{
    std::string pth = Support::temppath("tiledb_test_srs");

    Options options;
    options.add("array_name", pth);
    options.add("chunk_size", 80);

    Options writer_options;
    writer_options.add("array_name", pth);
    writer_options.add("chunk_size", 80);
    writer_options.add("x_tile_size", 1);
    writer_options.add("y_tile_size", 1);
    writer_options.add("z_tile_size", 1);

    if (FileUtils::directoryExists(pth))
        FileUtils::deleteDirectory(pth);

    FauxReader reader;
    Options reader_options;
    reader_options.add("mode", "ramp");
    reader_options.add("count", 50);
    reader.addOptions(reader_options);

    TileDBWriter writer;
    writer.setOptions(writer_options);
    writer.setInput(reader);
    writer.setSpatialReference(utm16);

    FixedPointTable table(count);
    writer.prepare(table);
    writer.execute(table);

    TileDBReader rdr;
    rdr.setOptions(options);
    FixedPointTable table2(count);
    rdr.prepare(table2);
    rdr.execute(table2);
    EXPECT_TRUE(rdr.getSpatialReference().equals(utm16));
};

TEST_F(TileDBReaderTest, unsupported_attribute)
{
    tiledb::Context ctx;
    std::string pth = Support::temppath("tiledb_test_unsupported");
    auto read_schema = tiledb::Array::load_schema(ctx, data_path);

    if (FileUtils::directoryExists(pth))
        FileUtils::deleteDirectory(pth);

    // add a new attribute a1
    auto a1 = tiledb::Attribute::create<std::string>(ctx, "a1");
    read_schema.add_attribute(a1);
    tiledb::Array::create(pth, read_schema);

    // read the schema from the array, and check we skip over the unsupported
    // attributes when strict is false
    TileDBReader rdr;
    Options options;
    options.add("array_name", pth);
    rdr.setOptions(options);
    FixedPointTable table(count);
    EXPECT_THROW(rdr.prepare(table), pdal_error);

    TileDBReader rdr2;
    options.add("strict", false);
    rdr2.setOptions(options);
    EXPECT_NO_THROW(rdr2.prepare(table));
}

}; // namespace pdal
