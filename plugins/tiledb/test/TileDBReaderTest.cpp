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

#include <nlohmann/json.hpp>

#include <io/BufferReader.hpp>
#include <io/FauxReader.hpp>
#include <pdal/Filter.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_test_main.hpp>
#include <pdal/util/FileUtils.hpp>

#if defined(DELETE)
#undef DELETE
#endif


#include <tiledb/tiledb>

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
        writer_options.add("scale_x", 1.0e-9);
        writer_options.add("scale_y", 1.0e-9);
        writer_options.add("scale_z", 1.0e-9);

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

    tiledb_datatype_t v_type = (tiledb_datatype_t)std::numeric_limits<int32_t>::max();
    EXPECT_TRUE(array.has_metadata("dataset_type", &v_type));
    EXPECT_EQ(v_type, TILEDB_STRING_UTF8);

    tiledb::Query q(ctx, array, TILEDB_READ);

    std::vector<double> xs(count);
    std::vector<double> ys(count);
    std::vector<double> zs(count);

    q.set_data_buffer("X", xs).set_data_buffer("Y", ys).set_data_buffer("Z",
                                                                        zs);

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

TEST(TileDBRoundTripTest, delta_filter_test)
{
    // Raw data.
    std::string uri = Support::temppath("tiledb_test_intensity_compression");
    std::vector<double> xValues{0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> yValues(6, 0.0);
    std::vector<double> zValues(6, 0.0);
    std::vector<uint16_t> intensityValues{1, 1, 0, 0, 2, 3};
    std::vector<double> gpsTimeValues{1306604063.0, 1306604064.0, 1306604078.0,
                                      1306604064.0, 1306604064.0, 1306603982.0};

    // Clean-up previous tests.
    if (FileUtils::directoryExists(uri))
        FileUtils::deleteDirectory(uri);

    // Write the original data
    {
        // Create point table with dimension registry.
        PointTable table;
        table.layout()->registerDims(
            {Dimension::Id::X, Dimension::Id::Y, Dimension::Id::Z,
             Dimension::Id::Intensity, Dimension::Id::GpsTime});

        // Create buffer reader with data.
        BufferReader bufferReader;
        PointViewPtr view(new PointView(table));
        for (uint32_t index = 0; index < 6; ++index)
        {
            view->setField(Dimension::Id::X, index, xValues[index]);
            view->setField(Dimension::Id::Y, index, yValues[index]);
            view->setField(Dimension::Id::Z, index, zValues[index]);
            view->setField(Dimension::Id::Intensity, index,
                           intensityValues[index]);
            view->setField(Dimension::Id::GpsTime, index, gpsTimeValues[index]);
        }
        bufferReader.addView(view);

        // Set TileDB writer options.
        Options writerOptions;
        writerOptions.add("filename", uri);
        writerOptions.add("x_tile_size", 6);
        writerOptions.add("y_tile_size", 6);
        writerOptions.add("z_tile_size", 6);
        NL::json filters({});
        filters["Intensity"] = {
            {{"compression", "delta"}},
            {{"compression", "zstd"}, {"compression_level", 5}}};
        filters["GpsTime"] = {
            {{"compression", "delta"}, {"reinterpret_datatype", "UINT64"}},
            {{"compression", "bit_width_reduction"}},
            {{"compression", "zstd"}, {"compression_level", 7}}};
        writerOptions.add("filters", filters);

        // Create TileDB writer, set options, and write data.
        TileDBWriter writer;
        writer.setOptions(writerOptions);
        writer.setInput(bufferReader);
        writer.prepare(table);
#if TILEDB_VERSION_MAJOR == 2 && TILEDB_VERSION_MINOR < 16
        EXPECT_THROW(writer.execute(table), tiledb::TileDBError);
    }
#else
        writer.execute(table);
    }

    // Read back the TileDB array.
    {
        // Check the array schema.
        tiledb::Context ctx{};
        tiledb::ArraySchema schema(ctx, uri);

        auto gpsTimeFilterList = schema.attribute("GpsTime").filter_list();
        EXPECT_EQ(gpsTimeFilterList.nfilters(), (uint32_t)3);
        auto deltaFilter = gpsTimeFilterList.filter(0);
        EXPECT_EQ(deltaFilter.filter_type(), TILEDB_FILTER_DELTA);
        // schema.dump();
        // EXPECT_TRUE(false);

        tiledb_datatype_t delta_datatype{};
        deltaFilter.get_option(TILEDB_COMPRESSION_REINTERPRET_DATATYPE,
                               &delta_datatype);
        EXPECT_EQ(delta_datatype, TILEDB_UINT64);

        // Set reader options.
        Options readerOptions;
        readerOptions.add("filename", uri);

        // Read data back from TileDB.
        TileDBReader reader;
        reader.setOptions(readerOptions);
        PointTable readTable;
        reader.prepare(readTable);
        PointViewSet viewSet = reader.execute(readTable);
        PointViewPtr view = *viewSet.begin();

        // Check values.
        for (uint32_t index = 0; index < 6; ++index)
        {

            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::X, index),
                            xValues[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Y, index),
                            0.0);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Z, index),
                            0.0);
            EXPECT_FLOAT_EQ(
                view->getFieldAs<uint16_t>(Dimension::Id::Intensity, index),
                intensityValues[index]);
            EXPECT_FLOAT_EQ(
                view->getFieldAs<double>(Dimension::Id::GpsTime, index),
                gpsTimeValues[index]);
        }
    }
#endif
}
}; // namespace pdal
