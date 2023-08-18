/******************************************************************************
 * Copyright (c) 2023 TileDB, Inc
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

#include <stdio.h>

#include <nlohmann/json.hpp>

#include <pdal/Filter.hpp>
#include <pdal/pdal_test_main.hpp>
#include <pdal/util/FileUtils.hpp>

#include <io/BufferReader.hpp>
#include <io/FauxReader.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"

#include "../io/TileDBReader.hpp"
#include "../io/TileDBWriter.hpp"

namespace pdal
{

TEST(CombineBitFields, round_trip)
{
    // Create buffers with the data.
    std::vector<double> xData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> yData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> zData{0.0, 1.0, 2.0, 3.0};
    std::vector<uint8_t> returnNum{0, 15, 12, 7};  // 4 bits
    std::vector<uint8_t> numReturns{15, 13, 1, 0}; // 4 bits
    std::vector<uint8_t> classFlags{2, 6, 0, 15};  // 4 bits
    std::vector<uint8_t> scanChannel{0, 1, 2, 3};  // 2 bits
    std::vector<uint8_t> scanDirFlag{0, 0, 1, 1};  // 1 bit
    std::vector<uint8_t> flightEdge{1, 0, 1, 0};   // 1 bit

    // Clean-up previous tests.
    std::string uri = Support::temppath("tiledb_test_bit_fields");
    if (FileUtils::directoryExists(uri))
        FileUtils::deleteDirectory(uri);

    // Write to TileDB.
    {
        // Create point table with dimension registry.
        PointTable table;
        table.layout()->registerDims(
            {Dimension::Id::X, Dimension::Id::Y, Dimension::Id::Z,
             Dimension::Id::ReturnNumber, Dimension::Id::NumberOfReturns,
             Dimension::Id::ClassFlags, Dimension::Id::ScanChannel,
             Dimension::Id::ScanDirectionFlag,
             Dimension::Id::EdgeOfFlightLine});

        // Create buffer reader with data.
        BufferReader bufferReader;
        PointViewPtr view(new PointView(table));
        for (uint32_t index = 0; index < 4; ++index)
        {
            view->setField(Dimension::Id::X, index, xData[index]);
            view->setField(Dimension::Id::Y, index, yData[index]);
            view->setField(Dimension::Id::Z, index, zData[index]);
            view->setField(Dimension::Id::ReturnNumber, index,
                           returnNum[index]);
            view->setField(Dimension::Id::NumberOfReturns, index,
                           numReturns[index]);
            view->setField(Dimension::Id::ClassFlags, index, classFlags[index]);
            view->setField(Dimension::Id::ScanChannel, index,
                           scanChannel[index]);
            view->setField(Dimension::Id::ScanDirectionFlag, index,
                           scanDirFlag[index]);
            view->setField(Dimension::Id::EdgeOfFlightLine, index,
                           flightEdge[index]);
        }
        bufferReader.addView(view);

        // Set TileDB writer options.
        Options writerOptions;
        writerOptions.add("filename", uri);
        writerOptions.add("x_tile_size", 4);
        writerOptions.add("y_tile_size", 4);
        writerOptions.add("z_tile_size", 4);
        writerOptions.add("combine_bit_fields", true);

        // Create TileDB writer, set options, and write data.
        TileDBWriter writer;
        writer.setOptions(writerOptions);
        writer.setInput(bufferReader);
        writer.prepare(table);
        writer.execute(table);
    }

    // Read from TileDB.
    {
        // Check the array schema.
        tiledb::Context ctx{};
        tiledb::ArraySchema schema(ctx, uri);

        EXPECT_EQ(schema.attribute_num(), 1);
        EXPECT_TRUE(schema.has_attribute("BitFields"));
        auto attr = schema.attribute("BitFields");
        EXPECT_EQ(attr.type(), TILEDB_UINT16);
        EXPECT_EQ(attr.cell_val_num(), 1);

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
        for (uint32_t index = 0; index < 4; ++index)
        {
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::X, index),
                            xData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Y, index),
                            yData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Z, index),
                            zData[index]);
            EXPECT_EQ(
                view->getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, index),
                returnNum[index]);
            EXPECT_EQ(view->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns,
                                                index),
                      numReturns[index]);
            EXPECT_EQ(
                view->getFieldAs<uint8_t>(Dimension::Id::ClassFlags, index),
                classFlags[index]);
            EXPECT_EQ(
                view->getFieldAs<uint8_t>(Dimension::Id::ScanChannel, index),
                scanChannel[index]);
            EXPECT_EQ(view->getFieldAs<uint8_t>(
                          Dimension::Id::ScanDirectionFlag, index),
                      scanDirFlag[index]);
            EXPECT_EQ(view->getFieldAs<uint8_t>(Dimension::Id::EdgeOfFlightLine,
                                                index),
                      flightEdge[index]);
        }
    }
}

TEST(CombineBitFields, truncate_return_num)
{
    // Create buffers with the data.
    std::vector<double> xData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> yData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> zData{0.0, 1.0, 2.0, 3.0};

    // Return number will be truncated to 4 bits.
    std::vector<uint8_t> returnNum{0, 15, 240, 175};
    std::vector<uint8_t> returnNumExpected{0, 15, 0, 15};

    // Clean-up previous tests.
    std::string uri = Support::temppath("tiledb_test_return_num");
    if (FileUtils::directoryExists(uri))
        FileUtils::deleteDirectory(uri);

    // Write to TileDB.
    {
        // Create point table with dimension registry.
        PointTable table;
        table.layout()->registerDims({Dimension::Id::X, Dimension::Id::Y,
                                      Dimension::Id::Z,
                                      Dimension::Id::ReturnNumber});

        // Create buffer reader with data.
        BufferReader bufferReader;
        PointViewPtr view(new PointView(table));
        for (uint32_t index = 0; index < 4; ++index)
        {
            view->setField(Dimension::Id::X, index, xData[index]);
            view->setField(Dimension::Id::Y, index, yData[index]);
            view->setField(Dimension::Id::Z, index, zData[index]);
            view->setField(Dimension::Id::ReturnNumber, index,
                           returnNum[index]);
        }
        bufferReader.addView(view);

        // Set TileDB writer options.
        Options writerOptions;
        writerOptions.add("filename", uri);
        writerOptions.add("x_tile_size", 4);
        writerOptions.add("y_tile_size", 4);
        writerOptions.add("z_tile_size", 4);
        writerOptions.add("combine_bit_fields", true);

        // Create TileDB writer, set options, and write data.
        TileDBWriter writer;
        writer.setOptions(writerOptions);
        writer.setInput(bufferReader);
        writer.prepare(table);
        writer.execute(table);
    }

    // Read from TileDB.
    {
        // Check the array schema.
        tiledb::Context ctx{};
        tiledb::ArraySchema schema(ctx, uri);

        EXPECT_EQ(schema.attribute_num(), 1);
        EXPECT_TRUE(schema.has_attribute("BitFields"));
        auto attr = schema.attribute("BitFields");
        EXPECT_EQ(attr.type(), TILEDB_UINT16);
        EXPECT_EQ(attr.cell_val_num(), 1);

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
        for (uint32_t index = 0; index < 4; ++index)
        {
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::X, index),
                            xData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Y, index),
                            yData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Z, index),
                            zData[index]);
            EXPECT_EQ(
                view->getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, index),
                returnNumExpected[index]);
        }
    }
}

TEST(CombineBitFields, truncate_num_returns)
{
    // Create buffers with the data.
    std::vector<double> xData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> yData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> zData{0.0, 1.0, 2.0, 3.0};

    // Result will be truncated to 4 bits.
    std::vector<uint8_t> numReturns{0, 15, 240, 175};
    std::vector<uint8_t> numReturnsExpected{0, 15, 0, 15};

    // Clean-up previous tests.
    std::string uri = Support::temppath("tiledb_test_num_returns");
    if (FileUtils::directoryExists(uri))
        FileUtils::deleteDirectory(uri);

    // Write to TileDB.
    {
        // Create point table with dimension registry.
        PointTable table;
        table.layout()->registerDims({Dimension::Id::X, Dimension::Id::Y,
                                      Dimension::Id::Z,
                                      Dimension::Id::NumberOfReturns});

        // Create buffer reader with data.
        BufferReader bufferReader;
        PointViewPtr view(new PointView(table));
        for (uint32_t index = 0; index < 4; ++index)
        {
            view->setField(Dimension::Id::X, index, xData[index]);
            view->setField(Dimension::Id::Y, index, yData[index]);
            view->setField(Dimension::Id::Z, index, zData[index]);
            view->setField(Dimension::Id::NumberOfReturns, index,
                           numReturns[index]);
        }
        bufferReader.addView(view);

        // Set TileDB writer options.
        Options writerOptions;
        writerOptions.add("filename", uri);
        writerOptions.add("x_tile_size", 4);
        writerOptions.add("y_tile_size", 4);
        writerOptions.add("z_tile_size", 4);
        writerOptions.add("combine_bit_fields", true);

        // Create TileDB writer, set options, and write data.
        TileDBWriter writer;
        writer.setOptions(writerOptions);
        writer.setInput(bufferReader);
        writer.prepare(table);
        writer.execute(table);
    }

    // Read from TileDB.
    {
        // Check the array schema.
        tiledb::Context ctx{};
        tiledb::ArraySchema schema(ctx, uri);

        EXPECT_EQ(schema.attribute_num(), 1);
        EXPECT_TRUE(schema.has_attribute("BitFields"));
        auto attr = schema.attribute("BitFields");
        EXPECT_EQ(attr.type(), TILEDB_UINT16);
        EXPECT_EQ(attr.cell_val_num(), 1);

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
        for (uint32_t index = 0; index < 4; ++index)
        {
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::X, index),
                            xData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Y, index),
                            yData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Z, index),
                            zData[index]);
            EXPECT_EQ(view->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns,
                                                index),
                      numReturnsExpected[index]);
        }
    }
}

TEST(CombineBitFields, truncate_class_flags)
{
    // Create buffers with the data.
    std::vector<double> xData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> yData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> zData{0.0, 1.0, 2.0, 3.0};

    // Result will be truncated to 4 bits.
    std::vector<uint8_t> classFlags{0, 15, 240, 175};
    std::vector<uint8_t> classFlagsExpected{0, 15, 0, 15};

    // Clean-up previous tests.
    std::string uri = Support::temppath("tiledb_test_class_flags");
    if (FileUtils::directoryExists(uri))
        FileUtils::deleteDirectory(uri);

    // Write to TileDB.
    {
        // Create point table with dimension registry.
        PointTable table;
        table.layout()->registerDims({Dimension::Id::X, Dimension::Id::Y,
                                      Dimension::Id::Z,
                                      Dimension::Id::ClassFlags});

        // Create buffer reader with data.
        BufferReader bufferReader;
        PointViewPtr view(new PointView(table));
        for (uint32_t index = 0; index < 4; ++index)
        {
            view->setField(Dimension::Id::X, index, xData[index]);
            view->setField(Dimension::Id::Y, index, yData[index]);
            view->setField(Dimension::Id::Z, index, zData[index]);
            view->setField(Dimension::Id::ClassFlags, index, classFlags[index]);
        }
        bufferReader.addView(view);

        // Set TileDB writer options.
        Options writerOptions;
        writerOptions.add("filename", uri);
        writerOptions.add("x_tile_size", 4);
        writerOptions.add("y_tile_size", 4);
        writerOptions.add("z_tile_size", 4);
        writerOptions.add("combine_bit_fields", true);

        // Create TileDB writer, set options, and write data.
        TileDBWriter writer;
        writer.setOptions(writerOptions);
        writer.setInput(bufferReader);
        writer.prepare(table);
        writer.execute(table);
    }

    // Read from TileDB.
    {
        // Check the array schema.
        tiledb::Context ctx{};
        tiledb::ArraySchema schema(ctx, uri);

        EXPECT_EQ(schema.attribute_num(), 1);
        EXPECT_TRUE(schema.has_attribute("BitFields"));
        auto attr = schema.attribute("BitFields");
        EXPECT_EQ(attr.type(), TILEDB_UINT16);
        EXPECT_EQ(attr.cell_val_num(), 1);

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
        for (uint32_t index = 0; index < 4; ++index)
        {
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::X, index),
                            xData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Y, index),
                            yData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Z, index),
                            zData[index]);
            EXPECT_EQ(
                view->getFieldAs<uint8_t>(Dimension::Id::ClassFlags, index),
                classFlagsExpected[index]);
        }
    }
}

TEST(CombineBitFields, truncate_scan_channel)
{
    // Create buffers with the data.
    std::vector<double> xData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> yData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> zData{0.0, 1.0, 2.0, 3.0};

    // Result will be truncated to 2 bit.
    std::vector<uint8_t> scanChannel{0, 3, 252, 171};
    std::vector<uint8_t> scanChannelExpected{0, 3, 0, 3};

    // Clean-up previous tests.
    std::string uri = Support::temppath("tiledb_test_scan_channel");
    if (FileUtils::directoryExists(uri))
        FileUtils::deleteDirectory(uri);

    // Write to TileDB.
    {
        // Create point table with dimension registry.
        PointTable table;
        table.layout()->registerDims({Dimension::Id::X, Dimension::Id::Y,
                                      Dimension::Id::Z,
                                      Dimension::Id::ScanChannel});

        // Create buffer reader with data.
        BufferReader bufferReader;
        PointViewPtr view(new PointView(table));
        for (uint32_t index = 0; index < 4; ++index)
        {
            view->setField(Dimension::Id::X, index, xData[index]);
            view->setField(Dimension::Id::Y, index, yData[index]);
            view->setField(Dimension::Id::Z, index, zData[index]);
            view->setField(Dimension::Id::ScanChannel, index,
                           scanChannel[index]);
        }
        bufferReader.addView(view);

        // Set TileDB writer options.
        Options writerOptions;
        writerOptions.add("filename", uri);
        writerOptions.add("x_tile_size", 4);
        writerOptions.add("y_tile_size", 4);
        writerOptions.add("z_tile_size", 4);
        writerOptions.add("combine_bit_fields", true);

        // Create TileDB writer, set options, and write data.
        TileDBWriter writer;
        writer.setOptions(writerOptions);
        writer.setInput(bufferReader);
        writer.prepare(table);
        writer.execute(table);
    }

    // Read from TileDB.
    {
        // Check the array schema.
        tiledb::Context ctx{};
        tiledb::ArraySchema schema(ctx, uri);

        EXPECT_EQ(schema.attribute_num(), 1);
        EXPECT_TRUE(schema.has_attribute("BitFields"));
        auto attr = schema.attribute("BitFields");
        EXPECT_EQ(attr.type(), TILEDB_UINT16);
        EXPECT_EQ(attr.cell_val_num(), 1);

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
        for (uint32_t index = 0; index < 4; ++index)
        {
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::X, index),
                            xData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Y, index),
                            yData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Z, index),
                            zData[index]);
            EXPECT_EQ(
                view->getFieldAs<uint8_t>(Dimension::Id::ScanChannel, index),
                scanChannelExpected[index]);
        }
    }
}

TEST(CombineBitFields, truncate_scan_dir_flag)
{
    // Create buffers with the data.
    std::vector<double> xData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> yData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> zData{0.0, 1.0, 2.0, 3.0};

    // Result will be truncated to 1 bit.
    std::vector<uint8_t> scanDirFlag{0, 1, 254, 171};
    std::vector<uint8_t> scanDirFlagExpected{0, 1, 0, 1};

    // Clean-up previous tests.
    std::string uri = Support::temppath("tiledb_test_intensity_compression");
    if (FileUtils::directoryExists(uri))
        FileUtils::deleteDirectory(uri);

    // Write to TileDB.
    {
        // Create point table with dimension registry.
        PointTable table;
        table.layout()->registerDims({Dimension::Id::X, Dimension::Id::Y,
                                      Dimension::Id::Z,
                                      Dimension::Id::ScanDirectionFlag});

        // Create buffer reader with data.
        BufferReader bufferReader;
        PointViewPtr view(new PointView(table));
        for (uint32_t index = 0; index < 4; ++index)
        {
            view->setField(Dimension::Id::X, index, xData[index]);
            view->setField(Dimension::Id::Y, index, yData[index]);
            view->setField(Dimension::Id::Z, index, zData[index]);
            view->setField(Dimension::Id::ScanDirectionFlag, index,
                           scanDirFlag[index]);
        }
        bufferReader.addView(view);

        // Set TileDB writer options.
        Options writerOptions;
        writerOptions.add("filename", uri);
        writerOptions.add("x_tile_size", 4);
        writerOptions.add("y_tile_size", 4);
        writerOptions.add("z_tile_size", 4);
        writerOptions.add("combine_bit_fields", true);

        // Create TileDB writer, set options, and write data.
        TileDBWriter writer;
        writer.setOptions(writerOptions);
        writer.setInput(bufferReader);
        writer.prepare(table);
        writer.execute(table);
    }

    // Read from TileDB.
    {
        // Check the array schema.
        tiledb::Context ctx{};
        tiledb::ArraySchema schema(ctx, uri);

        EXPECT_EQ(schema.attribute_num(), 1);
        EXPECT_TRUE(schema.has_attribute("BitFields"));
        auto attr = schema.attribute("BitFields");
        EXPECT_EQ(attr.type(), TILEDB_UINT16);
        EXPECT_EQ(attr.cell_val_num(), 1);

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
        for (uint32_t index = 0; index < 4; ++index)
        {
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::X, index),
                            xData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Y, index),
                            yData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Z, index),
                            zData[index]);
            EXPECT_EQ(view->getFieldAs<uint8_t>(
                          Dimension::Id::ScanDirectionFlag, index),
                      scanDirFlagExpected[index]);
        }
    }
}

TEST(CombineBitFields, truncate_flight_edge)
{
    // Create buffers with the data.
    std::vector<double> xData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> yData{0.0, 1.0, 2.0, 3.0};
    std::vector<double> zData{0.0, 1.0, 2.0, 3.0};

    // Result will be truncated to 1 bit.
    std::vector<uint8_t> flightEdge{1, 0, 171, 254};
    std::vector<uint8_t> flightEdgeExpected{1, 0, 1, 0};

    // Clean-up previous tests.
    std::string uri = Support::temppath("tiledb_test_flight_edge");
    if (FileUtils::directoryExists(uri))
        FileUtils::deleteDirectory(uri);

    // Write to TileDB.
    {
        // Create point table with dimension registry.
        PointTable table;
        table.layout()->registerDims({Dimension::Id::X, Dimension::Id::Y,
                                      Dimension::Id::Z,
                                      Dimension::Id::EdgeOfFlightLine});

        // Create buffer reader with data.
        BufferReader bufferReader;
        PointViewPtr view(new PointView(table));
        for (uint32_t index = 0; index < 4; ++index)
        {
            view->setField(Dimension::Id::X, index, xData[index]);
            view->setField(Dimension::Id::Y, index, yData[index]);
            view->setField(Dimension::Id::Z, index, zData[index]);
            view->setField(Dimension::Id::EdgeOfFlightLine, index,
                           flightEdge[index]);
        }
        bufferReader.addView(view);

        // Set TileDB writer options.
        Options writerOptions;
        writerOptions.add("filename", uri);
        writerOptions.add("x_tile_size", 4);
        writerOptions.add("y_tile_size", 4);
        writerOptions.add("z_tile_size", 4);
        writerOptions.add("combine_bit_fields", true);

        // Create TileDB writer, set options, and write data.
        TileDBWriter writer;
        writer.setOptions(writerOptions);
        writer.setInput(bufferReader);
        writer.prepare(table);
        writer.execute(table);
    }

    // Read from TileDB.
    {
        // Check the array schema.
        tiledb::Context ctx{};
        tiledb::ArraySchema schema(ctx, uri);

        EXPECT_EQ(schema.attribute_num(), 1);
        EXPECT_TRUE(schema.has_attribute("BitFields"));
        auto attr = schema.attribute("BitFields");
        EXPECT_EQ(attr.type(), TILEDB_UINT16);
        EXPECT_EQ(attr.cell_val_num(), 1);

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
        for (uint32_t index = 0; index < 4; ++index)
        {
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::X, index),
                            xData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Y, index),
                            yData[index]);
            EXPECT_FLOAT_EQ(view->getFieldAs<double>(Dimension::Id::Z, index),
                            zData[index]);
            EXPECT_EQ(view->getFieldAs<uint8_t>(Dimension::Id::EdgeOfFlightLine,
                                                index),
                      flightEdgeExpected[index]);
        }
    }
}

} // namespace pdal
