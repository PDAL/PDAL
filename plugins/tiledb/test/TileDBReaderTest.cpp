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

#include <pdal/Filter.hpp>
#include <pdal/pdal_test_main.hpp>

#include <pdal/PointView.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <io/FauxReader.hpp>

#include "Support.hpp"

#include "../io/TileDBReader.hpp"
#include "../io/TileDBWriter.hpp"


namespace pdal
{

const SpatialReference utm16("EPSG:26916");

class TileDBReaderTest : public ::testing::Test
{
    protected:
        virtual void SetUp()
        {
            tiledb::Context ctx;
            FauxReader rdr;
            TileDBWriter writer;
            Options writer_options;
            Options reader_options;

            if (Utils::fileExists(data_path))
                 tiledb::Object::remove(ctx, data_path);

            writer_options.add("array_name", data_path);
            writer_options.add("x_tile_size", 1);
            writer_options.add("y_tile_size", 1);
            writer_options.add("z_tile_size", 1);

            reader_options.add("mode", "ramp");
            reader_options.add("count", 100);
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
        tiledb::Context ctx;
        tiledb::VFS vfs(ctx);
        Options options;
        options.add("array_name", data_path);
        options.add("bbox3d", "([0, 0.5], [0, 0.5], [0, 0.5])");

        TileDBReader reader;
        reader.setOptions(options);

        FixedPointTable table(100);
        reader.prepare(table);
        reader.execute(table);
        EXPECT_EQ(table.numPoints(), 50);
    }

    TEST_F(TileDBReaderTest, read_zero_bbox)
    {
        tiledb::Context ctx;
        tiledb::VFS vfs(ctx);
        Options options;
        options.add("array_name", data_path);
        options.add("bbox3d", "([1.1, 1.2], [1.1, 1.2], [1.1, 1.2])");

        TileDBReader reader;
        reader.setOptions(options);

        FixedPointTable table(100);
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
                { return "checker"; }
        private:
            bool processOne(PointRef& point)
            {
                static int cnt = 0;
                if (cnt == 0)
                {
                    EXPECT_NEAR(0.f,
                        point.getFieldAs<double>(Dimension::Id::X), 1e-1);
                    EXPECT_NEAR(0.f,
                        point.getFieldAs<double>(Dimension::Id::Y), 1e-1);
                    EXPECT_NEAR(0.f,
                        point.getFieldAs<double>(Dimension::Id::Z), 1e-1);
                    EXPECT_EQ(0,
                        point.getFieldAs<int>(Dimension::Id::OffsetTime));
                }
                if (cnt == 1)
                {
                    EXPECT_NEAR(0.010101f,
                        point.getFieldAs<double>(Dimension::Id::X), 1e-5);
                    EXPECT_NEAR(0.010101f,
                        point.getFieldAs<double>(Dimension::Id::Y), 1e-5);
                    EXPECT_NEAR(0.010101f,
                        point.getFieldAs<double>(Dimension::Id::Z), 1e-5);
                    EXPECT_EQ(1,
                        point.getFieldAs<int>(Dimension::Id::OffsetTime));
                }
                if (cnt == 2)
                {
                    EXPECT_NEAR(0.020202f,
                        point.getFieldAs<double>(Dimension::Id::X), 1e-5);
                    EXPECT_NEAR(0.020202f,
                        point.getFieldAs<double>(Dimension::Id::Y), 1e-5);
                    EXPECT_NEAR(0.020202f,
                        point.getFieldAs<double>(Dimension::Id::Z), 1e-5);
                    EXPECT_EQ(2,
                        point.getFieldAs<int>(Dimension::Id::OffsetTime));
                }
                cnt++;
                return true;
            }
        };

        tiledb::Context ctx;
        tiledb::VFS vfs(ctx);
        Options options;
        options.add("array_name", data_path);

        tiledb::Array array(ctx, data_path, TILEDB_READ);
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

        TileDBReader reader;
        reader.setOptions(options);

        FixedPointTable table(100);

        Checker c;
        c.setInput(reader);
        c.prepare(table);
        c.execute(table);
        EXPECT_TRUE(reader.getSpatialReference().equals(utm16));
    }

    TEST_F(TileDBReaderTest, spatial_reference)
    {
        tiledb::Context ctx;
        tiledb::VFS vfs(ctx);
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

        if (vfs.is_dir(pth))
        {
            vfs.remove_dir(pth);
        }

        FauxReader reader;
        Options reader_options;
        reader_options.add("mode", "ramp");
        reader_options.add("count", 50);
        reader.addOptions(reader_options);

        TileDBWriter writer;
        writer.setOptions(writer_options);
        writer.setInput(reader);
        writer.setSpatialReference(utm16);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);

        TileDBReader rdr;
        rdr.setOptions(options);
        FixedPointTable table2(100);
        rdr.prepare(table2);
        rdr.execute(table2);
        EXPECT_TRUE(rdr.getSpatialReference().equals(utm16));
    }
}

