/******************************************************************************
 * Copyright (c) 2021 TileDB, Inc
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
#include <sys/types.h>

#include <filters/StatsFilter.hpp>
#include <io/FauxReader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_test_main.hpp>
#include <pdal/util/FileUtils.hpp>

#include "Support.hpp"

#include "../io/TileDBReader.hpp"
#include "../io/TileDBWriter.hpp"
#include "XYZTmUtils.hpp"

namespace pdal
{

const size_t count = 100;

class TileDBReaderTimeDimTest : public ::testing::TestWithParam<bool>
{
public:
    void SetUp() override
    {
        TileDBWriter writer;
        Options writer_options;

        data_path = Support::temppath("xyztm_tdb_array");
        m_time_first = GetParam();

        if (FileUtils::directoryExists(data_path))
            FileUtils::deleteDirectory(data_path);

        writer_options.add("array_name", data_path);
        writer_options.add("use_time_dim", true);
        writer_options.add("x_tile_size", 10.f);
        writer_options.add("y_tile_size", 10.f);
        writer_options.add("z_tile_size", 10.f);
        writer_options.add("time_tile_size", 777600.f);
        writer_options.add("time_first", m_time_first);

        Options reader_options;
        reader_options.add(
            "bounds",
            DomainBounds(
                0., 0., 0., 1314489618., 10., 10., 10.,
                1315353618.)); // sep 1 - sep 11 00:00:00 UTC 2021 -> gpstime
        reader_options.add("count", count);
        reader_options.add("xyz_mode", "ramp");
        reader_options.add("time_mode", "ramp");

        XYZTimeFauxReader rdr;
        rdr.setOptions(reader_options);

        writer.setOptions(writer_options);
        writer.setInput(rdr);

        FixedPointTable table(count);
        writer.prepare(table);
        writer.execute(table);
    }
    std::string data_path;

    bool time_is_first() const
    {
        return m_time_first;
    }

private:
    bool m_time_first;
};

TEST_P(TileDBReaderTimeDimTest, set_dims)
{
    tiledb::Context ctx;

    tiledb::Array array(ctx, data_path, TILEDB_READ);
    tiledb::Domain domain(array.schema().domain());
    if (time_is_first())
    {
        EXPECT_EQ(domain.dimension(0).name(), "GpsTime");
        EXPECT_EQ(domain.dimension(1).name(), "X");
        EXPECT_EQ(domain.dimension(2).name(), "Y");
        EXPECT_EQ(domain.dimension(3).name(), "Z");
    }
    else
    {
        EXPECT_EQ(domain.dimension(0).name(), "X");
        EXPECT_EQ(domain.dimension(1).name(), "Y");
        EXPECT_EQ(domain.dimension(2).name(), "Z");
        EXPECT_EQ(domain.dimension(3).name(), "GpsTime");
    }
}

TEST_P(TileDBReaderTimeDimTest, read_bbox4d)
{
    Options options;
    options.add("array_name", data_path);
    options.add(
        "bbox4d",
        "([2., 7.], [2., 7.], [2., 7.], [1314662418., 1315094418.])"); // sep 3
                                                                       // - sep
                                                                       // 8

    TileDBReader reader;
    reader.setOptions(options);

    FixedPointTable table(count);
    reader.prepare(table);
    reader.execute(table);

    EXPECT_EQ(table.numPoints(), 50);
}

TEST_P(TileDBReaderTimeDimTest, read_4d)
{
    class Checker4D : public Filter, public Streamable
    {
    public:
        std::string getName() const
        {
            return "checker4d";
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
                EXPECT_NEAR(1314489618,
                            point.getFieldAs<double>(Dimension::Id::GpsTime),
                            10);
                EXPECT_DOUBLE_EQ(
                    1.0f, point.getFieldAs<double>(Dimension::Id::Density));
            }
            if (cnt == 1)
            {
                EXPECT_NEAR(0.101f, point.getFieldAs<double>(Dimension::Id::X),
                            1e-1);
                EXPECT_NEAR(0.101f, point.getFieldAs<double>(Dimension::Id::Y),
                            1e-1);
                EXPECT_NEAR(0.101f, point.getFieldAs<double>(Dimension::Id::Z),
                            1e-1);
                EXPECT_NEAR(1314498345,
                            point.getFieldAs<double>(Dimension::Id::GpsTime),
                            10);
                EXPECT_DOUBLE_EQ(
                    1.0, point.getFieldAs<double>(Dimension::Id::Density));
            }
            if (cnt == 2)
            {
                EXPECT_NEAR(0.202f, point.getFieldAs<double>(Dimension::Id::X),
                            1e-1);
                EXPECT_NEAR(0.202f, point.getFieldAs<double>(Dimension::Id::Y),
                            1e-1);
                EXPECT_NEAR(0.202f, point.getFieldAs<double>(Dimension::Id::Z),
                            1e-1);
                EXPECT_NEAR(1314507072,
                            point.getFieldAs<double>(Dimension::Id::GpsTime),
                            10);
                EXPECT_DOUBLE_EQ(
                    1.0, point.getFieldAs<double>(Dimension::Id::Density));
            }
            cnt++;
            return true;
        }
    };
    tiledb::Context ctx;
    Options options;
    options.add("array_name", data_path);

    tiledb::Array array(ctx, data_path, TILEDB_READ);
    tiledb::Query q(ctx, array, TILEDB_READ);

    std::vector<double> xs(count);
    std::vector<double> ys(count);
    std::vector<double> zs(count);
    std::vector<double> ts(count);

    q.set_data_buffer("X", xs)
        .set_data_buffer("Y", ys)
        .set_data_buffer("Z", zs)
        .set_data_buffer("GpsTime", ts);

    q.submit();
    array.close();

    TileDBReader reader;
    reader.setOptions(options);

    FixedPointTable table(count);

    Checker4D c4d;
    c4d.setInput(reader);
    c4d.prepare(table);
    c4d.execute(table);
}

TEST_P(TileDBReaderTimeDimTest, test_dim4_change_name)
{
    Options input_options;
    input_options.add("bounds",
                      DomainBounds(0., 0., 0., 0., 10., 10., 10., 10.));
    input_options.add("count", count);
    input_options.add("xyz_mode", "ramp");
    input_options.add("time_mode", "ramp");
    input_options.add("dim4_name", "something");

    XYZTimeFauxReader faux_reader;
    faux_reader.setOptions(input_options);

    FixedPointTable table(count);
    faux_reader.prepare(table);
    faux_reader.execute(table);

    EXPECT_TRUE(table.layout()->dimName(table.layout()->findDim("something")) ==
                "something");
}

INSTANTIATE_TEST_SUITE_P(TileDBTimeDimTest, TileDBReaderTimeDimTest,
                         testing::Values(true, false));

} // namespace pdal
