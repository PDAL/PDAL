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

#define NOMINMAX

#include <stdio.h>
#include <sys/types.h>


#include <filters/StatsFilter.hpp>
#include <pdal/pdal_test_main.hpp>
#include <io/FauxReader.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"

#include "../io/TileDBReader.hpp"
#include "../io/TileDBWriter.hpp"
#include "XYZTmUtils.hpp"

namespace pdal
{

class TDBReaderTimeDimTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        tiledb::Context ctx;
        XYZTimeFauxReader rdr;
        TileDBWriter writer;
        Options reader_options;
        Options writer_options;

        if (Utils::fileExists(tm_data_path))
            tiledb::Object::remove(ctx, tm_data_path);

        writer_options.add("array_name", tm_data_path);
        writer_options.add("use_time_dim", true);
        writer_options.add("x_tile_size", 2.f);
        writer_options.add("y_tile_size", 2.f);
        writer_options.add("z_tile_size", 2.f);
        writer_options.add("time_tile_size", 777600.f);

        reader_options.add("bounds", BOX4D(0., 0., 0., 1314489618., 2., 2., 2., 1315267218.)); //sep 1 - sep 10 2021 gpstime
        reader_options.add("count", 10);
        rdr.setOptions(reader_options);

        writer.setOptions(writer_options);
        writer.setInput(rdr);

        FixedPointTable table(100);
        writer.prepare(table);
        writer.execute(table);
    }
    std::string tm_data_path = Support::temppath("test_time_dim_write");

};

TEST_F(TDBReaderTimeDimTest, set_dims)
{
    //    tiledb::Context ctx;
    //    tiledb::VFS vfs(ctx);
    //
    //    tiledb::Array array(ctx, tm_data_path, TILEDB_READ);
    //    std::cout << array.schema().domain().ndim();
    //    std::vector<std::string> dim_names{"X", "Y", "Z", "GpsTime"};
    //    for (size_t i(0); i != 3; ++i)
    //        EXPECT_EQ(domain.at(i).first, dim_names[i]);
}
//    Options reader_options;
//    reader_options.add("array_name", tm_data_path);
//    reader_options.add("use_time", true);
//
//    TileDBReader reader;



} // pdal namespace

