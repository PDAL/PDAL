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

#include "gtest/gtest.h"

/**
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/concept_check.hpp>
**/

#include <pdal/FileUtils.hpp>
#include <pdal/StageFactory.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/pdal_defines.h>

#include "Support.hpp"

using namespace pdal;

Options getSQLITEOptions()
{
    Options options;

    options.add("capacity", 15U, "capacity");
    options.add("overwrite", true, "overwrite");
    options.add("connection",
        Support::temppath("temp-SqliteWriterTest_test_simple_las.sqlite"),
        "connection");
    options.add("block_table_name", "PDAL_TEST_BLOCKS", "block_table_name");
    options.add("cloud_table_name", "PDAL_TEST_BASE");
    options.add("is3d", false);
    options.add("srid", 4326);
    options.add("out_srs", "EPSG:4269");
    options.add("filename", Support::datapath("las/1.2-with-color.las"));
    options.add("query", "SELECT b.schema, l.cloud, l.block_id, "
        "l.num_points, l.bbox, l.extent, l.points, b.cloud "
        "FROM PDAL_TEST_BLOCKS l, PDAL_TEST_BASE b "
        "WHERE l.cloud = b.cloud and l.cloud in (1) "
        "order by l.cloud", "");
    options.add("spatialreference", "EPSG:2926");
    options.add("pack_ignored_fields", true);
    options.add("cloud_column_name", "CLOUD");
    options.add("xml_schema_dump", "sqlite-xml-schema-dump.xml");
    options.add("type", "sqlite");

    return options;
}

void testReadWrite(bool compression, bool scaling)
{
    // remove file from earlier run, if needed
    std::string tempFilename = 
        getSQLITEOptions().getValueOrThrow<std::string>("connection");

    StageFactory f;

    Options sqliteOptions = getSQLITEOptions();
    if (scaling)
    {
        sqliteOptions.add("scale_x", 0.01);
        sqliteOptions.add("scale_y", 0.01);
    }
    sqliteOptions.add("compression", compression, "");

    StageFactory::ReaderCreator *lasRc = f.getReaderCreator("readers.las");
    StageFactory::WriterCreator* sqliteWc =
        f.getWriterCreator("writers.sqlite");
    StageFactory::ReaderCreator* sqliteRc =
        f.getReaderCreator("readers.sqlite");

    EXPECT_TRUE(lasRc);
    EXPECT_TRUE(sqliteWc);
    EXPECT_TRUE(sqliteRc);
    if (!lasRc || !sqliteWc || !sqliteRc)
        return;

    // remove file from earlier run, if needed
    std::string temp_filename =
        sqliteOptions.getValueOrThrow<std::string>("connection");

    Options lasReadOpts;
    lasReadOpts.add("filename", Support::datapath("las/1.2-with-color.las"));
    lasReadOpts.add("count", 11);

    ReaderPtr lasReader(lasRc());
    lasReader->setOptions(lasReadOpts);

    WriterPtr sqliteWriter(sqliteWc());
    sqliteWriter->setOptions(sqliteOptions);
    sqliteWriter->setInput(lasReader.get());

    PointContext ctx;
    sqliteWriter->prepare(ctx);
    sqliteWriter->execute(ctx);

    // Done - now read back.
    ReaderPtr sqliteReader(sqliteRc());
    sqliteReader->setOptions(sqliteOptions);

    PointContext ctx2;
    sqliteReader->prepare(ctx2);
    PointBufferSet pbSet = sqliteReader->execute(ctx2);
    EXPECT_EQ(pbSet.size(), 1U);
    PointBufferPtr buffer = *pbSet.begin();

    using namespace Dimension;

    uint16_t reds[] = {68, 54, 112, 178, 134, 99, 90, 106, 106, 100, 64};
    for (PointId idx = 0; idx < 11; idx++)
    {
        uint16_t r = buffer->getFieldAs<uint16_t>(Id::Red, idx);
        EXPECT_EQ(r, reds[idx]);
    }
    int32_t x = buffer->getFieldAs<int32_t>(Id::X, 10);
    EXPECT_EQ(x, 636038);
    double xd = buffer->getFieldAs<double>(Id::X, 10);
    EXPECT_FLOAT_EQ(xd, 636037.53);

   FileUtils::deleteFile(tempFilename);
}


TEST(SQLiteTest, readWrite)
{
    testReadWrite(false, false);
}

#ifdef PDAL_HAVE_LAZPERF
TEST(SQLiteTest, readWriteCompress)
{
    testReadWrite(true, false);
}
#endif

TEST(SQLiteTest, readWriteScale)
{
    testReadWrite(false, true);
}

#ifdef PDAL_HAVE_LAZPERF
TEST(SQLiteTest, readWriteCompressScale)
{
    testReadWrite(true, true);
}
#endif
